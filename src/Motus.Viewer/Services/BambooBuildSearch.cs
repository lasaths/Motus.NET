using Motus.Core;

namespace Motus.Viewer.Services;

/// <summary>
/// Derives the build order and the walk / relay sequence from the structure graph and the reach map, instead of a
/// hand-ordered task list. Depth-first over build states (placed members + where the robot stands): at each state the
/// frontier members (<see cref="BambooStructureGraph.Frontier"/>) are ranked by the cheapest way the reach map — with
/// the real set of fixed members of that state — allows: direct from where the robot stands, one targeted-landing walk,
/// or one relay (park in a node connector; candidates = the scene check's single-park options). A reach-map pass is
/// necessary, not sufficient: every choice is confirmed with the real planner (Motus-clear, PTP) before it is kept, and
/// rolled back (planner state and random stream) when it or anything after it fails.
/// </summary>
public static class BambooBuildSearch
{
    public enum Mode { Direct, Land, Relay }

    public sealed record Choice(string MemberId, Mode Mode, string? Relay, bool WalkToPark, bool WalkToPlace, string Note);

    public sealed record Result(List<BambooIcdMotion.TaskEdit> Tasks, List<Choice> Order, List<string> Log, bool Complete,
        int Confirmed, int Rejected, double Seconds)
    {
        public int Attempts => Confirmed + Rejected;
        public string StopReason { get; init; } = "";
    }

    public sealed record Options
    {
        /// <summary>Grid for reach-map sites on fixed canes (fraction of half length − 3 cm) and clamp rolls.</summary>
        public IReadOnlyList<double> CaneOffsets { get; init; } = new[] { -1.0, -0.5, 0.0, 0.5, 1.0 };
        public IReadOnlyList<double> Rolls { get; init; } = Enumerable.Range(0, 8).Select(i => i * 45.0).ToArray();
        /// <summary>Stop after this many rejected planner confirmations (the search reports how far it got).</summary>
        public int MaxRejected { get; init; } = 40;
        /// <summary>Total planner confirmations, including successful choices whose continuation reaches a dead end.</summary>
        public int MaxConfirmations { get; init; } = 1000;

        internal Options Validate()
        {
            if (MaxRejected < 0) throw new ArgumentOutOfRangeException(nameof(MaxRejected));
            if (MaxConfirmations < 0) throw new ArgumentOutOfRangeException(nameof(MaxConfirmations));
            if (CaneOffsets is null || CaneOffsets.Count == 0 || CaneOffsets.Count > 64
                || CaneOffsets.Any(f => !double.IsFinite(f) || f < -1 || f > 1))
                throw new ArgumentException("Cane offsets must contain 1–64 finite fractions in [-1, 1].", nameof(CaneOffsets));
            if (Rolls is null || Rolls.Count == 0 || Rolls.Count > 72 || Rolls.Any(r => !double.IsFinite(r)))
                throw new ArgumentException("Clamp rolls must contain 1–72 finite angles in degrees.", nameof(Rolls));
            if (CaneOffsets.Count * Rolls.Count > 512)
                throw new ArgumentException("The grid must contain at most 512 sites per cane.");
            return this;
        }
    }

    sealed class Grid
    {
        public List<BambooReachMap.ClampSite> Sites = new();
        public List<string> Owner = new();          // anchor column name or member id
        public Dictionary<string, int> Line = new(); // "stock", member id, relay id
        public BambooReachMap.Reach[,] R = new BambooReachMap.Reach[0, 0];
        public bool Ok(int s, string line) => Line.TryGetValue(line, out var l) && R[s, l].Ik;
    }

    public static Result Run(BambooIcdCell cell, Options? options = null, Action<string>? log = null)
    {
        ArgumentNullException.ThrowIfNull(cell);
        var o = (options ?? new Options()).Validate();
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var graph = BambooStructureGraph.FromLayout();
        var lines = new List<string>();
        void Log(string s) { lines.Add(s); log?.Invoke(s); }

        if (o.MaxRejected == 0 || o.MaxConfirmations == 0)
        {
            const string reason = "planner confirmation budget is zero";
            Log($"build search: incomplete — 0/{graph.Edges.Count} placed; {reason}");
            return new Result(new(), new(), lines, false, 0, 0, clock.Elapsed.TotalSeconds) { StopReason = reason };
        }

        // Scene check: optimistic necessary condition, and the relay candidate sets.
        var check = BambooSceneCheck.Run(null, BambooTrussLayout.Members);
        Log(check.Summary);
        if (!check.Ok)
        {
            foreach (var detail in check.Details) Log("scene check · " + detail);
            return new Result(new(), new(), lines, false, 0, 0, clock.Elapsed.TotalSeconds) { StopReason = "scene check failed" };
        }
        var relayOptions = check.Members.ToDictionary(m => m.MemberId,
            m => m.RelayOptions.Select(r => BambooTrussLayout.RelayId(m.MemberId, r.Split('/')[0], r.Split('/')[1])).ToList());

        var grid = BuildGrid(o, relayOptions.Values.SelectMany(x => x));
        Log($"reach grid: {grid.Sites.Count} sites × {grid.Line.Count} lines");

        var p = new BambooTrussPlanner(cell);
        if (!p.Start()) { Log("planner start failed: " + p.Error); return new Result(new(), new(), lines, false, 0, 0, clock.Elapsed.TotalSeconds) { StopReason = "planner start failed: " + p.Error }; }
        var start = BambooTrussLayout.StakeNamed(BambooTrussLayout.Current.StartStake);
        var placed = new List<string>();
        var order = new List<Choice>();
        var tasks = new List<BambooIcdMotion.TaskEdit>();
        var bestOrder = new List<Choice>();
        var bestTasks = new List<BambooIcdMotion.TaskEdit>();
        int confirmed = 0, rejected = 0;
        string stopReason = "search exhausted";

        bool BudgetReached()
        {
            if (rejected >= o.MaxRejected) { stopReason = $"rejected confirmation limit ({o.MaxRejected})"; return true; }
            if (confirmed + rejected >= o.MaxConfirmations) { stopReason = $"confirmation limit ({o.MaxConfirmations})"; return true; }
            return false;
        }

        bool Dfs()
        {
            if (placed.Count == graph.Edges.Count) return true;
            if (BudgetReached()) return false;

            var fixedIds = graph.Fixed(placed).ToHashSet(StringComparer.Ordinal);
            var stand = Stand(p);
            var options = new List<(string Id, Mode Mode, string? Relay, double Rank)>();
            foreach (var id in graph.Frontier(placed))
            {
                var m = BambooTrussLayout.Def(id)!;
                var usable = Enumerable.Range(0, grid.Sites.Count)
                    .Where(s => grid.Owner[s] != id && (BambooTrussLayout.IsSupport(grid.Owner[s]) || fixedIds.Contains(grid.Owner[s]))).ToList();
                // Far members first: what is built later must not block reaching what is built earlier.
                var far = Dist(m.Place, start);
                var map = new BambooReachMap { IkSeeds = 12 };
                if (map.Query(stand, BambooReachMap.Stock()).Ik && map.Query(stand, BambooReachMap.PlaceOf(m)).Ik)
                    options.Add((id, Mode.Direct, null, -far));
                if (usable.Any(s => grid.Ok(s, "stock") && grid.Ok(s, id)))
                    options.Add((id, Mode.Land, null, 10 - far));
                foreach (var r in relayOptions.GetValueOrDefault(id) ?? new List<string>())
                {
                    if (!graph.ExistingNodes(placed).Contains(BambooTrussLayout.RelayNode(r))) continue;
                    // Stock → park from where the robot stands or one landing; park → place from one landing.
                    var toPark = (map.Query(stand, BambooReachMap.Stock()).Ik && map.Query(stand, BambooReachMap.PlaceOf(BambooTrussLayout.Def(r)!)).Ik)
                        || usable.Any(s => grid.Ok(s, "stock") && grid.Ok(s, r));
                    if (toPark && usable.Any(s => grid.Ok(s, r) && grid.Ok(s, id)))
                        options.Add((id, Mode.Relay, r, 20 - far));
                }
            }
            foreach (var (id, mode, relay, _) in options.OrderBy(x => x.Rank))
            {
                if (BudgetReached()) break;
                var cp = p.SaveDeep();
                var choice = Confirm(p, id, mode, relay, Log);
                if (choice is null) { rejected++; p.RollbackDeep(cp); continue; }
                confirmed++;
                placed.Add(id); order.Add(choice);
                var nt = tasks.Count;
                tasks.AddRange(TasksFor(choice));
                if (order.Count > bestOrder.Count) { bestOrder = order.ToList(); bestTasks = tasks.ToList(); }
                Log($"{placed.Count}/{graph.Edges.Count} {choice.Note}");
                if (Dfs()) return true;
                Log($"backtrack: {id} ({mode}) leads to a dead end");
                placed.RemoveAt(placed.Count - 1); order.RemoveAt(order.Count - 1);
                tasks.RemoveRange(nt, tasks.Count - nt);
                p.RollbackDeep(cp);
            }
            return false;
        }

        var complete = Dfs();
        Log(complete ? $"build search: {string.Join(" → ", order.Select(c => c.MemberId + (c.Mode == Mode.Direct ? "" : c.Mode == Mode.Land ? " (land)" : $" (relay {c.Relay})")))} — {confirmed} confirmed, {rejected} rejected, {clock.Elapsed.TotalSeconds:F0} s"
            : $"build search: incomplete — best confirmed prefix {bestOrder.Count}/{graph.Edges.Count} placed, {confirmed} confirmed, {rejected} rejected; {stopReason}");
        cell.SetMountViewer(BambooTrussLayout.StartBase);
        return new Result(complete ? tasks : bestTasks, complete ? order : bestOrder, lines, complete, confirmed, rejected, clock.Elapsed.TotalSeconds)
        { StopReason = complete ? "complete" : stopReason };
    }

    /// <summary>Confirm one choice with the real planner (the same calls <see cref="BambooTrussMotion.RunSequence"/> makes).</summary>
    static Choice? Confirm(BambooTrussPlanner p, string id, Mode mode, string? relay, Action<string> log)
    {
        var m = BambooTrussLayout.Def(id)!;
        switch (mode)
        {
            case Mode.Direct:
                if (!p.BuildMember(m)) { log($"  {id} direct: rejected by the planner — {p.Error}"); return null; }
                return new Choice(id, mode, null, false, false, $"{id}: direct from {p.LStake}");
            case Mode.Land:
            {
                var land = BambooTrussMotion.LandStep(p, id, null);
                if (!land.Ok || !land.Walked) { log($"  {id} land: rejected — {land.Note}"); return null; }
                if (!p.BuildMember(m)) { log($"  {id} land: build after landing rejected — {p.Error}"); return null; }
                return new Choice(id, mode, null, false, true, $"{id}: {land.Note}");
            }
            default:
            {
                var park = BambooTrussLayout.Def(relay!)!;
                var toPark = BambooTrussMotion.LandStep(p, relay!, null);
                if (!toPark.Ok || !p.BuildMember(park)) { log($"  {id} relay {relay}: park rejected — {(toPark.Ok ? p.Error : toPark.Note)}"); return null; }
                var toPlace = BambooTrussMotion.LandStep(p, id, null);
                if (!toPlace.Ok || !p.BuildMember(m)) { log($"  {id} relay {relay}: re-grasp/place rejected — {(toPlace.Ok ? p.Error : toPlace.Note)}"); return null; }
                return new Choice(id, mode, relay, toPark.Walked, toPlace.Walked, $"{id}: relay via {relay} — {toPark.Note}; {toPlace.Note}");
            }
        }
    }

    static IEnumerable<BambooIcdMotion.TaskEdit> TasksFor(Choice c)
    {
        var m = BambooTrussLayout.Def(c.MemberId)!;
        var member = BambooTrussMotion.DefaultTasks(new[] { m });
        if (c.Mode == Mode.Direct) return member;
        if (c.Mode == Mode.Land) return new[] { BambooTrussMotion.LandTask(c.MemberId) }.Concat(member);
        var list = new List<BambooIcdMotion.TaskEdit>();
        if (c.WalkToPark) list.Add(BambooTrussMotion.LandTask(c.Relay!));
        list.Add(member[0]);
        list.Add(BambooTrussMotion.ParkTask(c.MemberId, BambooTrussLayout.RelayNode(c.Relay!), c.Relay![(c.Relay!.LastIndexOf('/') + 1)..]));
        if (c.WalkToPlace) list.Add(BambooTrussMotion.LandTask(c.MemberId));
        list.AddRange(BambooTrussMotion.RelayTasks(m, c.Relay!));
        return list;
    }

    static Grid BuildGrid(Options o, IEnumerable<string> relays)
    {
        var g = new Grid();
        var half = BambooTrussLayout.StrutLength / 2 - 0.03;
        foreach (var a in BambooTrussLayout.Stakes)
            foreach (var s in BambooReachMap.AnchorSites(a.Name)) { g.Sites.Add(s); g.Owner.Add(a.Name); }
        foreach (var m in BambooTrussLayout.Members)
            foreach (var s in BambooReachMap.CaneSites("placed-" + m.Id, m.Place, o.CaneOffsets.Select(f => f * half), o.Rolls)) { g.Sites.Add(s); g.Owner.Add(m.Id); }
        var targets = new List<BambooReachMap.TargetLine> { BambooReachMap.Stock() };
        g.Line["stock"] = 0;
        foreach (var m in BambooTrussLayout.Members) { g.Line[m.Id] = targets.Count; targets.Add(BambooReachMap.PlaceOf(m)); }
        foreach (var r in relays.Distinct()) { g.Line[r] = targets.Count; targets.Add(BambooReachMap.PlaceOf(BambooTrussLayout.Def(r)!)); }
        g.R = new BambooReachMap().Matrix(g.Sites, targets);
        return g;
    }

    /// <summary>Where the robot stands now: its clamped (L) base frame as a reach-map site.</summary>
    static BambooReachMap.ClampSite Stand(BambooTrussPlanner p)
    {
        var k = p.Keys[^1];
        return new BambooReachMap.ClampSite(k.BaseOn ?? p.LStake, 0, 0, k.BaseWorld!);
    }

    static double Dist(BambooIcdCell.StrutPlacement m, BambooTrussLayout.Stake s) =>
        Math.Sqrt((m.X - s.X) * (m.X - s.X) + (m.Y - s.ClampY) * (m.Y - s.ClampY) + (m.Z - s.Z) * (m.Z - s.Z));
}
