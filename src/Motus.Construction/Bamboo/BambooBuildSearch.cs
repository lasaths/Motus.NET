using Motus.Core;

namespace Motus.Construction;

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

        /// <summary>
        /// Write the choices made so far (with the layout fingerprint) to this file after every confirmed choice and at
        /// the end, so an interrupted search can resume (<see cref="LoadCheckpoint"/> → <see cref="ResumeAt"/>).
        /// </summary>
        public string? CheckpointPath { get; init; }
        /// <summary>
        /// Choices of an earlier (possibly interrupted) run: at each depth the recorded choice is tried first, so the
        /// recorded prefix is re-confirmed without re-trying the alternatives that were rejected before it.
        /// </summary>
        public IReadOnlyList<Choice>? ResumeAt { get; init; }
        /// <summary>
        /// Physics-aware sequencing (null = off, the default): drop frontier members whose partial structure fails
        /// <see cref="BambooFrameStability.Check"/> under self-weight, demote candidates by their sag
        /// (<see cref="BambooFrameStability.Settings.RankPerMetre"/>), and reject a confirmed choice when a member the
        /// robot clamps on during it fails under the robot load.
        /// </summary>
        public BambooFrameStability.Settings? Stability { get; init; }
        /// <summary>
        /// Task-and-motion search aids (null = off, the default): lazy reach preconditions, failure memory, an outcome
        /// log and a confirm-rate prior from earlier logs. Every kept choice is still confirmed by the planner.
        /// </summary>
        public TampSettings? Tamp { get; init; }
    }

    /// <summary>Settings for <see cref="Options.Tamp"/>.</summary>
    public sealed record TampSettings
    {
        /// <summary>Check a candidate's reach precondition (clamp site, landing, relay) only when it is next to be confirmed, memoised; same choices as the eager check.</summary>
        public bool LazyPreconditions { get; init; } = true;
        /// <summary>Skip a candidate the planner already rejected in the same build state (placed set + stand). Heuristic: the planner's random stream may differ.</summary>
        public bool FailureMemory { get; init; } = true;
        /// <summary>Skip a build state (placed set + stand) already found to be a dead end. Heuristic for the same reason.</summary>
        public bool DeadEndMemory { get; init; } = true;
        /// <summary>Append one JSON line per planner confirmation (candidate, build state, rank, outcome, seconds) to this file.</summary>
        public string? OutcomeLogPath { get; init; }
        /// <summary>Earlier outcome logs; candidates are ordered with their smoothed confirm rate (null = no prior).</summary>
        public IReadOnlyList<string>? PriorLogPaths { get; init; }
        /// <summary>Rank added per unit of −ln p̂ (p̂ = smoothed confirm rate of the member and mode); 0 ignores the logs.</summary>
        public double PriorWeight { get; init; } = 1.0;

        internal TampSettings Validate()
        {
            if (!double.IsFinite(PriorWeight) || PriorWeight < 0) throw new ArgumentOutOfRangeException(nameof(PriorWeight));
            return this;
        }
    }

    /// <summary>A saved search state: the layout it belongs to and the confirmed choices in order.</summary>
    public sealed record Checkpoint(string Fingerprint, List<Choice> Order, bool Complete);

    /// <summary>Choices saved by <see cref="Options.CheckpointPath"/>, or null when missing, unreadable or for another layout.</summary>
    public static List<Choice>? LoadCheckpoint(string path)
    {
        try
        {
            if (!File.Exists(path)) return null;
            var c = System.Text.Json.JsonSerializer.Deserialize<Checkpoint>(File.ReadAllText(path));
            return c is not null && c.Fingerprint == BambooTrussMotion.LayoutFingerprint ? c.Order : null;
        }
        catch (Exception e) when (e is System.Text.Json.JsonException or IOException) { return null; }
    }

    static void SaveCheckpoint(string? path, List<Choice> order, bool complete)
    {
        if (path is null) return;
        var tmp = path + ".tmp";
        File.WriteAllText(tmp, System.Text.Json.JsonSerializer.Serialize(new Checkpoint(BambooTrussMotion.LayoutFingerprint, order.ToList(), complete)));
        File.Move(tmp, path, overwrite: true);
    }

    sealed class Grid
    {
        public List<BambooReachMap.ClampSite> Sites = new();
        public List<string> Owner = new();          // anchor column name or member id
        public Dictionary<string, int> Line = new(); // "stock", member id, relay id
        public List<BambooReachMap.TargetLine> Targets = new();
        public BambooReachMap.Reach[,] R = new BambooReachMap.Reach[0, 0];
        public bool?[,]? Lazy;                       // memo when cells are queried on demand
        public bool Ok(int s, string line)
        {
            if (!Line.TryGetValue(line, out var l)) return false;
            if (Lazy is null) return R[s, l].Ik;
            return Lazy[s, l] ??= new BambooReachMap().Query(Sites[s], Targets[l]).Ik;
        }
    }

    public static Result Run(BambooIcdCell cell, Options? options = null, Action<string>? log = null)
    {
        ArgumentNullException.ThrowIfNull(cell);
        var o = (options ?? new Options()).Validate();
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var graph = BambooStructureGraph.FromLayout();
        var lines = new List<string>();
        void Log(string s) { lines.Add(s); log?.Invoke(s); }

        var stability = o.Stability?.Validate();
        var tamp = o.Tamp?.Validate();
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

        var grid = BuildGrid(o, relayOptions.Values.SelectMany(x => x), tamp?.LazyPreconditions == true);
        Log($"reach grid: {grid.Sites.Count} sites × {grid.Line.Count} lines{(grid.Lazy is null ? "" : " (queried on demand)")}");
        var layoutId = BambooTampMemory.LayoutId();
        var prior = tamp?.PriorLogPaths is { } logs && tamp.PriorWeight > 0 ? BambooTampMemory.Prior(BambooTampMemory.Read(logs), layoutId) : null;
        if (prior is not null) Log("tamp: candidates ordered with the confirm-rate prior");
        var failed = new HashSet<string>(StringComparer.Ordinal);
        var deadEnds = new HashSet<string>(StringComparer.Ordinal);

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
            var state = tamp is null ? "" : StateKey(placed, stand);
            if (tamp?.DeadEndMemory == true && deadEnds.Contains(state)) { Log("  tamp: build state already found to be a dead end — skipped"); return false; }
            // Each candidate carries its reach precondition; eager (default) evaluates it here, lazy when it is next.
            var options = new List<(string Id, Mode Mode, string? Relay, double Rank, Func<bool> Pre)>();
            var map = new BambooReachMap { IkSeeds = 12 };
            foreach (var id in graph.Frontier(placed))
            {
                var m = BambooTrussLayout.Def(id)!;
                var usable = Enumerable.Range(0, grid.Sites.Count)
                    .Where(s => grid.Owner[s] != id && (BambooTrussLayout.IsSupport(grid.Owner[s]) || fixedIds.Contains(grid.Owner[s]))).ToList();
                // Far members first: what is built later must not block reaching what is built earlier.
                var far = Dist(m.Place, start);
                options.Add((id, Mode.Direct, null, -far,
                    () => map.Query(stand, BambooReachMap.Stock()).Ik && map.Query(stand, BambooReachMap.PlaceOf(m)).Ik));
                options.Add((id, Mode.Land, null, 10 - far, () => usable.Any(s => grid.Ok(s, "stock") && grid.Ok(s, id))));
                foreach (var r in relayOptions.GetValueOrDefault(id) ?? new List<string>())
                {
                    if (!graph.ExistingNodes(placed).Contains(BambooTrussLayout.RelayNode(r))) continue;
                    // Stock → park from where the robot stands or one landing; park → place from one landing.
                    options.Add((id, Mode.Relay, r, 20 - far, () =>
                        ((map.Query(stand, BambooReachMap.Stock()).Ik && map.Query(stand, BambooReachMap.PlaceOf(BambooTrussLayout.Def(r)!)).Ik)
                            || usable.Any(s => grid.Ok(s, "stock") && grid.Ok(s, r)))
                        && usable.Any(s => grid.Ok(s, r) && grid.Ok(s, id))));
                }
            }
            if (tamp?.LazyPreconditions != true)
                options = options.Where(x => x.Pre()).Select(x => x with { Pre = () => true }).ToList();
            else
                options = options.Select(x => { bool? memo = null; var pre = x.Pre; return x with { Pre = () => memo ??= pre() }; }).ToList();
            if (prior is not null)
                options = options.Select(x => x with { Rank = x.Rank + tamp!.PriorWeight * prior(x.Id, x.Mode) }).ToList();
            if (stability is not null)
            {
                var sag = new Dictionary<string, BambooFrameStability.Report>(StringComparer.Ordinal);
                foreach (var id in options.Select(x => x.Id).Distinct())
                {
                    var r = sag[id] = BambooFrameStability.Check(graph, placed.Append(id), stability);
                    if (!r.Ok) Log($"  {id}: dropped by the stability check — {r.Note}");
                }
                options = options.Where(x => sag[x.Id].Ok)
                    .Select(x => x with { Rank = x.Rank + stability.RankPerMetre * sag[x.Id].MaxDeflectionM }).ToList();
            }
            var resume = o.ResumeAt is { } ra && placed.Count < ra.Count ? ra[placed.Count] : null;
            foreach (var (id, mode, relay, rank, pre) in options.OrderBy(x => resume is not null && x.Id == resume.MemberId && x.Mode == resume.Mode && x.Relay == resume.Relay ? 0 : 1).ThenBy(x => x.Rank))
            {
                if (BudgetReached()) break;
                if (!pre()) continue;
                var tried = $"{state}#{id}|{mode}|{relay}";
                if (tamp?.FailureMemory == true && failed.Contains(tried)) { Log($"  {id} {mode}: already rejected in this build state — skipped"); continue; }
                var cp = p.SaveDeep();
                var firstKey = p.Keys.Count;
                var t0 = clock.Elapsed.TotalSeconds;
                var choice = Confirm(p, id, mode, relay, Log);
                if (choice is not null && stability is not null && RobotLoadFailure(graph, placed, id, p, firstKey, stability) is { } why)
                { Log($"  {id} {mode}: rejected by the stability check — {why}"); choice = null; }
                if (tamp?.OutcomeLogPath is { } outPath)
                    BambooTampMemory.Append(outPath, new BambooTampMemory.Outcome(layoutId, id, mode.ToString(), relay, placed.Count,
                        string.Join(",", placed.Order(StringComparer.Ordinal)), rank, choice is not null, clock.Elapsed.TotalSeconds - t0));
                if (choice is null) { rejected++; failed.Add(tried); p.RollbackDeep(cp); continue; }
                confirmed++;
                placed.Add(id); order.Add(choice);
                var nt = tasks.Count;
                tasks.AddRange(TasksFor(choice));
                if (order.Count > bestOrder.Count) { bestOrder = order.ToList(); bestTasks = tasks.ToList(); }
                Log($"{placed.Count}/{graph.Edges.Count} {choice.Note}");
                SaveCheckpoint(o.CheckpointPath, order, placed.Count == graph.Edges.Count);
                if (Dfs()) return true;
                Log($"backtrack: {id} ({mode}) leads to a dead end");
                placed.RemoveAt(placed.Count - 1); order.RemoveAt(order.Count - 1);
                tasks.RemoveRange(nt, tasks.Count - nt);
                p.RollbackDeep(cp);
            }
            if (tamp is not null && !BudgetReached()) deadEnds.Add(state);
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

    /// <summary>
    /// Robot load on every structure member the robot clamps on in keys [firstKey..]: checked on the structure before
    /// the choice (the member being built is not there yet), or with it when the clamp is on that member. Null = ok.
    /// </summary>
    static string? RobotLoadFailure(BambooStructureGraph graph, List<string> placed, string id, BambooTrussPlanner p, int firstKey,
        BambooFrameStability.Settings s)
    {
        var payload = s.MassPerMetre * (BambooTrussLayout.Def(id)?.Length ?? 0);
        var members = graph.Edges.Select(e => e.Id).ToHashSet(StringComparer.Ordinal);
        foreach (var on in p.Keys.Skip(firstKey).Select(k => k.BaseOn?.StartsWith("placed-", StringComparison.Ordinal) == true ? k.BaseOn["placed-".Length..] : k.BaseOn)
                     .Where(b => b is not null && members.Contains(b)).Distinct())
        {
            var structure = placed.Contains(on!) ? placed : placed.Append(id).ToList();
            var r = BambooFrameStability.Check(graph, structure, s, on, payload);
            if (!r.Ok) return r.Note;
        }
        return null;
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

    static Grid BuildGrid(Options o, IEnumerable<string> relays, bool lazy)
    {
        var g = new Grid();
        foreach (var a in BambooTrussLayout.Stakes)
            foreach (var s in BambooReachMap.AnchorSites(a.Name)) { g.Sites.Add(s); g.Owner.Add(a.Name); }
        foreach (var m in BambooTrussLayout.Members)
            foreach (var s in BambooReachMap.CaneSites("placed-" + m.Id, m.Place, o.CaneOffsets.Select(f => f * (m.Length / 2 - 0.03)), o.Rolls)) { g.Sites.Add(s); g.Owner.Add(m.Id); }
        var targets = new List<BambooReachMap.TargetLine> { BambooReachMap.Stock() };
        g.Line["stock"] = 0;
        foreach (var m in BambooTrussLayout.Members) { g.Line[m.Id] = targets.Count; targets.Add(BambooReachMap.PlaceOf(m)); }
        foreach (var r in relays.Distinct()) { g.Line[r] = targets.Count; targets.Add(BambooReachMap.PlaceOf(BambooTrussLayout.Def(r)!)); }
        g.Targets = targets;
        if (lazy) g.Lazy = new bool?[g.Sites.Count, targets.Count];
        else g.R = new BambooReachMap().Matrix(g.Sites, targets);
        return g;
    }

    /// <summary>Build state for failure memory: the placed set (order-free) and where the robot stands.</summary>
    static string StateKey(List<string> placed, BambooReachMap.ClampSite stand) =>
        string.Join(",", placed.Order(StringComparer.Ordinal)) + "@" + stand.On + ":" + string.Join(",", stand.BaseWorld.Select(BambooTrussMotion.FingerprintNumber));

    /// <summary>Where the robot stands now: its clamped (L) base frame as a reach-map site.</summary>
    static BambooReachMap.ClampSite Stand(BambooTrussPlanner p)
    {
        var k = p.Keys[^1];
        return new BambooReachMap.ClampSite(k.BaseOn ?? p.LStake, 0, 0, k.BaseWorld!);
    }

    static double Dist(BambooIcdCell.StrutPlacement m, BambooTrussLayout.Stake s) =>
        Math.Sqrt((m.X - s.X) * (m.X - s.X) + (m.Y - s.ClampY) * (m.Y - s.ClampY) + (m.Z - s.Z) * (m.Z - s.Z));
}
