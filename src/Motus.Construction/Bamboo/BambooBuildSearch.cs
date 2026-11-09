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
    /// <summary>How a member is built: from where the robot stands, after a targeted landing, via a relay park, or after an
    /// anchor-to-anchor walk (<see cref="Options.AnchorWalks"/>).
    /// <see cref="Mode.Reposition"/> is a walk-only move (<see cref="Options.Reposition"/>); it never appears in
    /// <see cref="Result.Order"/> — it is recorded on the next choice (<see cref="Choice.RepositionTo"/>).</summary>
    public enum Mode { Direct, Land, Relay, Walk, Reposition }

    /// <summary>One confirmed choice. <paramref name="Relay"/>: the relay id (<see cref="Mode.Relay"/>) or the anchor walked to (<see cref="Mode.Walk"/>).</summary>
    public sealed record Choice(string MemberId, Mode Mode, string? Relay, bool WalkToPark, bool WalkToPlace, string Note)
    {
        /// <summary>Walk-only reposition made just before this choice (<see cref="Options.Reposition"/>): the foothold
        /// (anchor name or member id) L walked to, and the clamp parameter on it (null = none).</summary>
        public string? RepositionTo { get; init; }
        public double RepositionF { get; init; }
    }

    public sealed record Result(List<BambooIcdMotion.TaskEdit> Tasks, List<Choice> Order, List<string> Log, bool Complete,
        int Confirmed, int Rejected, double Seconds)
    {
        public int Attempts => Confirmed + Rejected;
        public string StopReason { get; init; } = "";
        /// <summary>Confirmations whose short RRT stage ran out (<see cref="Options.StagedRrtIters"/>): deferred, not rejected.</summary>
        public int Deferred { get; init; }
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

        /// <summary>
        /// Staged confirmation (off by default; it changes the order candidates are kept in). Each candidate is first
        /// confirmed with one RRT-Connect attempt of <see cref="StagedRrtIters"/> iterations; the IK, end-pose,
        /// direct-segment and fold-via checks run first, as always.
        /// <list type="bullet">
        /// <item>Short stage succeeds and no transit ran out: kept. The motion is the one the full budget plans (same
        /// random stream, same first attempt), so the replay plans it too.</item>
        /// <item>Short stage succeeds only after a transit ran out (a planner fallback): re-confirmed at the full budget
        /// before it is kept, so the kept motion is the one the replay plans.</item>
        /// <item>Short stage fails and no transit ran out: rejected (the full budget fails the same way).</item>
        /// <item>Short stage fails after a transit ran out: "not yet", not rejected — RRT cannot prove there is no path.
        /// It is retried once at the full budget after the other candidates of the same build state.</item>
        /// </list>
        /// A resumed choice (<see cref="ResumeAt"/>) is confirmed at the full budget. Collision checking is unchanged.
        /// </summary>
        public bool StagedConfirm { get; init; }
        /// <summary>RRT-Connect iterations of the short stage (<see cref="StagedConfirm"/>); one attempt.</summary>
        public int StagedRrtIters { get; init; } = 150;
        /// <summary>
        /// Walk-only reposition (off by default): when the robot cannot reach a frontier member from where it stands,
        /// also try walking L to another foothold (an anchor, a fixed member, or any placed member with
        /// <see cref="PlacedFootholds"/>) without placing a cane, then build from there. Targets are prefiltered with
        /// the reach map and ranked by the route length over the foothold graph; at most one reposition between two
        /// placements.
        /// </summary>
        public bool Reposition { get; init; }
        /// <summary>Reposition targets tried per build state (nearest route first).</summary>
        public int MaxRepositions { get; init; } = 3;

        internal Options Validate()
        {
            if (StagedRrtIters < 1) throw new ArgumentOutOfRangeException(nameof(StagedRrtIters));
            if (MaxRepositions < 0) throw new ArgumentOutOfRangeException(nameof(MaxRepositions));
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
        /// <summary>
        /// Also try walking L from the anchor it stands on to another anchor (<see cref="BambooTrussPlanner.WalkToAnchor"/>)
        /// and building from there, for anchors whose reach-map sites reach the stock and the member. Off by default.
        /// </summary>
        public bool AnchorWalks { get; init; }
        /// <summary>
        /// Rank candidates nearest the robot first (place + stock distance from the stand) instead of farthest from the
        /// start support first. Off by default; helps structures the robot must walk across (grid shells, many stock spots).
        /// </summary>
        public bool NearestFirst { get; init; }
        /// <summary>
        /// Let the grippers land and walk on every placed member, not only on members the rigidity test calls fixed
        /// (<see cref="BambooStructureGraph.Fixed"/>); the planner runs with <see cref="MotionConstraints.FixedMemberRule"/>
        /// off. Off by default. Approximate: with pin joints a placed ring can still be a first-order mechanism (e.g. the
        /// interior of a diagrid grid shell), and this option treats it as a rigid foothold. Collision checking is unchanged.
        /// Replay the result with the same rule (<see cref="AssemblyPlanner"/> does this when the option is on).
        /// </summary>
        public bool PlacedFootholds { get; init; }
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

    internal sealed class Grid
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

        if (o.PlacedFootholds) Log("placed footholds: every placed member may be clamped (approximate for pin-joint mechanisms)");
        var p = new BambooTrussPlanner(cell, o.PlacedFootholds ? MotionConstraints.Current with { FixedMemberRule = false } : null);
        if (!p.Start()) { Log("planner start failed: " + p.Error); return new Result(new(), new(), lines, false, 0, 0, clock.Elapsed.TotalSeconds) { StopReason = "planner start failed: " + p.Error }; }
        var start = BambooTrussLayout.StakeNamed(BambooTrussLayout.Current.StartStake);
        var placed = new List<string>();
        var order = new List<Choice>();
        var tasks = new List<BambooIcdMotion.TaskEdit>();
        var bestOrder = new List<Choice>();
        var bestTasks = new List<BambooIcdMotion.TaskEdit>();
        int confirmed = 0, rejected = 0, deferred = 0;
        var gap = new Dictionary<(string, string), double>();
        string stopReason = "search exhausted";

        bool BudgetReached()
        {
            if (rejected >= o.MaxRejected) { stopReason = $"rejected confirmation limit ({o.MaxRejected})"; return true; }
            if (confirmed + rejected >= o.MaxConfirmations) { stopReason = $"confirmation limit ({o.MaxConfirmations})"; return true; }
            return false;
        }

        bool Dfs((string To, double F)? pending = null)
        {
            if (placed.Count == graph.Edges.Count) return true;
            if (BudgetReached()) return false;

            var fixedIds = Footholds(graph, placed, o);
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
                // Far members first: what is built later must not block reaching what is built earlier. NearestFirst: members
                // whose place and stock lie nearest the stand first, so the robot finishes an area before it moves on.
                var far = o.NearestFirst ? -(Dist(stand.Origin, (m.Place.X, m.Place.Y, m.Place.Z)) + Dist(stand.Origin, (m.Store.X, m.Store.Y, m.Store.Z)))
                    : Dist(m.Place, start);
                var stock = BambooReachMap.Stock(m);
                options.Add((id, Mode.Direct, null, -far,
                    () => map.Query(stand, stock).Ik && map.Query(stand, BambooReachMap.PlaceOf(m)).Ik));
                // A landing puts L on a fixed member one node-transfer step away (BambooTrussPlanner.LandingCandidates):
                // only member sites within two arm reaches of the stand can be landed on.
                var landable = usable.Where(s => !BambooTrussLayout.IsSupport(grid.Owner[s]) && Near(stand, grid.Sites[s])).ToList();
                options.Add((id, Mode.Land, null, 10 - far, () => landable.Any(s => grid.Ok(s, stock.Name) && grid.Ok(s, id))));
                if (o.AnchorWalks && BambooTrussLayout.IsSupport(p.LStake))
                    foreach (var a in BambooTrussLayout.Stakes.Where(a => a.Name != p.LStake))
                        options.Add((id, Mode.Walk, a.Name, 15 - far + Dist(m.Place, a),
                            // The walk is at most two steps (R on a via point, then L on the anchor): within two arm reaches.
                            () => Enumerable.Range(0, grid.Sites.Count).Any(s => grid.Owner[s] == a.Name && Near(stand, grid.Sites[s]) && grid.Ok(s, stock.Name) && grid.Ok(s, id))));
                foreach (var r in relayOptions.GetValueOrDefault(id) ?? new List<string>())
                {
                    if (!graph.ExistingNodes(placed).Contains(BambooTrussLayout.RelayNode(r))) continue;
                    // Stock → park from where the robot stands or one landing; park → place from one landing.
                    options.Add((id, Mode.Relay, r, 20 - far, () =>
                        ((map.Query(stand, stock).Ik && map.Query(stand, BambooReachMap.PlaceOf(BambooTrussLayout.Def(r)!)).Ik)
                            || usable.Any(s => grid.Ok(s, stock.Name) && grid.Ok(s, r)))
                        && usable.Any(s => grid.Ok(s, r) && grid.Ok(s, id))));
                }
            }
            if (o.Reposition && pending is null && o.MaxRepositions > 0)
                foreach (var (to, f, travel) in RepositionTargets(graph, placed, fixedIds, stand, grid, map, gap).Take(o.MaxRepositions))
                    options.Add((to, Mode.Reposition, f.ToString("R", System.Globalization.CultureInfo.InvariantCulture), 30 + travel, () => true));
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
            bool Resumed(Mode mode, string id, string? relay) => resume is not null && (resume.RepositionTo is { } rt && pending is null
                ? mode == Mode.Reposition && id == rt
                : id == resume.MemberId && mode == resume.Mode && relay == resume.Relay);
            var staged = o.StagedConfirm && o.StagedRrtIters < p.RrtIters;
            var notYet = new List<(string Id, Mode Mode, string? Relay, double Rank)>();

            // One confirmation; true when the search completes below it. shortStage: one RRT attempt of
            // StagedRrtIters; if any transit ran out the choice is rolled back and kept as "not yet".
            bool Try(string id, Mode mode, string? relay, double rank, bool shortStage)
            {
                var tried = $"{state}#{id}|{mode}|{relay}";
                var cp = p.SaveDeep();
                var firstKey = p.Keys.Count;
                var t0 = clock.Elapsed.TotalSeconds;
                var (a0, i0, miss0) = (p.RrtAttempts, p.RrtIters, p.StagedMisses);
                if (shortStage) { p.RrtAttempts = 1; p.RrtIters = o.StagedRrtIters; p.StagedIters = o.StagedRrtIters; }
                Choice? choice;
                try
                {
                    choice = mode == Mode.Reposition
                        ? Reposition(p, id, double.Parse(relay!, System.Globalization.CultureInfo.InvariantCulture), Log)
                        : Confirm(p, id, mode, relay, Log);
                }
                finally { p.RrtAttempts = a0; p.RrtIters = i0; p.StagedIters = 0; }
                if (shortStage && p.StagedMisses > miss0)
                {
                    p.RollbackDeep(cp);
                    if (choice is not null)
                    {
                        Log($"  {id} {mode}: short stage passed after a planner fallback — re-confirmed at the full budget");
                        return Try(id, mode, relay, rank, false);
                    }
                    deferred++;
                    notYet.Add((id, mode, relay, rank));
                    Log($"  {id} {mode}: not yet — the short RRT stage ({o.StagedRrtIters} iterations) ran out after {clock.Elapsed.TotalSeconds - t0:F1} s; retried at the full budget if this state needs it");
                    return false;
                }
                if (choice is not null && stability is not null
                    && RobotLoadFailure(graph, placed, mode == Mode.Reposition ? null : id, p, firstKey, stability) is { } why)
                { Log($"  {id} {mode}: rejected by the stability check — {why}"); choice = null; }
                if (tamp?.OutcomeLogPath is { } outPath)
                    BambooTampMemory.Append(outPath, new BambooTampMemory.Outcome(layoutId, id, mode.ToString(), relay, placed.Count,
                        string.Join(",", placed.Order(StringComparer.Ordinal)), rank, choice is not null, clock.Elapsed.TotalSeconds - t0));
                if (choice is null) { rejected++; failed.Add(tried); p.RollbackDeep(cp); return false; }
                confirmed++;
                if (mode == Mode.Reposition)
                {
                    Log($"  {choice.Note}");
                    if (Dfs((id, double.Parse(relay!, System.Globalization.CultureInfo.InvariantCulture)))) return true;
                    Log($"backtrack: reposition → {id} leads to a dead end");
                    p.RollbackDeep(cp);
                    return false;
                }
                if (pending is { } pv) choice = choice with { RepositionTo = pv.To, RepositionF = pv.F, Note = $"reposition → {pv.To}; {choice.Note}" };
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
                return false;
            }

            foreach (var (id, mode, relay, rank, pre) in options.OrderBy(x => Resumed(x.Mode, x.Id, x.Relay) ? 0 : 1).ThenBy(x => x.Rank))
            {
                if (BudgetReached()) break;
                if (!pre()) continue;
                if (tamp?.FailureMemory == true && failed.Contains($"{state}#{id}|{mode}|{relay}")) { Log($"  {id} {mode}: already rejected in this build state — skipped"); continue; }
                if (Try(id, mode, relay, rank, staged && !Resumed(mode, id, relay))) return true;
            }
            // "Not yet" candidates: RRT cannot prove there is no path, so each one gets the full budget once.
            foreach (var (id, mode, relay, rank) in notYet.ToList())
            {
                if (BudgetReached()) break;
                Log($"  {id} {mode}: retry at the full RRT budget");
                if (Try(id, mode, relay, rank, false)) return true;
            }
            if (tamp is not null && !BudgetReached()) deadEnds.Add(state);
            return false;
        }

        var complete = Dfs();
        Log(complete ? $"build search: {string.Join(" → ", order.Select(c => c.MemberId + (c.Mode == Mode.Direct ? "" : c.Mode == Mode.Land ? " (land)" : $" (relay {c.Relay})")))} — {confirmed} confirmed, {rejected} rejected, {deferred} deferred, {clock.Elapsed.TotalSeconds:F0} s"
            : $"build search: incomplete — best confirmed prefix {bestOrder.Count}/{graph.Edges.Count} placed, {confirmed} confirmed, {rejected} rejected, {deferred} deferred; {stopReason}");
        cell.SetMountViewer(BambooTrussLayout.StartBase);
        return new Result(complete ? tasks : bestTasks, complete ? order : bestOrder, lines, complete, confirmed, rejected, clock.Elapsed.TotalSeconds)
        { StopReason = complete ? "complete" : stopReason, Deferred = deferred };
    }

    /// <summary>Placed members whose reach-map sites the search may land on (<see cref="Options.PlacedFootholds"/>).</summary>
    internal static HashSet<string> Footholds(BambooStructureGraph graph, IReadOnlyList<string> placed, Options o) =>
        (o.PlacedFootholds ? placed : graph.Fixed(placed)).ToHashSet(StringComparer.Ordinal);

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
            case Mode.Walk:
                if (!p.WalkToAnchor(relay!)) { log($"  {id} walk → {relay}: rejected — {p.Error}"); return null; }
                if (!p.BuildMember(m)) { log($"  {id} walk → {relay}: build after the walk rejected — {p.Error}"); return null; }
                return new Choice(id, mode, relay, false, false, $"{id}: walk {relay} then direct");
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
    /// Walk-only reposition: L walks to <paramref name="to"/> (anchor name or member id) at clamp parameter
    /// <paramref name="f"/> and nothing is placed — the same calls <see cref="BambooTrussMotion.RunSequence"/> makes for a
    /// walk block (anchor → anchor stake walk, else <see cref="BambooWalk.Explore"/> with the default gait).
    /// </summary>
    static Choice? Reposition(BambooTrussPlanner p, string to, double f, Action<string> log)
    {
        var target = BambooTrussLayout.IsSupport(to) ? to : "placed-" + to;
        if (BambooTrussLayout.IsSupport(to) && BambooTrussLayout.IsSupport(p.LStake))
        {
            if (p.WalkToAnchor(to)) return new Choice(to, Mode.Reposition, null, false, false, $"reposition → {to}: stake walk");
            log($"  reposition → {to}: rejected — {p.Error}"); return null;
        }
        var gait = BambooWalk.GaitNamed(null);
        var label = BambooTrussLayout.IsSupport(to) ? to : $"{to}@{f:+0.00;-0.00}";
        bool Goal(BambooWalk.Site s) => s.On == target && (s.IsStake || Math.Abs(s.F - f) < 1e-9);
        var res = BambooWalk.Explore(p, gait, BambooTrussLayout.Current.WalkRAnchor, Goal, ReplayWalkSteps, null, cycleName: $"walk ({gait.Id}) → {label}");
        if (res.Reached is { } r) return new Choice(to, Mode.Reposition, null, false, false, $"reposition → {label}: {res.PathTo(r).Count} steps");
        log($"  reposition → {label}: rejected — not reachable ({res.Tried} steps tried)");
        return null;
    }

    /// <summary>Walk steps per walk block, as in <see cref="BambooTrussMotion.RunSequence"/> (the replay must find the same walk).</summary>
    const int ReplayWalkSteps = 150;

    /// <summary>
    /// Reposition targets: footholds (anchors and <paramref name="footholds"/>) from whose reach-map sites a frontier
    /// member that the robot cannot reach from <paramref name="stand"/> can be picked and placed; ranked by the route
    /// length over the foothold graph (two footholds are joined when their sites lie within two arm reaches). The clamp
    /// parameter is the node end nearest that member (default gait: node transfer), 0 for an anchor.
    /// </summary>
    internal static IEnumerable<(string To, double F, double Travel)> RepositionTargets(BambooStructureGraph graph, List<string> placed,
        HashSet<string> footholds, BambooReachMap.ClampSite stand, Grid grid, BambooReachMap map, Dictionary<(string, string), double> gap)
    {
        static string Owner(string on) => on.StartsWith("placed-", StringComparison.Ordinal) ? on["placed-".Length..] : on;
        var here = Owner(stand.On);
        var want = graph.Frontier(placed).Select(id => BambooTrussLayout.Def(id)!)
            .Where(m => !(map.Query(stand, BambooReachMap.Stock(m)).Ik && map.Query(stand, BambooReachMap.PlaceOf(m)).Ik)).ToList();
        if (want.Count == 0) yield break;
        var nodes = BambooTrussLayout.Stakes.Select(a => a.Name).Concat(footholds.Order(StringComparer.Ordinal)).Append(here).Distinct().ToList();
        var sites = nodes.ToDictionary(n => n, n => Enumerable.Range(0, grid.Sites.Count).Where(s => grid.Owner[s] == n).ToList());
        double Gap(string a, string b)
        {
            var key = string.CompareOrdinal(a, b) < 0 ? (a, b) : (b, a);
            if (gap.TryGetValue(key, out var g)) return g;
            g = double.MaxValue;
            foreach (var i in sites[a]) foreach (var j in sites[b]) g = Math.Min(g, Dist(grid.Sites[i].Origin, grid.Sites[j].Origin));
            return gap[key] = g;
        }
        (double X, double Y, double Z) Center(string n) => sites[n].Count == 0 ? stand.Origin
            : (sites[n].Average(s => grid.Sites[s].Origin.X), sites[n].Average(s => grid.Sites[s].Origin.Y), sites[n].Average(s => grid.Sites[s].Origin.Z));
        // Dijkstra from where the robot stands.
        var dist = nodes.ToDictionary(n => n, _ => double.PositiveInfinity);
        dist[here] = 0;
        var open = nodes.ToHashSet();
        while (open.Count > 0)
        {
            var u = open.MinBy(n => dist[n])!;
            if (double.IsPositiveInfinity(dist[u])) break;
            open.Remove(u);
            foreach (var v in open)
                if (Gap(u, v) <= 2 * BambooReachMap.MaxReachM) dist[v] = Math.Min(dist[v], dist[u] + Dist(Center(u), Center(v)));
        }
        var list = new List<(string, double, double)>();
        foreach (var n in nodes)
        {
            if (n == here || double.IsPositiveInfinity(dist[n])) continue;
            var helps = want.Where(m => m.Id != n && sites[n].Any(s => grid.Ok(s, BambooReachMap.Stock(m).Name) && grid.Ok(s, m.Id))).ToList();
            if (helps.Count == 0) continue;
            var f = 0.0;
            if (!BambooTrussLayout.IsSupport(n))
            {
                var m = helps[0].Place;
                f = new[] { -1.0, 1.0 }.MinBy(e => Dist(BambooWalk.Pos(n, e), (m.X, m.Y, m.Z)));
            }
            list.Add((n, f, dist[n]));
        }
        foreach (var x in list.OrderBy(x => x.Item3)) yield return x;
    }

    /// <summary>
    /// Robot load on every structure member the robot clamps on in keys [firstKey..]: checked on the structure before
    /// the choice (the member being built is not there yet), or with it when the clamp is on that member. Null = ok.
    /// </summary>
    static string? RobotLoadFailure(BambooStructureGraph graph, List<string> placed, string? id, BambooTrussPlanner p, int firstKey,
        BambooFrameStability.Settings s)
    {
        var payload = id is null ? 0 : s.MassPerMetre * (BambooTrussLayout.Def(id)?.Length ?? 0);
        var members = graph.Edges.Select(e => e.Id).ToHashSet(StringComparer.Ordinal);
        foreach (var on in p.Keys.Skip(firstKey).Select(k => k.BaseOn?.StartsWith("placed-", StringComparison.Ordinal) == true ? k.BaseOn["placed-".Length..] : k.BaseOn)
                     .Where(b => b is not null && members.Contains(b)).Distinct())
        {
            var structure = id is null || placed.Contains(on!) ? placed : placed.Append(id).ToList();
            var r = BambooFrameStability.Check(graph, structure, s, on, payload);
            if (!r.Ok) return r.Note;
        }
        return null;
    }

    internal static IEnumerable<BambooIcdMotion.TaskEdit> TasksFor(Choice c)
    {
        var m = BambooTrussLayout.Def(c.MemberId)!;
        var walk = c.RepositionTo is { } to
            ? new[] { BambooTrussMotion.WalkTask(BambooTrussLayout.IsSupport(to) ? to : "placed-" + to, c.RepositionF) }
            : Array.Empty<BambooIcdMotion.TaskEdit>();
        return walk.Concat(TasksForMember(c));
    }

    static IEnumerable<BambooIcdMotion.TaskEdit> TasksForMember(Choice c)
    {
        var m = BambooTrussLayout.Def(c.MemberId)!;
        var member = BambooTrussMotion.DefaultTasks(new[] { m });
        if (c.Mode == Mode.Direct) return member;
        if (c.Mode == Mode.Land) return new[] { BambooTrussMotion.LandTask(c.MemberId) }.Concat(member);
        if (c.Mode == Mode.Walk) return new[] { BambooTrussMotion.WalkTask(c.Relay!) }.Concat(member);
        var list = new List<BambooIcdMotion.TaskEdit>();
        if (c.WalkToPark) list.Add(BambooTrussMotion.LandTask(c.Relay!));
        list.Add(member[0]);
        list.Add(BambooTrussMotion.ParkTask(c.MemberId, BambooTrussLayout.RelayNode(c.Relay!), c.Relay![(c.Relay!.LastIndexOf('/') + 1)..]));
        if (c.WalkToPlace) list.Add(BambooTrussMotion.LandTask(c.MemberId));
        list.AddRange(BambooTrussMotion.RelayTasks(m, c.Relay!));
        return list;
    }

    internal static Grid BuildGrid(Options o, IEnumerable<string> relays, bool lazy)
    {
        var g = new Grid();
        foreach (var a in BambooTrussLayout.Stakes)
            foreach (var s in BambooReachMap.AnchorSites(a.Name)) { g.Sites.Add(s); g.Owner.Add(a.Name); }
        foreach (var m in BambooTrussLayout.Members)
            foreach (var s in BambooReachMap.CaneSites("placed-" + m.Id, m.Place, o.CaneOffsets.Select(f => f * (m.Length / 2 - 0.03)), o.Rolls)) { g.Sites.Add(s); g.Owner.Add(m.Id); }
        var targets = new List<BambooReachMap.TargetLine>();
        foreach (var k in BambooTrussLayout.FeedSpotsInUse.Prepend(0).Distinct()) { var st = BambooReachMap.Stock(k); g.Line[st.Name] = targets.Count; targets.Add(st); }
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

    /// <summary>Clamp site <paramref name="b"/> lies within two arm reaches of <paramref name="a"/> (sound IK prefilter for one step).</summary>
    static bool Near(BambooReachMap.ClampSite a, BambooReachMap.ClampSite b)
    {
        var (p, q) = (a.Origin, b.Origin);
        return Math.Sqrt((p.X - q.X) * (p.X - q.X) + (p.Y - q.Y) * (p.Y - q.Y) + (p.Z - q.Z) * (p.Z - q.Z)) <= 2 * BambooReachMap.MaxReachM;
    }

    static double Dist((double X, double Y, double Z) a, (double X, double Y, double Z) b) =>
        Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));

    static double Dist(BambooIcdCell.StrutPlacement m, BambooTrussLayout.Stake s) =>
        Math.Sqrt((m.X - s.X) * (m.X - s.X) + (m.Y - s.ClampY) * (m.Y - s.ClampY) + (m.Z - s.Z) * (m.Z - s.Z));
}
