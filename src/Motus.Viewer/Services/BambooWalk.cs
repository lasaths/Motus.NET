namespace Motus.Viewer.Services;

/// <summary>
/// Walking on the bamboo: a graph search over clamp sites of the L-grip (the resting base) on canes fixed at both ends
/// and on the outboard stakes. Every edge is one Motus-checked step planned by <see cref="BambooTrussPlanner"/>:
/// <list type="bullet">
/// <item><b>brachiate</b> (stake → cane): R clamps a stake / double-fixed cane, base swaps to R at rest, L slides up off
/// its stake and side-clamps the cane (open L-grip), base swaps back to L, R parks.</item>
/// <item><b>inchworm</b> (cane → same cane, new parameter): R clamps the same cane beside L, base swaps to R, L backs off
/// out of its open side and side-clamps the cane further along, base swaps back, R parks.</item>
/// <item><b>transfer</b> (cane → other cane across a node): as inchworm, but R and/or L clamp the other cane, so the
/// robot crosses B (baseL ↔ baseR) or any node shared by double-fixed canes.</item>
/// <item><b>step to stake</b> (cane → stake): R clamps a double-fixed cane, L backs off and slides down onto a stake.</item>
/// </list>
/// One gripper is always clamped on a fixed object (the IK base); joint motion only; swaps happen at rest with their own
/// labels; only stakes outboard of A and canes fixed at both ends are clamped; nothing is carried while walking (both
/// grippers take part in every step).
/// </summary>
public static class BambooWalk
{
    /// <summary>A clamp site of the L-grip: the object, the parameter along it (fraction of half − 3 cm; 0 for a stake)
    /// and the side the arm leaves toward (sign of the L frame's Y·ẑ — the IK branch / roll about the cane).</summary>
    public sealed record Site(string On, double F, int Side)
    {
        public override string ToString() => On.StartsWith("stake", StringComparison.Ordinal) ? On : $"{On}@{F:+0.00;-0.00}{(Side > 0 ? "+z" : "−z")}";
    }

    public sealed record Step(string Kind, Site From, Site To, string RAnchor);

    public sealed class Result
    {
        public List<Site> Reachable { get; } = new();
        public Dictionary<Site, Step> Parent { get; } = new();
        public int Tried { get; set; }
        public int Planned { get; set; }
        public List<Step> PathTo(Site s)
        {
            var path = new List<Step>();
            while (Parent.TryGetValue(s, out var st)) { path.Add(st); s = st.From; }
            path.Reverse();
            return path;
        }
    }

    public static double[] Offsets { get; set; } = { -1.0, -0.5, 0.0, 0.5, 1.0 };
    public static int LSolutions { get; set; } = 2;

    /// <summary>Site of the planner's current L base (L must be the base).</summary>
    public static Site Current(BambooTrussPlanner p, double f = 0)
    {
        var on = p.LStake;
        if (on.StartsWith("stake", StringComparison.Ordinal)) return new Site(on, 0, 0);
        var w = p.BaseWorldNow;
        return new Site(on, f, w[9] >= 0 ? 1 : -1);
    }

    static Site SiteAfter(BambooTrussPlanner p, string on, double f) =>
        on.StartsWith("stake", StringComparison.Ordinal) ? new Site(on, 0, 0) : new Site(on, f, p.BaseWorldNow[9] >= 0 ? 1 : -1);

    /// <summary>
    /// Breadth-first search from the planner's current state. Each discovered site keeps a deep checkpoint of the plan
    /// that reached it. Stops when <paramref name="goal"/> is reached (the planner is left at the goal) or the graph is
    /// exhausted / <paramref name="maxSteps"/> step attempts were made (the planner is restored to the start).
    /// <paramref name="rAnchorStake"/> is the stake R may clamp when L is on a stake (e.g. "stake-S3@0.13").
    /// </summary>
    public static Result Explore(BambooTrussPlanner p, string rAnchorStake, Func<Site, bool>? goal = null,
        int maxSteps = 400, IReadOnlyList<string>? stakes = null)
    {
        var res = new Result();
        var canes = BambooTrussLayout.FixedBothEnds(p.PlacedIds).ToList();
        var start = Current(p);
        var cps = new Dictionary<Site, BambooTrussPlanner.DeepCheckpoint> { [start] = p.SaveDeep() };
        var queue = new Queue<Site>();
        queue.Enqueue(start);
        res.Reachable.Add(start);
        var n = 0;
        while (queue.Count > 0 && res.Tried < maxSteps)
        {
            var from = queue.Dequeue();
            var onStake = from.On.StartsWith("stake", StringComparison.Ordinal);
            var moves = new List<(string kind, string rAnchor, string to, double f, int lSol)>();
            foreach (var cane in canes)
                foreach (var f in Offsets)
                    for (var l = 0; l < LSolutions; l++)
                    {
                        if (onStake) moves.Add(("brachiate", rAnchorStake, cane, f, l));
                        else
                            foreach (var r in canes)
                                moves.Add(("placed-" + cane == from.On ? "inchworm" : "transfer", "placed-" + r, cane, f, l));
                    }
            if (!onStake)
                foreach (var st in stakes ?? Array.Empty<string>())
                    foreach (var r in canes) moves.Add(("step to stake", "placed-" + r, st, 0, 0));
            foreach (var mv in moves)
            {
                if (res.Tried >= maxSteps) break;
                // Skip moves whose target site (any side) is already known: BFS keeps the first (shortest) path.
                if (res.Reachable.Any(s => s.On == (mv.to.StartsWith("stake") ? mv.to : "placed-" + mv.to) && Math.Abs(s.F - mv.f) < 1e-9)) continue;
                p.RollbackDeep(cps[from]);
                res.Tried++;
                var cycle = $"walk-{++n}";
                var ok = mv.kind switch
                {
                    "brachiate" => p.Brachiate(mv.rAnchor, mv.to, mv.f, mv.lSol, cycle),
                    "step to stake" => p.StepToStake(mv.rAnchor, mv.to, cycle),
                    _ => p.HandOverHand(mv.rAnchor, mv.to, mv.f, mv.lSol, cycle)
                };
                if (!ok) continue;
                res.Planned++;
                var to = SiteAfter(p, mv.to.StartsWith("stake") ? mv.to : "placed-" + mv.to, mv.f);
                if (cps.ContainsKey(to)) continue;
                cps[to] = p.SaveDeep();
                res.Parent[to] = new Step(mv.kind, from, to, mv.rAnchor);
                res.Reachable.Add(to);
                queue.Enqueue(to);
                if (goal?.Invoke(to) == true) return res; // planner left at the goal
            }
        }
        p.RollbackDeep(cps[start]);
        return res;
    }
}
