namespace Motus.Viewer.Services;

/// <summary>
/// Walking on the bamboo — the planner behind the /bamboo-truss <c>walk</c> task block. The L-grip is the resting base;
/// a walk is a breadth-first search over L clamp sites (a double-fixed cane + a parameter along it, or a stake) where
/// every edge is one Motus-checked step planned by <see cref="BambooTrussPlanner"/>:
/// R clamps something fixed → <c>swap base L→R</c> at rest → L opens, backs off out of its open side (or slides up off a
/// stake) → L side-clamps the next site (or slides down onto a stake) → <c>swap base R→L</c> at rest → R retreats and
/// parks. One gripper is always clamped on a fixed object (the IK base), joints only, FK-exact clamps, only stakes outboard
/// of A and canes fixed at both ends are clamped, and nothing is carried while walking.
/// A <see cref="Gait"/> decides which steps are allowed (where R clamps, where L may go): five gaits are defined in
/// <see cref="Gaits"/>.
/// </summary>
public static class BambooWalk
{
    /// <summary>L clamp site: the object, the parameter along it (fraction of half − 3 cm from the centre; 0 for a stake)
    /// and the side the arm leaves toward (sign of the L frame's Y·ẑ — the IK branch / roll about the cane).</summary>
    public sealed record Site(string On, double F, int Side)
    {
        public bool IsStake => BambooTrussLayout.IsSupport(On);
        public override string ToString() => IsStake ? On : $"{On}@{F:+0.00;-0.00}{(Side > 0 ? "+z" : Side < 0 ? "−z" : "")}";
    }

    /// <summary>One candidate step: R clamps <paramref name="RAnchor"/> (at <paramref name="RAtM"/> m from a cane's centre
    /// when pinned), L goes to <paramref name="To"/> at <paramref name="F"/> with IK solution <paramref name="LSol"/>.</summary>
    public sealed record Move(string Kind, string RAnchor, double? RAtM, string To, double F, int LSol);

    public sealed record Step(string Kind, Site From, Site To, string RAnchor, double StrideM);

    /// <summary>A walking behaviour: the clamp-site grid and the steps it allows from a site.</summary>
    public sealed record Gait(string Id, string Name, string Plain, double[] Offsets, bool TrackSides,
        Func<Ctx, Site, IEnumerable<Move>> Moves);

    /// <summary>What a gait sees when proposing steps.</summary>
    public sealed record Ctx(IReadOnlyList<string> Canes, string RAnchorStake, Func<Site, bool> Goal, double[] Offsets, int LSolutions)
    {
        public IEnumerable<(string Cane, double F)> Sites() => from c in Canes from f in Offsets select (c, f);
    }

    public sealed class Result
    {
        public List<Site> Reachable { get; } = new();
        public Dictionary<Site, Step> Parent { get; } = new();
        public Dictionary<Site, double[]> BaseAt { get; } = new();
        public int Tried { get; set; }
        public int Planned { get; set; }
        public Site? Reached { get; set; }
        public List<Step> PathTo(Site s)
        {
            var path = new List<Step>();
            while (Parent.TryGetValue(s, out var st)) { path.Add(st); s = st.From; }
            path.Reverse();
            return path;
        }
    }

    public static double Half => BambooTrussLayout.StrutLength / 2 - 0.03;

    /// <summary>World position of a clamp site on its (design) cane line.</summary>
    public static (double X, double Y, double Z) Pos(string cane, double f)
    {
        var id = cane.StartsWith("placed-", StringComparison.Ordinal) ? cane["placed-".Length..] : cane;
        var d = BambooTrussLayout.Members.First(m => m.Id == id).Place; var u = d.Axis;
        return (d.X + u.X * f * Half, d.Y + u.Y * f * Half, d.Z + u.Z * f * Half);
    }

    static double Dist((double X, double Y, double Z) a, (double X, double Y, double Z) b) =>
        Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));

    static (double X, double Y, double Z) SitePos(Site s) =>
        s.IsStake ? (BambooTrussLayout.StakeNamed(s.On).X, BambooTrussLayout.StakeNamed(s.On).ClampY, BambooTrussLayout.StakeNamed(s.On).Z) : Pos(s.On, s.F);

    static IEnumerable<Move> Entry(Ctx c, double[] offsets) =>
        from cane in c.Canes from f in offsets from l in Enumerable.Range(0, c.LSolutions)
        select new Move("brachiate", c.RAnchorStake, null, cane, f, l);

    static IEnumerable<Move> ToStake(Ctx c, Site s) =>
        from st in BambooTrussLayout.Current.Stakes.Select(x => x.Name) where c.Goal(new Site(st, 0, 0))
        from r in c.Canes select new Move("step to stake", "placed-" + r, null, st, 0, 0);

    static string Kind(Site from, string cane) => "placed-" + cane == from.On ? "along" : "across";

    // ---------------- the five gaits ----------------

    static readonly double[] Fine = { -1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0 };
    static readonly double[] Coarse = { -1.0, -0.5, 0.0, 0.5, 1.0 };

    /// <summary>(a) Inchworm: R clamps the same cane right next to L, L re-clamps a short step (≤ 7 cm) further on —
    /// along the cane or onto the neighbouring cane at a node.</summary>
    public static readonly Gait Inchworm = new("inchworm", "Inchworm",
        "R clamps L's own cane next to L; L moves ≤ 7 cm (along the cane, or onto the next cane at a node).", Fine, false,
        (c, s) => s.IsStake ? Entry(c, Fine) : ToStake(c, s).Concat(
            from t in c.Sites() let d = Dist(Pos(t.Cane, t.F), SitePos(s)) where d > 0.01 && d <= 0.07
            orderby d descending
            from l in Enumerable.Range(0, c.LSolutions)
            select new Move("inchworm " + Kind(s, t.Cane), s.On, s.F * Half, t.Cane, t.F, l)));

    /// <summary>(b) Shuffle: R leads — it clamps the next site ahead (up to 13 cm) — and L closes up one grid step
    /// behind R on R's cane.</summary>
    public static readonly Gait Shuffle = new("shuffle", "Shuffle",
        "R leads and clamps up to 13 cm ahead; L closes up one grid step (3 cm) behind R on R's cane.", Fine, false,
        (c, s) => s.IsStake ? Entry(c, Fine) : ToStake(c, s).Concat(
            from lead in c.Sites() let d = Dist(Pos(lead.Cane, lead.F), SitePos(s)) where d > 0.04 && d <= 0.13
            let back = lead.F + (lead.F > -0.99 ? -0.25 : 0.25) * (Dist(Pos(lead.Cane, lead.F - 0.25), SitePos(s)) < d ? 1 : -1)
            where back >= -1.0001 && back <= 1.0001
            orderby d descending
            from l in Enumerable.Range(0, c.LSolutions)
            select new Move("shuffle", "placed-" + lead.Cane, lead.F * Half, lead.Cane, back, l)));

    /// <summary>(c) Brachiation swing: R clamps next to L (≤ 4 cm), L swings past R to the farthest site it can reach
    /// (≥ 10 cm) — the longest stride.</summary>
    public static readonly Gait Swing = new("swing", "Brachiation swing",
        "R clamps right beside L; L swings past R to the farthest reachable site (stride ≥ 10 cm).", Coarse, false,
        (c, s) => s.IsStake ? Entry(c, Coarse) : ToStake(c, s).Concat(
            from t in c.Sites() let d = Dist(Pos(t.Cane, t.F), SitePos(s)) where d >= 0.10
            orderby d descending
            from rOff in new[] { 0.04, -0.04 }
            from l in Enumerable.Range(0, c.LSolutions)
            select new Move("swing", s.On, s.F * Half + rOff, t.Cane, t.F, l)));

    static readonly double[] Nodes = { -1.0, 1.0 };

    /// <summary>(d) Node transfer: L only ever clamps next to a joint (3 cm from A, B, C or D: f = ±1); each step goes
    /// to another node end — the far end of the cane or the other cane at the same node. R clamps anywhere fixed.</summary>
    public static readonly Gait NodeTransfer = new("node", "Node transfer",
        "L only clamps next to a joint (A, B, C, D); each step goes node to node, R clamps wherever it can.", Nodes, false,
        (c, s) => s.IsStake ? Entry(c, Nodes) : ToStake(c, s).Concat(
            from t in c.Sites() where !("placed-" + t.Cane == s.On && Math.Abs(t.F - s.F) < 1e-9)
            from r in c.Canes
            from l in Enumerable.Range(0, c.LSolutions)
            select new Move("node transfer", "placed-" + r, null, t.Cane, t.F, l)));

    /// <summary>(e) Side-clamp roll: inchworm steps plus a roll in place — L lets go and re-clamps the same point from the
    /// other side of the cane (other IK branch), so the arm changes the side it works from.</summary>
    public static readonly Gait Roll = new("roll", "Side-clamp roll",
        "Inchworm plus roll-in-place: L re-clamps the same point from the other side (the arm changes side).", Fine, true,
        (c, s) => s.IsStake ? Entry(c, Fine) : ToStake(c, s).Concat(
            (from l in Enumerable.Range(0, 4)
             select new Move("roll", s.On, (s.F + (s.F > 0 ? -0.5 : 0.5)) * Half, s.On["placed-".Length..], s.F, l))
            .Concat(Inchworm.Moves(c, s).Where(m => m.Kind.StartsWith("inchworm", StringComparison.Ordinal)))));

    public static IReadOnlyList<Gait> Gaits { get; } = new[] { Inchworm, Shuffle, Swing, NodeTransfer, Roll };
    public static Gait Default { get; set; } = NodeTransfer;
    public static Gait GaitNamed(string? id) => Gaits.FirstOrDefault(g => g.Id == id) ?? Default;

    public static int LSolutions { get; set; } = 2;

    /// <summary>Site of the planner's current L base (L must be the base), snapped to <paramref name="offsets"/>.</summary>
    public static Site Current(BambooTrussPlanner p, double[] offsets)
    {
        var on = p.LStake;
        if (BambooTrussLayout.IsSupport(on)) return new Site(on, 0, 0);
        var w = p.BaseWorldNow;
        var d = BambooTrussLayout.Members.First(m => m.Id == on["placed-".Length..]).Place; var u = d.Axis;
        var f = ((w[3] - d.X) * u.X + (w[7] - d.Y) * u.Y + (w[11] - d.Z) * u.Z) / Half;
        var all = offsets.Concat(Fine).Distinct();
        f = all.OrderBy(o => Math.Abs(o - f)).First();
        return new Site(on, f, w[9] >= 0 ? 1 : -1);
    }

    static Site SiteAfter(BambooTrussPlanner p, string on, double f, bool sides) =>
        BambooTrussLayout.IsSupport(on) ? new Site(on, 0, 0)
            : new Site(on, f, sides ? (p.BaseWorldNow[9] >= 0 ? 1 : -1) : 0);

    /// <summary>
    /// Breadth-first search with <paramref name="gait"/> from the planner's current state. Each discovered site keeps a
    /// deep checkpoint of the plan that reached it. When <paramref name="goal"/> is reached the planner is left there
    /// (<see cref="Result.Reached"/>); otherwise (graph exhausted or <paramref name="maxSteps"/> attempts) it is restored.
    /// <paramref name="rAnchorStake"/> is what R clamps when L is on a stake (e.g. "stake-S3@0.13").
    /// </summary>
    public static Result Explore(BambooTrussPlanner p, Gait gait, string rAnchorStake, Func<Site, bool>? goal = null,
        int maxSteps = 400, Action<string>? log = null, double maxSeconds = 90, string? cycleName = null)
    {
        var res = new Result();
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var canes = p.Clampable(p.PlacedIds).ToList();
        var cur = Current(p, gait.Offsets);
        var start = gait.TrackSides ? cur : cur with { Side = 0 };
        var cps = new Dictionary<Site, BambooTrussPlanner.DeepCheckpoint> { [start] = p.SaveDeep() };
        res.BaseAt[start] = p.BaseWorldNow;
        var ctx = new Ctx(canes, rAnchorStake, goal ?? (_ => false), gait.Offsets, LSolutions);
        var queue = new Queue<Site>();
        queue.Enqueue(start);
        res.Reachable.Add(start);
        var failed = new HashSet<string>();
        var rrt0 = p.RrtAttempts; var it0 = p.RrtIters;
        p.RrtAttempts = RrtAttemptsPerStep; p.RrtIters = RrtItersPerStep;
        try {
        while (queue.Count > 0 && res.Tried < maxSteps && clock.Elapsed.TotalSeconds <= maxSeconds)
        {
            var from = queue.Dequeue();
            foreach (var mv in gait.Moves(ctx, from))
            {
                if (res.Tried >= maxSteps || clock.Elapsed.TotalSeconds > maxSeconds) break;
                var toOn = BambooTrussLayout.IsSupport(mv.To) ? mv.To : "placed-" + mv.To;
                var known = res.Reachable.Count(s => s.On == toOn && Math.Abs(s.F - mv.F) < 1e-9);
                if (known >= (gait.TrackSides && !BambooTrussLayout.IsSupport(mv.To) ? 2 : 1)) continue;
                // Same move already failed from this site (another IK index of an identical request).
                var sig = $"{from}|{mv}";
                if (!failed.Add(sig)) continue;
                p.RollbackDeep(cps[from]);
                res.Tried++;
                var cycle = cycleName ?? $"walk {gait.Id}";
                var ok = mv.Kind switch
                {
                    "brachiate" => p.Brachiate(mv.RAnchor, mv.To, mv.F, mv.LSol, cycle),
                    "step to stake" => p.StepToStake(mv.RAnchor, mv.To, cycle, mv.RAtM),
                    _ => p.HandOverHand(mv.RAnchor, mv.To, mv.F, mv.LSol, cycle, mv.RAtM)
                };
                log?.Invoke($"{gait.Id}: {mv.Kind} {from} → {toOn}@{mv.F:+0.00;-0.00} (R {mv.RAnchor}{(mv.RAtM is { } r ? $"@{r:+0.000;-0.000}" : "")}, L sol {mv.LSol}): {(ok ? "ok" : p.Error)}");
                if (!ok) continue;
                res.Planned++;
                var to = SiteAfter(p, toOn, mv.F, gait.TrackSides);
                if (cps.ContainsKey(to)) continue;
                cps[to] = p.SaveDeep();
                res.BaseAt[to] = p.BaseWorldNow;
                res.Parent[to] = new Step(mv.Kind, from, to, mv.RAnchor, Dist(SitePos(from), SitePos(to)));
                res.Reachable.Add(to);
                queue.Enqueue(to);
                if (goal?.Invoke(to) == true) { res.Reached = to; return res; } // planner left at the goal
            }
        }
        p.RollbackDeep(cps[start]);
        return res;
        } finally { p.RrtAttempts = rrt0; p.RrtIters = it0; }
    }

    /// <summary>RRT-Connect attempts per transit while searching steps (1: a step that needs luck is skipped).</summary>
    public static int RrtAttemptsPerStep { get; set; } = 1;
    /// <summary>RRT-Connect iterations per attempt while searching steps.</summary>
    public static int RrtItersPerStep { get; set; } = 300;
}
