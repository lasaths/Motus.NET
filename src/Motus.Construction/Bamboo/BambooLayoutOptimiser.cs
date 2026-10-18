using System.Globalization;

namespace Motus.Construction;

/// <summary>
/// Layout optimiser: a small grid search over the layout knobs that decide reachability — stock pose (offset from the
/// A column, distance off the truss plane, yaw), cane length / node gap, anchor clamp height — scored with the
/// fail-fast scene check (reach map, IK). A layout is feasible when every member is reachable (directly or by a relay);
/// feasible layouts rank by fewest relays, then by the weakest member's direct-site count (margin), then by the total
/// number of stock+place sites. Pure kinematics, no planning: the winner still has to be confirmed by the build search.
/// The current site is restored afterwards.
/// </summary>
public static class BambooLayoutOptimiser
{
    /// <summary>One candidate layout (the arguments of <see cref="SiteDefaults.Make"/>).</summary>
    public sealed record Candidate(double Cane, double Gap, double NodeY, double ClampY, double Heading,
        double FeedDx, double FeedZ, double FeedYaw = 0)
    {
        public BambooTrussLayout.Site Site() => SiteDefaults.Make(Cane, Gap, NodeY, ClampY, Heading, FeedDx, FeedZ, FeedYaw);

        public static Candidate Of(BambooTrussLayout.Site s)
        {
            var a = s.Stakes.Single(x => x.Name == s.StartStake);
            return new Candidate(s.Length, s.Gap, s.NodeY, a.ClampY, a.HeadingDeg, s.FeedX - a.X, s.FeedZ, s.FeedYawDeg);
        }

        public override string ToString() => string.Create(CultureInfo.InvariantCulture,
            $"cane {Cane:0.000} gap {Gap:0.000} clampY {ClampY:0.000} stock dx {FeedDx:+0.000;-0.000} z {FeedZ:0.000} yaw {FeedYaw:0}°");
    }

    public sealed record Scored(Candidate Layout, bool Feasible, int Reachable, int Relays, int MinDirect, int TotalBoth,
        double Seconds, string Summary)
    {
        public override string ToString() =>
            $"{(Feasible ? "ok " : "NO ")} {Layout} — {Reachable}/7 reachable, {Relays} relay(s), weakest {MinDirect}, Σboth {TotalBoth} ({Seconds:F1} s)";
    }

    public sealed record Result(IReadOnlyList<Scored> Ranked, Scored? Best, Scored? Baseline, double Seconds)
    {
        /// <summary>True when the baseline (current) layout is feasible and no candidate strictly beats it.</summary>
        public bool BaselineIsBest => Baseline is { Feasible: true } b && Best is not null && Compare(b, Best) <= 0;
        public IEnumerable<string> Lines => Ranked.Select((s, i) => $"{i + 1,2}. {s}");
    }

    /// <summary>Search options: the values swept per knob (empty ⇒ keep the base layout's value).</summary>
    public sealed record Options
    {
        public IReadOnlyList<double> FeedDx { get; init; } = Array.Empty<double>();
        public IReadOnlyList<double> FeedZ { get; init; } = Array.Empty<double>();
        public IReadOnlyList<double> FeedYaw { get; init; } = Array.Empty<double>();
        /// <summary>(cane, gap) pairs.</summary>
        public IReadOnlyList<(double Cane, double Gap)> CaneGap { get; init; } = Array.Empty<(double, double)>();
        public IReadOnlyList<double> ClampY { get; init; } = Array.Empty<double>();
        /// <summary>Scene-check resolution: the planner's own (default). <see cref="Coarse"/> is ≈ 3× faster but misses
        /// sites (on Warren it reports two relays where the full check finds one), so use it only for a first pass.</summary>
        public BambooSceneCheck.Options Check { get; init; } = new();
        public static BambooSceneCheck.Options Coarse { get; } = new()
        {
            CaneOffsets = new[] { -1.0, 0.0, 1.0 },
            Rolls = new[] { 0.0, 90, 180, 270 },
            AnchorHeadingStepDeg = 45,
        };
        /// <summary>Hard cap on evaluated candidates (grid order; the base layout is always evaluated first).</summary>
        public int MaxCandidates { get; init; } = 64;
    }

    /// <summary>The grid: the base layout first, then every combination of the swept values (deduplicated).</summary>
    public static List<Candidate> Grid(Candidate b, Options o)
    {
        IEnumerable<T> Or<T>(IReadOnlyList<T> v, T d) => v.Count > 0 ? v : new[] { d };
        var list = new List<Candidate> { b };
        foreach (var (cane, gap) in Or(o.CaneGap, (b.Cane, b.Gap)))
        foreach (var cy in Or(o.ClampY, b.ClampY))
        foreach (var dx in Or(o.FeedDx, b.FeedDx))
        foreach (var fz in Or(o.FeedZ, b.FeedZ))
        foreach (var yaw in Or(o.FeedYaw, b.FeedYaw))
        {
            var c = b with { Cane = cane, Gap = gap, ClampY = cy, FeedDx = dx, FeedZ = fz, FeedYaw = yaw };
            if (!list.Contains(c)) list.Add(c);
        }
        return list.Take(Math.Max(1, o.MaxCandidates)).ToList();
    }

    /// <summary>Score one layout with the scene check (switches the site; the caller restores it).</summary>
    public static Scored Evaluate(Candidate c, BambooSceneCheck.Options? check = null)
    {
        BambooTrussLayout.Use(c.Site());
        var r = BambooSceneCheck.Run(null, null, check ?? new Options().Check);
        var direct = r.Members.Select(m => m.SitesBoth).ToList();
        return new Scored(c, r.Ok, r.Members.Count(m => m.Reachable), r.Members.Count(m => m.NeedsRelay),
            direct.Count > 0 ? direct.Where(d => d > 0).DefaultIfEmpty(0).Min() : 0, direct.Sum(), r.Seconds, r.Summary);
    }

    /// <summary>Ranking: feasible first, then fewer relays, more reachable members, larger weakest margin, larger total.</summary>
    public static int Compare(Scored a, Scored b)
    {
        var c = b.Feasible.CompareTo(a.Feasible);
        if (c == 0) c = b.Reachable.CompareTo(a.Reachable);
        if (c == 0) c = a.Relays.CompareTo(b.Relays);
        if (c == 0) c = b.MinDirect.CompareTo(a.MinDirect);
        if (c == 0) c = b.TotalBoth.CompareTo(a.TotalBoth);
        return c;
    }

    /// <summary>Evaluate the grid around <paramref name="baseLayout"/> (default: the current site) and rank it.</summary>
    public static Result Run(Options? options = null, BambooTrussLayout.Site? baseLayout = null, Action<string>? log = null)
    {
        var o = options ?? new Options();
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var saved = BambooTrussLayout.Current;
        var scored = new List<Scored>();
        try
        {
            foreach (var c in Grid(Candidate.Of(baseLayout ?? saved), o))
            {
                var s = Evaluate(c, o.Check);
                scored.Add(s);
                log?.Invoke(s.ToString());
            }
        }
        finally { BambooTrussLayout.Use(saved); }
        var baseline = scored.FirstOrDefault();
        // Stable sort: ties keep grid order, so the baseline wins ties.
        var ranked = scored.Select((s, i) => (s, i)).OrderBy(x => x.s, Comparer<Scored>.Create(Compare)).ThenBy(x => x.i).Select(x => x.s).ToList();
        return new Result(ranked, ranked.FirstOrDefault(r => r.Feasible), baseline, clock.Elapsed.TotalSeconds);
    }
}
