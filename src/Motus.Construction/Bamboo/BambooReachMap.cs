using Motus.Core;
using K = Motus.Construction.BambooGripKinematics;

namespace Motus.Construction;

/// <summary>
/// Reach map: for a clamp site (the base gripper's world frame on an anchor column or a cane), which grasp targets the
/// free gripper reaches by true IK — optionally only with a Motus-clear pose. Pure kinematics + the shared collision
/// check; no planning, no motion. Results are deterministic (fixed IK seeds; parallel queries are gathered by index).
/// </summary>
public sealed class BambooReachMap
{
    /// <summary>A clamp site: the base (L) gripper clamps <c>On</c> at <c>OffM</c> along it, rolled <c>RollDeg</c> about
    /// its axis (anchor columns: <c>RollDeg</c> = heading); <c>BaseWorld</c> is the clamp frame (viewer 4×4).</summary>
    public sealed record ClampSite(string On, double OffM, double RollDeg, double[] BaseWorld)
    {
        public (double X, double Y, double Z) Origin => (BaseWorld[3], BaseWorld[7], BaseWorld[11]);
        public override string ToString() => BambooTrussLayout.IsSupport(On) ? $"{On} h{RollDeg:0}°" : $"{On}{OffM:+0.000;-0.000}m r{RollDeg:0}°";
    }

    /// <summary>A target line: a cane pose (stock, a member's place, a parked relay) and the grasp offsets along it.</summary>
    public sealed record TargetLine(string Name, BambooIcdCell.StrutPlacement Pose, IReadOnlyList<double> Offsets)
    {
        /// <summary>Scene object the free gripper's jaw plates may touch at this target (the cane itself).</summary>
        public string? Touch { get; init; }
        public IEnumerable<K.GripTarget> Grips => Offsets.Select(o => K.OnStrut(Pose, o, Name));
    }

    /// <summary>Result of one query: IK solutions found for the first reachable grasp offset (and how many were Motus-clear).</summary>
    public readonly record struct Reach(bool Ik, bool Clear, double OffsetM, int Solutions);

    public BambooIcdCell? Cell { get; }
    public MotionConstraints Constraints { get; }
    /// <summary>Random IK seeds per grasp offset.</summary>
    public int IkSeeds { get; init; } = 6;
    /// <summary>Check the IK pose against <see cref="Environment"/> (needs <see cref="Cell"/>).</summary>
    public bool Collision { get; init; }
    /// <summary>Obstacles for collision-checked queries (default: ground, fixtures, the design scene's members).</summary>
    public IReadOnlyList<CollisionObject>? Environment { get; init; }

    List<CollisionObject>? _designEnv;

    public BambooReachMap(BambooIcdCell? cell = null, MotionConstraints? constraints = null)
    {
        Cell = cell;
        Constraints = constraints ?? MotionConstraints.Current;
    }

    // ---------------- sites ----------------

    /// <summary>Clamp sites on an anchor column: the configured clamp height, headings in <paramref name="stepDeg"/> steps (configured first).</summary>
    public static List<ClampSite> AnchorSites(string anchor, double stepDeg = 30)
    {
        var st = BambooTrussLayout.StakeNamed(anchor);
        var list = new List<ClampSite> { new(anchor, 0, st.HeadingDeg, K.StakeClampFrame(st.X, st.ClampY, st.Z, st.HeadingDeg)) };
        for (var h = 0.0; h < 360 - 1e-9; h += stepDeg)
            if (Math.Abs(((h - st.HeadingDeg) % 360 + 360) % 360) > 1e-6)
                list.Add(new ClampSite(anchor, 0, h, K.StakeClampFrame(st.X, st.ClampY, st.Z, h)));
        return list;
    }

    /// <summary>
    /// Side-clamp sites on a cane: slot axis (local Z) along the cane, jaw opening (local Y) rolled about it, at
    /// <paramref name="offsets"/> (m from the centre) × <paramref name="rolls"/> roll angles.
    /// </summary>
    public static List<ClampSite> CaneSites(string on, BambooIcdCell.StrutPlacement pose, IEnumerable<double> offsets, IEnumerable<double> rolls)
    {
        var u = pose.Axis;
        var helper = Math.Abs(u.Y) < 0.9 ? (X: 0.0, Y: 1.0, Z: 0.0) : (X: 1.0, Y: 0.0, Z: 0.0);
        var e1 = Unit(Cross(helper, u));
        var e2 = Cross(u, e1);
        var list = new List<ClampSite>();
        foreach (var o in offsets)
            foreach (var r in rolls)
            {
                var a = r * Math.PI / 180;
                var y = (X: Math.Cos(a) * e1.X + Math.Sin(a) * e2.X, Y: Math.Cos(a) * e1.Y + Math.Sin(a) * e2.Y, Z: Math.Cos(a) * e1.Z + Math.Sin(a) * e2.Z);
                var x = Cross(y, u);
                var p = (X: pose.X + u.X * o, Y: pose.Y + u.Y * o, Z: pose.Z + u.Z * o);
                list.Add(new ClampSite(on, o, r, K.FromCols(x, y, u, p)));
            }
        return list;
    }

    // ---------------- targets ----------------

    public static TargetLine Stock() => new("stock", BambooTrussLayout.Store, BambooTrussLayout.StockGraspOffsets) { Touch = BambooIcdCell.Strut };

    /// <summary>A member's place line (or a relay's parked pose), grasped at the stock offsets.</summary>
    public static TargetLine PlaceOf(BambooTrussLayout.MemberDef m) => new("place " + m.Id, m.Place, BambooTrussLayout.StockGraspOffsets) { Touch = "placed-" + m.Id };

    // ---------------- queries ----------------

    static double? _maxReach;

    /// <summary>Largest base-origin → free-gripper-origin distance over the joint limits (sampled, +2 cm margin): a sound IK prefilter.</summary>
    public static double MaxReachM
    {
        get
        {
            if (_maxReach is { } m) return m;
            var rng = new Random(3);
            var best = 0.0;
            var q = new double[5];
            for (var i = 0; i < 20000; i++)
            {
                for (var k = 0; k < 5; k++) { var (lo, hi) = BambooIcdMotion.LimitsDeg[k]; q[k] = lo + (hi - lo) * rng.NextDouble(); }
                var t = K.Tlr(q);
                best = Math.Max(best, Math.Sqrt(t[3] * t[3] + t[7] * t[7] + t[11] * t[11]));
            }
            return (_maxReach = best + 0.02).Value;
        }
    }

    // ---------------- cache ----------------

    static readonly System.Collections.Concurrent.ConcurrentDictionary<string, Reach> _cache = new();
    const int CacheCap = 250_000;
    static long _hits, _misses;

    /// <summary>Cached reach queries so far (hits, misses). Query results are pure functions of the key.</summary>
    public static (long Hits, long Misses, int Entries) CacheStats => (Interlocked.Read(ref _hits), Interlocked.Read(ref _misses), _cache.Count);

    /// <summary>Drop every cached query (tests / benchmarks).</summary>
    public static void ClearCache() { _cache.Clear(); Interlocked.Exchange(ref _hits, 0); Interlocked.Exchange(ref _misses, 0); }

    /// <summary>Use the process-wide query cache (default on). Off for custom <see cref="Environment"/>s (not keyable).</summary>
    public bool UseCache { get; init; } = true;

    /// <summary>
    /// Cache key: the layout fingerprint (covers the design environment and every scene setting), IK seeds, collision
    /// flag, the clamp frame and the target line (pose, offsets, touch), all at the fingerprint's 6-decimal precision.
    /// </summary>
    string? Key(ClampSite site, TargetLine line, string fingerprint)
    {
        if (!UseCache || (Collision && Environment is not null)) return null;
        var sb = new System.Text.StringBuilder(fingerprint.Length + 256);
        sb.Append(fingerprint).Append('|').Append(IkSeeds).Append('|').Append(Collision ? 'c' : 'k').Append('|').Append(site.On).Append('|');
        foreach (var v in site.BaseWorld) sb.Append(BambooTrussMotion.FingerprintNumber(v)).Append(',');
        var p = line.Pose;
        sb.Append('|').Append(line.Touch).Append('|');
        foreach (var v in new[] { p.X, p.Y, p.Z, p.PitchDeg, p.YawDeg }) sb.Append(BambooTrussMotion.FingerprintNumber(v)).Append(',');
        sb.Append('|');
        foreach (var o in line.Offsets) sb.Append(BambooTrussMotion.FingerprintNumber(o)).Append(',');
        return sb.ToString();
    }

    /// <summary>Does the free gripper reach <paramref name="line"/> from <paramref name="site"/>? First reachable offset wins.</summary>
    public Reach Query(ClampSite site, TargetLine line) => Query(site, line, BambooTrussMotion.LayoutFingerprint);

    Reach Query(ClampSite site, TargetLine line, string fingerprint)
    {
        var key = Key(site, line, fingerprint);
        if (key is not null && _cache.TryGetValue(key, out var hit)) { Interlocked.Increment(ref _hits); return hit; }
        Interlocked.Increment(ref _misses);
        var r = Compute(site, line);
        if (key is not null)
        {
            if (_cache.Count >= CacheCap) _cache.Clear();
            _cache[key] = r;
        }
        return r;
    }

    Reach Compute(ClampSite site, TargetLine line)
    {
        var o = site.Origin;
        foreach (var off in line.Offsets)
        {
            var g = K.OnStrut(line.Pose, off, line.Name);
            var d = Math.Sqrt((g.X - o.X) * (g.X - o.X) + (g.Y - o.Y) * (g.Y - o.Y) + (g.Z - o.Z) * (g.Z - o.Z));
            if (d > MaxReachM) continue;
            var sols = K.SolveGrip(K.Gripper.L, site.BaseWorld, g, null, IkSeeds);
            if (sols.Count == 0) continue;
            if (!Collision) return new Reach(true, true, off, sols.Count);
            var env = Environment ?? _designEnv ?? DesignEnvironment(Cell!);
            var clear = 0;
            lock (Cell!)
                foreach (var q in sols)
                    if (BambooTrussCollision.Contacts(Cell, "L", site.BaseWorld, q, BambooTrussLayout.JawOpenMm, BambooTrussLayout.JawClosedMm,
                            null, env, site.On, line.Touch, 1, Constraints.ClearanceMarginM).Count == 0) clear++;
            if (clear > 0) return new Reach(true, true, off, clear);
        }
        return default;
    }

    /// <summary>Reach matrix: <c>[site, line]</c>, computed in parallel, gathered by index (deterministic).</summary>
    public Reach[,] Matrix(IReadOnlyList<ClampSite> sites, IReadOnlyList<TargetLine> lines)
    {
        var m = new Reach[sites.Count, lines.Count];
        // Scene objects are built once, before the parallel queries (layout caches are not thread-safe).
        if (Collision && Environment is null) _designEnv ??= DesignEnvironment(Cell!);
        var fp = BambooTrussMotion.LayoutFingerprint; // once, not per query
        Parallel.For(0, sites.Count * lines.Count, i => m[i / lines.Count, i % lines.Count] = Query(sites[i / lines.Count], lines[i % lines.Count], fp));
        return m;
    }

    /// <summary>Ground, fixtures and every member of the design at its place pose (for collision-checked queries).</summary>
    public static List<CollisionObject> DesignEnvironment(BambooIcdCell cell) =>
        BambooTrussCollision.Environment(cell, BambooTrussLayout.Members.Select(m => ("placed-" + m.Id, BambooIcdCell.PlacementWorld(m.Place))),
            null, null, null, heldTracked: true, hold: 0);

    static (double X, double Y, double Z) Cross((double X, double Y, double Z) a, (double X, double Y, double Z) b) =>
        (a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);

    static (double X, double Y, double Z) Unit((double X, double Y, double Z) a)
    {
        var n = Math.Sqrt(a.X * a.X + a.Y * a.Y + a.Z * a.Z);
        return (a.X / n, a.Y / n, a.Z / n);
    }
}
