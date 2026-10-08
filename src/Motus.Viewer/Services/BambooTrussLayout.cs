using Motus.Core;
using K = Motus.Viewer.Services.BambooGripKinematics;

namespace Motus.Viewer.Services;

/// <summary>
/// Store/truss member poses and fixed ground fixtures for /bamboo-truss (viewer coordinates, Y-up).
/// <para>
/// Five-member bridge frame in the vertical XY plane, every edge one cane of <see cref="StrutLength"/>:
/// <code>
///        C
///       /|\        baseL A–B, baseR B–D (lower chord), sep B–C, diagL A–C, diagR D–C
///      / | \       canes meeting at a node are lashed side by side, 16 mm apart in z
///     A--B--D
/// </code>
/// Ends A and D sit on anchor posts (pads under them, no earth between). The cane length is chosen by an offline
/// reach + Motus sweep (see <see cref="Current"/>): the longest round length at which the arm builds the most.
/// </para>
/// <para>
/// <b>Grounded start, no floating mount.</b> The arm starts with its L-grip clamped on anchor stake
/// <see cref="S1"/> (a 14 mm post planted on its own pad beside the stock feed). Every instant at least one gripper
/// is clamped on a real fixed object, and that gripper is the IK base. The L-grip is a closed clamp (rail + body on
/// one side, base tube on the other, jaws on both — the cane cannot leave it sideways at any opening), so it can only
/// come off a support by sliding over a free end: the stakes' tops. The R-grip (open slot) clamps the canes.
/// Inside the span only canes fixed at both ends may be clamped (by the open-slot R-grip).
/// </para>
/// </summary>
public static class BambooTrussLayout
{
    // ---------------- site (cane length, supports, stock) ----------------

    /// <summary>A vertical stake (slot axis = stake axis = viewer +Y). HeadingDeg is the arm heading for a start clamp.</summary>
    public sealed record Stake(string Name, double X, double Z, double ClampY, double Top, double HeadingDeg)
    {
        /// <summary>Grip target on the stake at <see cref="ClampY"/>.</summary>
        public K.GripTarget Grip => new(X, ClampY, Z, 0, 1, 0, Name);
        public K.GripTarget At(double y) => new(X, y, Z, 0, 1, 0, Name);
    }

    /// <summary>
    /// Everything that depends on the chosen cane length: the cane length itself, the outboard stakes, the stock and
    /// the walk. <see cref="Unreachable"/> lists members no allowed support reaches (with the reason) — the planner
    /// reports them instead of faking them.
    /// </summary>
    public sealed record Site(
        double Length,
        IReadOnlyList<Stake> Stakes,
        string StartStake,
        double FeedX, double FeedZ, double FeedYawDeg, double FeedTop,
        string WalkRAnchor, string WalkLTarget,
        IReadOnlyDictionary<string, string> Unreachable);

    /// <summary>
    /// Shipped site: 0.35 m canes (chosen by the length sweep in the README — the longest 0.05 m multiple at which
    /// the true-IK planner completes the double-fixed lower chord and more). Positions from an offline reach + Motus
    /// clearance search.
    /// </summary>
    public static Site Default { get; } = SiteDefaults.Chosen;

    public static Site Current { get; private set; } = Default;

    /// <summary>Switch the site (offline sweeps / tests). Clears every derived cache.</summary>
    public static void Use(Site site)
    {
        Current = site;
        _fixtures = null; _members = null;
    }

    /// <summary>Truss cane length (metres). All edge canes are this length.</summary>
    public static double StrutLength => Current.Length;
    /// <summary>Rhombus side (= cane length; canes meet at the nodes).</summary>
    public static double Side => Current.Length;
    /// <summary>Upright B–C (= cane length).</summary>
    public static double Height => Current.Length;

    public const double JawClosedMm = 14;
    public const double JawOpenMm = 40;
    /// <summary>L-grip fully open while sliding along a stake (jaw plates 15 mm off the 14 mm stake).</summary>
    public const double LJawOpenMm = 45;

    /// <summary>Vertical plane of the truss (viewer Z).</summary>
    public const double PlaneZ = 0.11;
    /// <summary>Lash spacing: canes meeting at a node run side by side 16 mm apart in z (14 mm cane + 2 mm gap).</summary>
    public const double LashDz = -0.016;
    /// <summary>Ground-side node B height (metres).</summary>
    public const double FootY0 = 0.06;
    /// <summary>Half-width |x| of A and D.</summary>
    public static double HalfSpan => Math.Sqrt(Side * Side - (Height / 2) * (Height / 2));
    /// <summary>Half-extent of Motus ground pads about anchors A and D (metres).</summary>
    public const double AnchorPadHalf = 0.06;
    /// <summary>Anchor posts under A and D (radius, and gap below the node so the cane end clears the post top).</summary>
    public const double AnchorPostRadius = 0.012;
    public const double AnchorPostGap = 0.02;

    public const double StakeRadius = 0.007;
    public const double StakePadHalf = 0.03;
    /// <summary>Stake top above its clamp height: the L clamp slides off / onto the free top.</summary>
    public const double StakeTopAbove = 0.02;
    /// <summary>How far above the top the L clamp clears before it moves sideways.</summary>
    public const double StakeSlideClear = 0.018;

    /// <summary>
    /// Every support an L clamp may use: outboard stakes ("stake-*") and, optionally, anchor post A ("post-A": its
    /// free top lets the closed L clamp slide on/off until a cane end is lashed above it).
    /// </summary>
    public static IReadOnlyList<Stake> Stakes => Current.Stakes;
    static bool IsStake(Stake s) => s.Name.StartsWith("stake-", StringComparison.Ordinal);
    /// <summary>Post-A as an L-clamp support at a clamp height / heading.</summary>
    public static Stake PostA(double clampY, double heading) =>
        new("post-A", -HalfSpan, PlaneZ, clampY, FootY0 + Height / 2 - AnchorPostGap, heading);
    public static Stake StakeNamed(string name) => Current.Stakes.Single(s => s.Name == name);
    /// <summary>Start stake (L-grip clamped at t = 0).</summary>
    public static Stake S1 => StakeNamed(Current.StartStake);
    /// <summary>Walk target stake for the L clamp.</summary>
    public static Stake WalkTarget => StakeNamed(Current.WalkLTarget);

    /// <summary>Exact L-grip clamp frame on <see cref="S1"/> (viewer 4×4): the IK base at t = 0.</summary>
    public static double[] StartBase => K.StakeClampFrame(S1.X, S1.ClampY, S1.Z, S1.HeadingDeg);

    /// <summary>
    /// Stock: the cane lies flat next to anchor A on two narrow pegs near its ends, leaving the grasp zone free of
    /// anything the closing jaws could hit. 0.5 mm seat gap: Motus counts resting contact as a hit.
    /// </summary>
    public static double RackTop => Current.FeedTop;
    public const double RackHalfAlong = 0.015;
    public const double RackHalfAcross = 0.004;
    /// <summary>Pegs sit well inside the cane ends so the jaw can also take the cane near either end (the far reach a
    /// placement at the opposite end of the span needs); grasps run between the pegs or outboard of them.</summary>
    public static double RackPegOffset => StrutLength / 2 - 0.085;
    /// <summary>Grasp offsets along the stock cane (from its centre), clear of the pegs: near each end first.</summary>
    public static double[] StockGraspOffsets
    {
        get
        {
            var e = StrutLength / 2;
            var inner = RackPegOffset - 0.035;
            return new[] { -(e - 0.025), e - 0.025, -(e - 0.045), e - 0.045, 0.0, -inner, inner }.Select(v => Math.Round(v, 3)).ToArray();
        }
    }
    public static double FeedX => Current.FeedX;
    public static double FeedZ => Current.FeedZ;
    public const double RackSeatGap = 0.0005;
    public static double StrutY => RackTop + 0.007 + RackSeatGap;
    static (double X, double Z) FeedAxis => (Math.Sin(Current.FeedYawDeg * Math.PI / 180), Math.Cos(Current.FeedYawDeg * Math.PI / 180));
    static bool FeedAlongX => Math.Abs(FeedAxis.X) > 0.5;

    /// <summary>Ground pads (viewer XZ centre + half extents): anchors, stakes and the stock — no earth elsewhere.</summary>
    public static IReadOnlyList<(string Name, double X, double Z, double HalfX, double HalfZ)> Pads
    {
        get
        {
            var list = new List<(string, double, double, double, double)>
            {
                ("ground-A", -HalfSpan, PlaneZ, AnchorPadHalf, AnchorPadHalf),
                ("ground-D", HalfSpan, PlaneZ + LashDz, AnchorPadHalf, AnchorPadHalf)
            };
            foreach (var st in Stakes.Where(IsStake)) list.Add(("ground-" + st.Name["stake-".Length..], st.X, st.Z, StakePadHalf, StakePadHalf));
            var along = StrutLength / 2 - 0.02; var across = 0.02;
            list.Add(("ground-store", FeedX, FeedZ, FeedAlongX ? along : across, FeedAlongX ? across : along));
            return list;
        }
    }

    /// <summary>Anchor posts (A, D) and stakes as vertical cylinders: (name, x, z, top, radius).</summary>
    public static IReadOnlyList<(string Name, double X, double Z, double Top, double Radius)> Posts
    {
        get
        {
            var list = new List<(string, double, double, double, double)>
            {
                ("post-A", -HalfSpan, PlaneZ, FootY0 + Height / 2 - AnchorPostGap, AnchorPostRadius),
                ("post-D", HalfSpan, PlaneZ + LashDz, FootY0 + Height / 2 - AnchorPostGap, AnchorPostRadius)
            };
            foreach (var st in Stakes.Where(IsStake)) list.Add((st.Name, st.X, st.Z, st.Top, StakeRadius));
            return list;
        }
    }

    /// <summary>Stock pegs as boxes (viewer): name, centre x/z, half extents x/z, top y (bottom on the ground).</summary>
    public static IReadOnlyList<(string Name, double X, double Z, double HalfX, double HalfZ, double Top)> Racks
    {
        get
        {
            var (ax, az) = FeedAxis; var o = RackPegOffset;
            var hx = FeedAlongX ? RackHalfAlong : RackHalfAcross; var hz = FeedAlongX ? RackHalfAcross : RackHalfAlong;
            return
            [
                ("store-peg-1", FeedX - ax * o, FeedZ - az * o, hx, hz, RackTop),
                ("store-peg-2", FeedX + ax * o, FeedZ + az * o, hx, hz, RackTop)
            ];
        }
    }

    /// <summary>
    /// Every ground object (pads, posts, pegs) as an x-extent. Only the A/D anchor pad + post may reach into the span;
    /// everything else must lie at x ≤ −HalfSpan or x ≥ +HalfSpan.
    /// </summary>
    public static IEnumerable<(string Name, double MinX, double MaxX, bool IsAnchor)> GroundXExtents() =>
        Pads.Select(p => (p.Name, p.X - p.HalfX, p.X + p.HalfX, p.Name is "ground-A" or "ground-D"))
            .Concat(Posts.Select(p => (p.Name, p.X - p.Radius, p.X + p.Radius, p.Name is "post-A" or "post-D")))
            .Concat(Racks.Select(r => (r.Name, r.X - r.HalfX, r.X + r.HalfX, false)));

    static IReadOnlyList<CollisionObject>? _fixtures;
    /// <summary>Motus objects for <see cref="Posts"/> and <see cref="Racks"/> (cached per site; pads come from the cell).</summary>
    public static IReadOnlyList<CollisionObject> FixtureObjects =>
        _fixtures ??= Posts.Select(p => BambooIcdCell.PostAt(p.Name, p.X, p.Z, 0, p.Top, p.Radius))
            .Concat(Racks.Select(r => BambooIcdCell.BoxAt(r.Name, r.X, r.Top / 2, r.Z, r.HalfX, r.Top / 2, r.HalfZ)))
            .ToList();

    /// <summary>Configure a cell for /bamboo-truss: no hang rod, pads only.</summary>
    public static void Configure(BambooIcdCell cell)
    {
        cell.IncludePole = false;
        cell.SetGroundPads(Pads);
    }

    public sealed record MemberDef(
        string Id,
        string Label,
        string Glyph,
        BambooIcdCell.StrutPlacement Store,
        BambooIcdCell.StrutPlacement Place,
        (double X, double Y, double Z) EdgeA,
        (double X, double Y, double Z) EdgeB);

    /// <summary>Stock feed: cane flat on its pegs next to anchor A.</summary>
    public static BambooIcdCell.StrutPlacement Store => new(FeedX, StrutY, FeedZ, 0, Current.FeedYawDeg);

    /// <summary>Truss nodes: A/D anchors (on posts), B lower mid, C apex.</summary>
    public static (string Name, double X, double Y, double Z)[] FootNodes =>
    [
        ("A", -HalfSpan, FootY0 + Height / 2, PlaneZ),
        ("B", 0, FootY0, PlaneZ),
        ("D", HalfSpan, FootY0 + Height / 2, PlaneZ + LashDz),
        ("C", 0, FootY0 + Height, PlaneZ)
    ];

    static IReadOnlyList<MemberDef>? _members;
    /// <summary>The five members in build order (each cane in its own lash layer so canes meeting at a node never touch).</summary>
    public static IReadOnlyList<MemberDef> Members => _members ??= BuildMembers();

    static IReadOnlyList<MemberDef> BuildMembers()
    {
        (double, double, double) At(string node, double dz)
        {
            var n = FootNodes.First(f => f.Name == node);
            return (n.X, n.Y, PlaneZ + dz);
        }
        var edges = new (string Id, string Label, string Glyph, (double X, double Y, double Z) P, (double X, double Y, double Z) Q)[]
        {
            ("baseL", "base left", "/", At("A", 0), At("B", 0)),
            ("baseR", "base right", "_", At("B", LashDz), At("D", LashDz)),
            ("diagL", "diag left", "/", At("A", LashDz), At("C", LashDz)),
            ("sep", "separator", "|", At("B", -LashDz), At("C", -LashDz)),
            ("diagR", "diag right", "\\", At("D", 0), At("C", 0))
        };
        return edges.Select(e => new MemberDef(e.Id, e.Label, e.Glyph, Store, AlongEdge(e.P, e.Q), e.P, e.Q)).ToList();
    }

    /// <summary>
    /// Cane of <see cref="StrutLength"/> centred on edge P→Q, axis along the edge.
    /// </summary>
    public static BambooIcdCell.StrutPlacement AlongEdge(
        (double X, double Y, double Z) p, (double X, double Y, double Z) q)
    {
        var dx = q.X - p.X; var dy = q.Y - p.Y; var dz = q.Z - p.Z;
        var len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        if (len < StrutLength - 1e-6)
            throw new InvalidOperationException($"edge length {len:F4} m < strut {StrutLength} m");
        dx /= len; dy /= len; dz /= len;
        var (pitch, yaw) = PitchYawForAxis(dx, dy, dz);
        var alt = PitchYawForAxis(-dx, -dy, -dz);
        if (PreferAlt(pitch, yaw, alt.Pitch, alt.Yaw))
            (pitch, yaw) = alt;
        return new BambooIcdCell.StrutPlacement((p.X + q.X) / 2, (p.Y + q.Y) / 2, (p.Z + q.Z) / 2, pitch, yaw);
    }

    static (double Pitch, double Yaw) PitchYawForAxis(double dx, double dy, double dz)
    {
        if (dx * dx + dz * dz < 1e-12)
            return (dy >= 0 ? -90 : 90, 0);
        var pitch = Math.Atan2(-dy, Math.Sqrt(dx * dx + dz * dz)) * 180 / Math.PI;
        var yaw = Math.Atan2(dx, dz) * 180 / Math.PI;
        return (pitch, yaw);
    }

    static bool PreferAlt(double p0, double y0, double p1, double y1)
    {
        if (Math.Abs(Math.Abs(y1) - 90) < 1 && p1 <= 0 && p0 > 0) return true;
        if (Math.Abs(y1) < 1e-9 && Math.Abs(p1 + 90) < 1) return true;
        return false;
    }

    /// <summary>Members no allowed support can reach, with the reason (reported, never faked).</summary>
    public static IReadOnlyDictionary<string, string> UnreachableMembers => Current.Unreachable;

    /// <summary>
    /// Nodes that are fixed once <paramref name="placed"/> members are lashed: A and D (posts), and any node held by
    /// two placed members whose other ends are already fixed (a triangulated, statically determinate joint).
    /// </summary>
    public static HashSet<string> FixedNodes(IEnumerable<string> placed)
    {
        var set = placed.ToHashSet(StringComparer.Ordinal);
        var ends = new Dictionary<string, (string P, string Q)>
        {
            ["baseL"] = ("A", "B"), ["baseR"] = ("B", "D"), ["sep"] = ("B", "C"), ["diagL"] = ("A", "C"), ["diagR"] = ("D", "C")
        };
        var fixedNodes = new HashSet<string>(StringComparer.Ordinal) { "A", "D" };
        for (var changed = true; changed;)
        {
            changed = false;
            foreach (var node in new[] { "B", "C" })
            {
                if (fixedNodes.Contains(node)) continue;
                var supports = ends.Where(e => set.Contains(e.Key) && (e.Value.P == node || e.Value.Q == node))
                    .Count(e => fixedNodes.Contains(e.Value.P == node ? e.Value.Q : e.Value.P));
                if (supports >= 2) { fixedNodes.Add(node); changed = true; }
            }
        }
        return fixedNodes;
    }

    /// <summary>Placed members fixed at both ends (the only canes the robot may clamp inside the span).</summary>
    public static IReadOnlyList<string> FixedBothEnds(IEnumerable<string> placed)
    {
        var list = placed.ToList();
        var nodes = FixedNodes(list);
        return Members.Where(m => list.Contains(m.Id))
            .Where(m => nodes.Contains(NodeOf(m.EdgeA)) && nodes.Contains(NodeOf(m.EdgeB)))
            .Select(m => m.Id).ToList();
    }

    static string NodeOf((double X, double Y, double Z) p) =>
        FootNodes.OrderBy(n => Math.Abs(n.X - p.X) + Math.Abs(n.Y - p.Y)).First().Name;

    /// <summary>Members fixed at both ends after the first <paramref name="placedCount"/> members are placed.</summary>
    public static IReadOnlyList<string> WalkableMemberIdsAfter(int placedCount) =>
        FixedBothEnds(Members.Take(placedCount).Select(m => m.Id));

    public static object FootprintDescription() => new
    {
        nodes = FootNodes.Select(n => new { name = n.Name, x = n.X, y = n.Y, z = n.Z }),
        edges = Members.Select(m => new
        {
            id = m.Id,
            glyph = m.Glyph,
            ax = m.EdgeA.X, ay = m.EdgeA.Y, az = m.EdgeA.Z,
            bx = m.EdgeB.X, by = m.EdgeB.Y, bz = m.EdgeB.Z
        }),
        anchors = Pads.Select(a => new { name = a.Name, x = a.X, z = a.Z, halfX = a.HalfX, halfZ = a.HalfZ }),
        posts = Posts.Select(p => new { name = p.Name, x = p.X, z = p.Z, top = p.Top, radius = p.Radius }),
        racks = Racks.Select(r => new { name = r.Name, x = r.X, z = r.Z, halfX = r.HalfX, halfZ = r.HalfZ, top = r.Top }),
        motusLimit = $"{StrutLength:F2} m canes; no earth inside the span; L-grip on outboard stakes; R places; unreachable: " +
                     (UnreachableMembers.Count == 0 ? "none" : string.Join(", ", UnreachableMembers.Keys))
    };

    public static object StoreDescription() => new
    {
        slots = new[] { new { id = "feed", x = Store.X, y = Store.Y, z = Store.Z, pitch = Store.PitchDeg, yaw = Store.YawDeg } }
    };
}
