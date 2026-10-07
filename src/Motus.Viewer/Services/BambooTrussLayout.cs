using Motus.Core;
using K = Motus.Viewer.Services.BambooGripKinematics;

namespace Motus.Viewer.Services;

/// <summary>
/// Store/truss member poses and fixed ground fixtures for /bamboo-truss (viewer coordinates, Y-up).
/// <para>
/// Bridge lower chord in the vertical XY plane at <see cref="StrutLength"/> = 0.5 m:
/// <code>
///    A--B--D     (baseL /  +  baseR _ ; baseR lashed 16 mm beside baseL at B)
/// </code>
/// Ends A and D sit on anchor posts (pads under them, no earth between). The two canes meet at B side by side
/// (<see cref="LashDz"/>): two 14 mm canes cannot occupy the same node, so baseR runs 16 mm nearer the robot.
/// </para>
/// <para>
/// <b>Grounded start, no floating mount.</b> The arm starts with its L-grip clamped on anchor stake
/// <see cref="S1"/> (a 14 mm post planted on its own pad beside the stock feed). Every instant at least one gripper
/// is clamped on a real fixed object, and that gripper is the IK base. The L-grip is a closed clamp (rail + body on
/// one side, base tube on the other, jaws on both — the cane cannot leave it sideways at any opening), so it can only
/// come off a support by sliding over a free end: the stakes' tops. The R-grip (open slot) clamps the canes.
/// Walk: from S1, R clamps the double-fixed chord, L slides off S1 and onto stake <see cref="S2"/>, then R re-grips
/// the chord further along.
/// </para>
/// </summary>
public static class BambooTrussLayout
{
    /// <summary>Truss cane length (metres). All edge canes are this length.</summary>
    public const double StrutLength = 0.5;
    /// <summary>Design side length for A–B and B–D (equals <see cref="StrutLength"/> — canes meet at node B).</summary>
    public const double Side = 0.5;
    /// <summary>Notional upright B–C length (apex C is drawn in the footprint only — not built).</summary>
    public const double Height = 0.5;

    public const double JawClosedMm = 14;
    public const double JawOpenMm = 40;
    /// <summary>L-grip fully open while sliding along a stake (jaw plates 15 mm off the 14 mm stake).</summary>
    public const double LJawOpenMm = 45;

    /// <summary>Vertical plane of the truss (viewer Z).</summary>
    public const double PlaneZ = 0.11;
    /// <summary>baseR runs this far off <see cref="PlaneZ"/> (lashed beside baseL at B: 14 mm cane + 2 mm gap).</summary>
    public const double LashDz = -0.016;
    /// <summary>Ground-side node B height (metres).</summary>
    public const double FootY0 = 0.06;
    /// <summary>Half-width |x| of A and D.</summary>
    public static readonly double HalfSpan = Math.Sqrt(Side * Side - (Height / 2) * (Height / 2));
    /// <summary>Half-extent of Motus ground pads about anchors A and D (metres).</summary>
    public const double AnchorPadHalf = 0.06;
    /// <summary>Anchor posts under A and D (radius, and gap below the node so the cane end clears the post top).</summary>
    public const double AnchorPostRadius = 0.012;
    public const double AnchorPostGap = 0.02;

    /// <summary>A vertical anchor stake the L-grip clamps (slot axis = stake axis = viewer +Y).</summary>
    public sealed record Stake(string Name, double X, double Z, double ClampY, double Top, double HeadingDeg)
    {
        /// <summary>L-grip grip target on the stake at <see cref="ClampY"/>.</summary>
        public K.GripTarget Grip => new(X, ClampY, Z, 0, 1, 0, Name);
        public K.GripTarget At(double y) => new(X, y, Z, 0, 1, 0, Name);
    }
    public const double StakeRadius = 0.007;
    public const double StakePadHalf = 0.03;

    /// <summary>Start anchor stake beside the stock feed: L-grip clamped at y = 0.313, arm heading −66.2° (chosen by an offline reach + Motus clearance search).</summary>
    public static Stake S1 { get; } = new("stake-S1", 0.054, -0.092, 0.313, 0.343, -66.2);
    /// <summary>Second anchor stake (walk target for the L-grip, reached while the R-grip holds the chord).</summary>
    public static Stake S2 { get; } = new("stake-S2", -0.25, -0.20, 0.24, 0.27, 0);
    public static IReadOnlyList<Stake> Stakes { get; } = new[] { S1, S2 };

    /// <summary>Exact L-grip clamp frame on <see cref="S1"/> (viewer 4×4): the IK base at t = 0.</summary>
    public static double[] StartBase => K.StakeClampFrame(S1.X, S1.ClampY, S1.Z, S1.HeadingDeg);

    /// <summary>Stock feed rail: a narrow fixed rail the next cane rests on (top = cane bottom), so the open R jaws
    /// straddle it without touching the ground (a cane flat on the ground leaves only ~3 mm under the jaw plates).</summary>
    public const double RackTop = 0.043;
    public const double RackHalfZ = 0.004;
    public const double RackHalfX = 0.2;
    /// <summary>Feed rail centre (viewer x / z): in front of S1, between S1 and the truss plane.</summary>
    public const double FeedX = 0.054;
    public const double FeedZ = -0.130;
    /// <summary>
    /// Feed cane centre height: on the rail with a 0.5 mm seat gap. Motus reports a resting contact (distance 0) as a
    /// hit, so the feed cane is modelled 0.5 mm proud of the rail rather than whitelisting cane↔rail.
    /// </summary>
    public const double RackSeatGap = 0.0005;
    public const double StrutY = RackTop + 0.007 + RackSeatGap;

    /// <summary>Ground pads (viewer XZ centre + half extents): anchors, stakes and the stock feed — no earth elsewhere.</summary>
    public static IReadOnlyList<(string Name, double X, double Z, double HalfX, double HalfZ)> Pads { get; } =
    [
        ("ground-A", -HalfSpan, PlaneZ, AnchorPadHalf, AnchorPadHalf),
        ("ground-D", HalfSpan, PlaneZ + LashDz, AnchorPadHalf, AnchorPadHalf),
        ("ground-S1", S1.X, S1.Z, StakePadHalf, StakePadHalf),
        ("ground-S2", S2.X, S2.Z, StakePadHalf, StakePadHalf),
        ("ground-store", FeedX, FeedZ, 0.22, 0.02)
    ];

    /// <summary>Anchor posts (A, D) and stakes as vertical cylinders: (name, x, z, top, radius).</summary>
    public static IReadOnlyList<(string Name, double X, double Z, double Top, double Radius)> Posts { get; } =
    [
        ("post-A", -HalfSpan, PlaneZ, FootY0 + Height / 2 - AnchorPostGap, AnchorPostRadius),
        ("post-D", HalfSpan, PlaneZ + LashDz, FootY0 + Height / 2 - AnchorPostGap, AnchorPostRadius),
        (S1.Name, S1.X, S1.Z, S1.Top, StakeRadius),
        (S2.Name, S2.X, S2.Z, S2.Top, StakeRadius)
    ];

    /// <summary>Feed rails as boxes (viewer): name, centre x/z, half extents x/z, top y (bottom on the ground).</summary>
    public static IReadOnlyList<(string Name, double X, double Z, double HalfX, double HalfZ, double Top)> Racks { get; } =
    [
        ("store-rack", FeedX, FeedZ, RackHalfX, RackHalfZ, RackTop)
    ];

    static IReadOnlyList<CollisionObject>? _fixtures;
    /// <summary>Motus objects for <see cref="Posts"/> and <see cref="Racks"/> (built once; pads come from the cell).</summary>
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

    /// <summary>Notional sep / diag place poses (not built — see type summary).</summary>
    public static BambooIcdCell.StrutPlacement NotionalSep
    {
        get
        {
            var b = FootNodes[1]; var c = FootNodes[3];
            return AlongEdge((b.X, b.Y, b.Z), (c.X, c.Y, c.Z));
        }
    }

    public sealed record MemberDef(
        string Id,
        string Label,
        string Glyph,
        BambooIcdCell.StrutPlacement Store,
        BambooIcdCell.StrutPlacement Place,
        (double X, double Y, double Z) EdgeA,
        (double X, double Y, double Z) EdgeB);

    /// <summary>Stock feed: cane lying on the feed rail along +X (yaw 90), between the stake S1 and the truss plane.</summary>
    public static BambooIcdCell.StrutPlacement Store { get; } = new(FeedX, StrutY, FeedZ, 0, 90);

    /// <summary>Truss nodes: A/D anchors (on posts), B lower mid (baseR lashed beside it), C notional apex.</summary>
    public static readonly (string Name, double X, double Y, double Z)[] FootNodes =
    [
        ("A", -HalfSpan, FootY0 + Height / 2, PlaneZ),
        ("B", 0, FootY0, PlaneZ),
        ("D", HalfSpan, FootY0 + Height / 2, PlaneZ + LashDz),
        ("C", 0, FootY0 + Height, PlaneZ)
    ];

    public static IReadOnlyList<MemberDef> Members { get; } = BuildMembers();

    static IReadOnlyList<MemberDef> BuildMembers()
    {
        var A = (-HalfSpan, FootY0 + Height / 2, PlaneZ);
        var B = (0.0, FootY0, PlaneZ);
        var Bl = (0.0, FootY0, PlaneZ + LashDz);
        var D = (HalfSpan, FootY0 + Height / 2, PlaneZ + LashDz);
        var edges = new (string Id, string Label, string Glyph, (double X, double Y, double Z) P, (double X, double Y, double Z) Q)[]
        {
            ("baseL", "base left", "/", A, B),
            ("baseR", "base right", "_", Bl, D)
        };
        var list = new List<MemberDef>();
        foreach (var e in edges)
            list.Add(new MemberDef(e.Id, e.Label, e.Glyph, Store, AlongEdge(e.P, e.Q), e.P, e.Q));
        return list;
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

    /// <summary>Members fixed at both ends after <paramref name="placedCount"/> placements (walk rails).</summary>
    public static IReadOnlyList<string> WalkableMemberIdsAfter(int placedCount)
    {
        var placed = Members.Take(placedCount).Select(m => m.Id).ToHashSet(StringComparer.Ordinal);
        var walkable = new List<string>();
        if (placed.Contains("baseL") && placed.Contains("baseR"))
        {
            walkable.Add("baseL");
            walkable.Add("baseR");
        }
        return walkable;
    }

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
        motusLimit = "L-grip clamped on stake S1 (grounded base); R builds baseL+baseR; walk: R clamps chord, L slides S1→S2, R re-grips chord"
    };

    public static object StoreDescription() => new
    {
        slots = new[] { new { id = "feed", x = Store.X, y = Store.Y, z = Store.Z, pitch = Store.PitchDeg, yaw = Store.YawDeg } }
    };
}
