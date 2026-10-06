namespace Motus.Viewer.Services;

/// <summary>
/// Ground footprint and store/truss member poses for the /bamboo-truss variation.
/// <para>
/// Truss: two equilateral triangles A–B–C and B–D–C on a compact ground footprint, sharing edge B–C.
/// Each member is a <see cref="StrutLength"/> vertical cane planted at the edge midpoint (pitch −90°).
/// Cane length is 0.5 m — beyond flat-ground hung reach — so the robot brachiates: after the first
/// upright is planted it is treated as fixed at both ends (ground stake + pole sleeve where the cane
/// crosses the support pole at y = <see cref="BambooIcdMotion.PoleY"/>), and later cycles hang from it.
/// </para>
/// <para>
/// Material store: a single feed at the proven ICD pick pose. One cane is presented per cycle.
/// </para>
/// </summary>
public static class BambooTrussLayout
{
    /// <summary>Truss cane length (metres). The /bamboo-truss cell is loaded with this strut length.</summary>
    public const double StrutLength = 0.5;
    /// <summary>Ground clearance of a cane resting / planted centre when horizontal; upright centres use <see cref="PlaceY"/>.</summary>
    public const double StrutY = 0.00778;
    /// <summary>Centre height of a planted upright (bottom ≈ ground, top ≈ StrutLength).</summary>
    public const double PlaceY = 0.28;
    public const double StoreApproachDy = 0.04;
    public const double PlaceHoldDy = 0.025;
    public const double JawClosedMm = 14;
    public const double JawOpenMm = 40;

    /// <summary>Half-width of bottom edge A–D (metres). Compact so uprights stay Motus-reachable.</summary>
    public const double HalfSpan = 0.06;
    public const double FootZ0 = 0.20;
    public const double FootH = 0.06;

    /// <summary>Hang height on a walkable upright (below the pole so the arm still hangs downward).</summary>
    public const double WalkHangY = 0.32;

    public sealed record MemberDef(
        string Id,
        string Label,
        BambooIcdCell.StrutPlacement Store,
        BambooIcdCell.StrutPlacement Place,
        (double X, double Z) EdgeA,
        (double X, double Z) EdgeB);

    /// <summary>Single feed pose (proven ICD pick): radial cane at x = 0 on the ground.</summary>
    public static BambooIcdCell.StrutPlacement Store { get; } =
        new(BambooIcdMotion.Pick.X, BambooIcdMotion.Pick.Y, BambooIcdMotion.Pick.Z, 0);

    public static readonly (string Name, double X, double Z)[] FootNodes =
    [
        ("A", -HalfSpan, FootZ0),
        ("B", 0, FootZ0),
        ("D", HalfSpan, FootZ0),
        ("C", 0, FootZ0 + FootH)
    ];

    public static IReadOnlyList<MemberDef> Members { get; } = BuildMembers();

    /// <summary>
    /// Hang pose used while placing each member. Index matches <see cref="Members"/>.
    /// Cycle 0 hangs from the support pole; after baseL is planted (ground + pole sleeve) later
    /// cycles hang from that upright — the only walk rail that is fixed on both ends at that time.
    /// </summary>
    /// <summary>Place hang: always the pole. Brachiation along baseL is a dedicated Motus-clear segment after cycle 0.</summary>
    public static BambooIcdMotion.HangPose HangForMemberIndex(int index) => BambooIcdMotion.HangPose.Pole;

    /// <summary>Walk-rail hang on the planted baseL upright (ground + pole sleeve).</summary>
    public static BambooIcdMotion.HangPose WalkHang =>
        new(Members[0].Place.X, WalkHangY, Members[0].Place.Z);

    static IReadOnlyList<MemberDef> BuildMembers()
    {
        var A = (-HalfSpan, FootZ0);
        var B = (0.0, FootZ0);
        var D = (HalfSpan, FootZ0);
        var C = (0.0, FootZ0 + FootH);
        // Edge midpoints → vertical place centres. First matches proven ICD upright place XZ band.
        // All uprights keep |x| ≥ 0.05 so they clear the 0.5 m store cane on the Z-axis feed.
        // Later places are chosen inside the walk-hang reach from baseL at WalkHangY.
        var edges = new (string Id, string Label, (double X, double Z) P, (double X, double Z) Q, double Px, double Pz)[]
        {
            // Compact Motus-reachable uprights; |x|≥0.05 clears the 0.5 m store feed.
            // Azimuth about the pole (deg): baseL −22, sep −10, diagL +10, diagR +23, baseR +42.
            // baseR sits outboard so diagR (placed later) is not in its radial shadow: the arm reaching
            // diagR never has to cross a planted cane. WalkHang is Members[0] (baseL) once
            // ground-staked + pole-sleeved (both ends fixed).
            ("baseL", "base left", A, B, -0.08, 0.20),
            ("baseR", "base right", B, D, 0.16, 0.18),
            ("sep", "shared upright", B, C, -0.05, 0.28),
            ("diagL", "diag left", A, C, 0.05, 0.28),
            ("diagR", "diag right", D, C, 0.11, 0.26)
        };
        var list = new List<MemberDef>();
        foreach (var e in edges)
            list.Add(new MemberDef(e.Id, e.Label, Store, new BambooIcdCell.StrutPlacement(e.Px, PlaceY, e.Pz, -90), e.P, e.Q));
        return list;
    }

    public static object FootprintDescription() => new
    {
        nodes = FootNodes.Select(n => new { name = n.Name, x = n.X, z = n.Z }),
        edges = Members.Select(m => new { id = m.Id, ax = m.EdgeA.X, az = m.EdgeA.Z, bx = m.EdgeB.X, bz = m.EdgeB.Z })
    };

    public static object StoreDescription() => new
    {
        slots = new[] { new { id = "feed", x = Store.X, y = Store.Y, z = Store.Z, pitch = Store.PitchDeg, yaw = Store.YawDeg } }
    };
}
