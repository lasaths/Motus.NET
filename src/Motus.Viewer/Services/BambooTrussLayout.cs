namespace Motus.Viewer.Services;

/// <summary>
/// Ground footprint and store/truss member poses for the /bamboo-truss variation.
/// <para>
/// Truss: two equilateral triangles A–B–C and B–D–C lying flat on the ground and sharing the middle
/// edge B–C (a 60° rhombus, five members). B–C runs along viewer Z through x = 0; A and D sit left and
/// right, so A–B / B–D are the bottom edges and A–C / D–C the diagonals up to the apex C.
/// Every member is a horizontal cane lying on the ground along its edge, centred on the edge midpoint,
/// ending <see cref="NodeGap"/> short of each node so neighbouring canes never touch.
/// </para>
/// <para>
/// Material store: a single feed at the proven ICD pick pose (radial cane at x = 0 beyond apex C, in line
/// with the shared edge). One cane is presented per cycle; waiting stock is not in the Motus scene.
/// </para>
/// <para>
/// Reach: the hung arm grips a ground cane straight down (pitch 0, θ5 rolling the jaw onto the cane)
/// only within ≈0.095 m of the pole axis, and at most ≈0.128 m when fully stretched. A flat two-triangle
/// truss of the 0.30 m ICD strut needs ≥0.13 m (rhombus centre→edge = 0.433·side), so truss canes are
/// shorter (<see cref="StrutLength"/>); every member centre lies inside the straight-down reach.
/// </para>
/// </summary>
public static class BambooTrussLayout
{
    /// <summary>Truss cane length (metres). The cell for /bamboo-truss is loaded with this strut length.</summary>
    public const double StrutLength = 0.10;
    /// <summary>Clear distance from each node to the end of every cane meeting there (metres).</summary>
    public const double NodeGap = 0.02;
    /// <summary>Rhombus side = edge length A–B = B–C = …</summary>
    public const double Side = StrutLength + 2 * NodeGap;

    public const double StrutY = 0.00778;
    public const double StoreApproachDy = 0.04;
    /// <summary>Hold plane height above the release (cane centre) plane.</summary>
    public const double PlaceHoldDy = 0.02;
    public const double JawClosedMm = 14;
    public const double JawOpenMm = 40;

    /// <summary>Centre of the rhombus (midpoint of B–C) along viewer Z; apex C stays short of the store cane.</summary>
    public const double CentreZ = -0.030;
    /// <summary>Node B (bottom of the shared edge) Z, and the shared edge length B→C along +Z.</summary>
    public const double FootZ0 = CentreZ - Side / 2;
    public const double FootH = Side;
    /// <summary>Half the long diagonal A–D (|x| of A and D).</summary>
    public static readonly double HalfSpan = Side * Math.Sqrt(3) / 2;

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

    /// <summary>Footprint nodes in ground XZ (y = 0): B bottom, C apex, A left, D right; shared edge B–C.</summary>
    public static readonly (string Name, double X, double Z)[] FootNodes =
    [
        ("A", -HalfSpan, CentreZ),
        ("B", 0, FootZ0),
        ("D", HalfSpan, CentreZ),
        ("C", 0, FootZ0 + FootH)
    ];

    public static IReadOnlyList<MemberDef> Members { get; } = BuildMembers();

    static IReadOnlyList<MemberDef> BuildMembers()
    {
        var A = (-HalfSpan, CentreZ);
        var B = (0.0, FootZ0);
        var D = (HalfSpan, CentreZ);
        var C = (0.0, FootZ0 + FootH);
        // Build order: bottoms, shared edge, then diagonals.
        var edges = new (string Id, string Label, (double X, double Z) P, (double X, double Z) Q)[]
        {
            ("baseL", "base left", A, B),
            ("baseR", "base right", B, D),
            ("sep", "shared edge", B, C),
            ("diagL", "diag left", A, C),
            ("diagR", "diag right", D, C)
        };
        var list = new List<MemberDef>();
        foreach (var e in edges)
            list.Add(new MemberDef(e.Id, e.Label, Store, LyingOn(e.P, e.Q), e.P, e.Q));
        return list;
    }

    /// <summary>Horizontal cane lying on the ground along edge P→Q, centred on the edge midpoint.</summary>
    public static BambooIcdCell.StrutPlacement LyingOn((double X, double Z) p, (double X, double Z) q)
    {
        var mx = (p.X + q.X) / 2;
        var mz = (p.Z + q.Z) / 2;
        // Placement axis at pitch 0 is Ry(yaw)·(0,0,1) = (sin yaw, 0, cos yaw). Canes are symmetric, so fold
        // the yaw into (−90°, 90°] to keep the jaw roll small.
        var yaw = Math.Atan2(q.X - p.X, q.Z - p.Z) * 180 / Math.PI;
        if (yaw > 90) yaw -= 180;
        if (yaw <= -90) yaw += 180;
        return new BambooIcdCell.StrutPlacement(mx, StrutY, mz, 0, yaw);
    }

    public static object FootprintDescription() => new
    {
        nodes = FootNodes.Select(n => new { name = n.Name, x = n.X, z = n.Z }),
        edges = Members.Select(m => new { id = m.Id, ax = m.EdgeA.X, az = m.EdgeA.Z, bx = m.EdgeB.X, bz = m.EdgeB.Z })
    };

    /// <summary>The single feed slot (one cane presented per cycle).</summary>
    public static object StoreDescription() => new
    {
        slots = new[] { new { id = "feed", x = Store.X, y = Store.Y, z = Store.Z, pitch = Store.PitchDeg, yaw = Store.YawDeg } }
    };
}
