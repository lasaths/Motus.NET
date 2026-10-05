namespace Motus.Viewer.Services;

/// <summary>
/// Ground footprint and store/truss member poses for the /bamboo-truss variation.
/// Layout: two triangles side-by-side along X on the ground (shared middle edge B–C),
/// material store = rack of five horizontal struts near the arm; play picks each store
/// strut and places it as a vertical member at the corresponding edge midpoint.
/// </summary>
public static class BambooTrussLayout
{
    public const double StrutY = 0.00778;
    public const double StoreZ = 0.07;
    public const double StoreApproachDy = 0.04;
    public const double PlaceY = 0.1506;
    public const double PlaceHoldDy = 0.06;
    public const double JawClosedMm = 14;
    public const double JawOpenMm = 40;

    /// <summary>Half-width of the bottom edge A–D (metres). Full bottom span = 2·HalfSpan.</summary>
    public const double HalfSpan = 0.06;
    /// <summary>Bottom edge Z and apex offset along +Z for the ground footprint.</summary>
    public const double FootZ0 = 0.14;
    public const double FootH = 0.10;

    public sealed record MemberDef(
        string Id,
        string Label,
        BambooIcdCell.StrutPlacement Store,
        BambooIcdCell.StrutPlacement Place,
        (double X, double Z) EdgeA,
        (double X, double Z) EdgeB);

    /// <summary>Footprint nodes in ground XZ (y = 0): A—B—D along the bottom, C apex, shared edge B–C.</summary>
    public static readonly (string Name, double X, double Z)[] FootNodes =
    [
        ("A", -HalfSpan, FootZ0),
        ("B", 0, FootZ0),
        ("D", HalfSpan, FootZ0),
        ("C", 0, FootZ0 + FootH)
    ];

    public static IReadOnlyList<MemberDef> Members { get; } = BuildMembers();

    static IReadOnlyList<MemberDef> BuildMembers()
    {
        var A = (-HalfSpan, FootZ0);
        var B = (0.0, FootZ0);
        var D = (HalfSpan, FootZ0);
        var C = (0.0, FootZ0 + FootH);
        // Edge midpoints → vertical place centres (same XZ, PlaceY, pitch −90°).
        var edges = new (string Id, string Label, (double X, double Z) P, (double X, double Z) Q)[]
        {
            ("baseL", "base left", A, B),
            ("baseR", "base right", B, D),
            ("sep", "shared upright", B, C),
            ("diagL", "diag left", A, C),
            ("diagR", "diag right", D, C)
        };

        // First slot = proven ICD pick. Others: parallel rack at z=0.05, |x|∈{0.05,0.06}
        // (reachable, beyond open-jaw half-width so baseL grasp stays Motus-clear).
        var storeSlots = new BambooIcdCell.StrutPlacement[]
        {
            new(BambooIcdMotion.Pick.X, BambooIcdMotion.Pick.Y, BambooIcdMotion.Pick.Z, 0), // baseL
            new(-0.06, StrutY, 0.05, 0), // baseR
            new(-0.05, StrutY, 0.05, 0), // sep
            new(0.05, StrutY, 0.05, 0),  // diagL
            new(0.06, StrutY, 0.05, 0),  // diagR
        };
        var placeSlots = new BambooIcdCell.StrutPlacement[]
        {
            // First place matches the proven default upright place (clear under Motus).
            new(BambooIcdMotion.Place.X, BambooIcdMotion.Place.Y, BambooIcdMotion.Place.Z, -90),
            new(-0.03, PlaceY, FootZ0, -90),
            new(0.03, PlaceY, FootZ0, -90),
            new(-0.03, PlaceY, FootZ0 + FootH * 0.5, -90),
            new(0.03, PlaceY, FootZ0 + FootH * 0.5, -90),
        };
        var list = new List<MemberDef>();
        for (var i = 0; i < edges.Length; i++)
        {
            var e = edges[i];
            list.Add(new MemberDef(
                e.Id,
                e.Label,
                storeSlots[i],
                placeSlots[i],
                e.P,
                e.Q));
        }
        return list;
    }

    public static object FootprintDescription() => new
    {
        nodes = FootNodes.Select(n => new { name = n.Name, x = n.X, z = n.Z }),
        edges = Members.Select(m => new { id = m.Id, ax = m.EdgeA.X, az = m.EdgeA.Z, bx = m.EdgeB.X, bz = m.EdgeB.Z })
    };

    public static object StoreDescription() => new
    {
        slots = Members.Select(m => new { id = m.Id, x = m.Store.X, y = m.Store.Y, z = m.Store.Z, pitch = m.Store.PitchDeg })
    };
}
