namespace Motus.Viewer.Services;

/// <summary>
/// Store/truss member poses for the /bamboo-truss variation.
/// <para>
/// Bridge span in the vertical XY plane (constant <see cref="PlaneZ"/>), silhouette of two
/// triangles sharing upright B–C — the Motus-reachable stand-in for ASCII <c>/_\-/_\</c>:
/// <code>
///       C
///      /|\
///     / | \
///    A--B--D
/// </code>
/// Anchored at outer bottom-ish nodes A and D (and B where the shared upright meets the lower chord).
/// True five-triangle Warren <c>/_\-/_\</c> at 0.5 m (11 edge members, span ≈ 1.5 m) leaves Motus
/// jaw-axis reach even with brachiation onto double-fixed rails — largest Motus-clear edge prefix
/// found: first <c>/_\</c> only at ≈ 0.22 m side; BD and beyond unreachable. This cell keeps a
/// Motus-clear full path for the two-triangle / five-edge span at <see cref="StrutLength"/>.
/// Not five freestanding upright poles and not a flat ground diamond. /bamboo stays 0.30 m.
/// </para>
/// <para>
/// Build order (bridge-game incremental): baseL <c>/</c>, baseR <c>_</c> along the lower chord,
/// sep <c>\</c> (shared upright closes the first bay), then diagL / diagR.
/// Walk/brachiate only on members fixed at both ends.
/// </para>
/// <para>
/// Material store: stock laying flat next to the first anchor A.
/// </para>
/// </summary>
public static class BambooTrussLayout
{
    /// <summary>Truss cane length (metres). Prefer 0.5; Motus jaw-axis reach caps the Motus-clear edge span here.</summary>
    public const double StrutLength = 0.08;
    /// <summary>Clearance from each outer-edge node to the cane end on AB/BD/AC/DC (metres).</summary>
    public const double NodeGap = 0.03;
    /// <summary>Rhombus side length for A–B, B–D, A–C, D–C.</summary>
    public const double Side = 0.14;
    /// <summary>Shared upright B–C length (metres); equals <see cref="StrutLength"/> so the sep cane spans B–C.</summary>
    public const double Height = 0.08;

    public const double StrutY = 0.00778;
    public const double StoreApproachDy = 0.04;
    /// <summary>Hold offset along +Y above the release (cane centre).</summary>
    public const double PlaceHoldDy = 0.012;
    public const double JawClosedMm = 14;
    public const double JawOpenMm = 40;

    /// <summary>Vertical plane of the truss (viewer Z).</summary>
    public const double PlaneZ = 0.11;
    /// <summary>Ground-side node B height (metres). Slightly elevated so inclined base canes clear the slab.</summary>
    public const double FootY0 = 0.06;
    /// <summary>Half-width |x| of A and D.</summary>
    public static readonly double HalfSpan = Math.Sqrt(Side * Side - (Height / 2) * (Height / 2));
    /// <summary>Half-extent of Motus ground pads about anchors A and D (metres).</summary>
    public const double AnchorPadHalf = 0.06;

    public sealed record MemberDef(
        string Id,
        string Label,
        string Glyph,
        BambooIcdCell.StrutPlacement Store,
        BambooIcdCell.StrutPlacement Place,
        (double X, double Y, double Z) EdgeA,
        (double X, double Y, double Z) EdgeB);

    /// <summary>
    /// Stock feed next to the span (proven Motus pick band): cane along +X (yaw 90) at the ICD
    /// pick height, offset −Z from the truss plane so Lift/Pitch clear placed edge members. Visually
    /// beside anchor A’s side of the cell.
    /// </summary>
    public static BambooIcdCell.StrutPlacement Store { get; } =
        new(0.0, BambooIcdMotion.Pick.Y, 0.05, 0, 90);

    /// <summary>Truss nodes: A/D outer anchors, B lower mid, C apex; shared edge B–C vertical.</summary>
    public static readonly (string Name, double X, double Y, double Z)[] FootNodes =
    [
        ("A", -HalfSpan, FootY0 + Height / 2, PlaneZ),
        ("B", 0, FootY0, PlaneZ),
        ("D", HalfSpan, FootY0 + Height / 2, PlaneZ),
        ("C", 0, FootY0 + Height, PlaneZ)
    ];

    /// <summary>Motus ground pad centres (viewer XZ) at the two outer anchors — no earth between.</summary>
    public static readonly (double X, double Z)[] AnchorPads =
    [
        (-HalfSpan, PlaneZ),
        (HalfSpan, PlaneZ)
    ];

    public static IReadOnlyList<MemberDef> Members { get; } = BuildMembers();

    static IReadOnlyList<MemberDef> BuildMembers()
    {
        var A = (-HalfSpan, FootY0 + Height / 2, PlaneZ);
        var B = (0.0, FootY0, PlaneZ);
        var D = (HalfSpan, FootY0 + Height / 2, PlaneZ);
        var C = (0.0, FootY0 + Height, PlaneZ);
        // Bridge-game order: / along left base, _ along right base, \ shared upright, then closing diags.
        var edges = new (string Id, string Label, string Glyph, (double X, double Y, double Z) P, (double X, double Y, double Z) Q)[]
        {
            ("baseL", "base left", "/", A, B),
            ("baseR", "base right", "_", B, D),
            ("sep", "shared upright", "\\", B, C),
            ("diagL", "diag left", "/", A, C),
            ("diagR", "diag right", "\\", D, C)
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
        // After baseL+baseR the lower chord is fixed at A,B,D (A/D anchors, B junction).
        // After sep, B–C is fixed. Diagonals become walkable once both ends have ≥2 incident members.
        var placed = Members.Take(placedCount).Select(m => m.Id).ToHashSet(StringComparer.Ordinal);
        var walkable = new List<string>();
        if (placed.Contains("baseL") && placed.Contains("baseR"))
        {
            walkable.Add("baseL");
            walkable.Add("baseR");
        }
        if (placed.Contains("sep") && placed.Contains("baseL") && placed.Contains("baseR"))
            walkable.Add("sep");
        if (placed.Contains("diagL") && placed.Contains("baseL") && placed.Contains("sep"))
            walkable.Add("diagL");
        if (placed.Contains("diagR") && placed.Contains("baseR") && placed.Contains("sep"))
            walkable.Add("diagR");
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
        anchors = AnchorPads.Select(a => new { x = a.X, z = a.Z, half = AnchorPadHalf })
    };

    public static object StoreDescription() => new
    {
        slots = new[] { new { id = "feed", x = Store.X, y = Store.Y, z = Store.Z, pitch = Store.PitchDeg, yaw = Store.YawDeg } }
    };
}
