namespace Motus.Viewer.Services;

/// <summary>
/// Store/truss member poses for the /bamboo-truss variation.
/// <para>
/// Bridge edge cane in the vertical XY plane (constant <see cref="PlaneZ"/>) at
/// <see cref="StrutLength"/> = 0.5 m — Motus-reachable stand-in for ASCII <c>/_\-/_\</c>:
/// <code>
///    A--B        (baseL / )     D = far anchor (pad only)
/// </code>
/// Anchored at A (and pad at D with no earth between). Stock lays flat next to A.
/// </para>
/// <para>
/// <b>Motus limit (0.5 m edge canes, no horizontal hang rod):</b> the ICD arm hangs downward from its
/// L-grip start dock (no drawn support pole). After planting baseL, every Motus-checked empty-hand
/// return to the store feed collides (<c>R-grip rail ↔ placed-baseL</c>) — the 0.5 m chord occupies
/// the dock approach corridor. On-cane WalkHang is also too low to seat upright B–C or diagonals
/// A–C / D–C. Largest Motus-clear 0.5 m edge set: <b>baseL only</b>. baseR, sep, diags, and
/// five-triangle Warren (~1.5 m / 11 edges) leave Motus under the no-hang-rod rule. /bamboo stays 0.30 m.
/// </para>
/// <para>
/// Build order: baseL <c>/</c> only. Walk rails require a second both-ends-fixed cane (not Motus-clear here).
/// </para>
/// </summary>
public static class BambooTrussLayout
{
    /// <summary>Truss cane length (metres). All edge canes are this length.</summary>
    public const double StrutLength = 0.5;
    /// <summary>Clearance from each outer-edge node to the cane end on AB/BD (metres).</summary>
    public const double NodeGap = 0.03;
    /// <summary>Design side length for A–B and B–D (equals <see cref="StrutLength"/> — canes meet at node B).</summary>
    public const double Side = 0.5;
    /// <summary>
    /// Notional upright B–C length for the Motus-unreachable apex (documented limit only — not placed).
    /// Kept equal to <see cref="StrutLength"/> so FootNodes sketch the intended two-triangle bay.
    /// </summary>
    public const double Height = 0.5;

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

    /// <summary>
    /// Start-dock hang (viewer). Same coordinates as the legacy support-pole hang so ICD IK/Home match,
    /// but /bamboo-truss draws no horizontal hang rod — only the L-grip mount at this dock. After the
    /// lower chord is fixed the path brachiates onto <see cref="WalkHang"/>.
    /// </summary>
    public static BambooIcdMotion.HangPose StartDock { get; } = BambooIcdMotion.HangPose.Pole;

    /// <summary>L-grip clamp offset from a walk-rail cane axis (metres).</summary>
    public const double WalkHangOffset = 0.03;
    /// <summary>
    /// Walk-rail hang on the planted lower chord near B (both ends fixed: A/D anchors + B junction).
    /// Offset beside the cane so Home stays Motus-clear (L-grip clamp, not through the member).
    /// </summary>
    public static BambooIcdMotion.HangPose WalkHang
    {
        get
        {
            // Near B on baseL: parametric t≈0.85 along A→B.
            var a = FootNodes[0]; var b = FootNodes[1];
            const double t = 0.85;
            var x = a.X + (b.X - a.X) * t;
            var y = a.Y + (b.Y - a.Y) * t;
            return new BambooIcdMotion.HangPose(x + WalkHangOffset, y + WalkHangOffset, PlaneZ - WalkHangOffset);
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

    /// <summary>
    /// Stock feed next to the span (proven Motus pick band): cane along +X (yaw 90) at the ICD
    /// pick height, offset −Z from the truss plane so Lift/Pitch clear placed edge members. Visually
    /// beside anchor A’s side of the cell.
    /// </summary>
    public static BambooIcdCell.StrutPlacement Store { get; } =
        new(0.0, BambooIcdMotion.Pick.Y, 0.05, 0, 90);

    /// <summary>
    /// Truss nodes: A/D outer anchors, B lower mid; C is the notional apex of the Motus-unreachable
    /// first bay (drawn in the footprint only).
    /// </summary>
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
        // Motus-clear 0.5 m edge prefix: baseL only (see type summary).
        var edges = new (string Id, string Label, string Glyph, (double X, double Y, double Z) P, (double X, double Y, double Z) Q)[]
        {
            ("baseL", "base left", "/", A, B)
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
        // baseR leaves Motus; no both-ends-fixed walk rail in the Motus-clear prefix.
        _ = placed;
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
        anchors = AnchorPads.Select(a => new { x = a.X, z = a.Z, half = AnchorPadHalf }),
        // Intended but Motus-unreachable at 0.5 m without a hang rod (see type summary).
        motusLimit = "baseL only at 0.5 m; baseR re-approach / sep / diags / five-triangle leave Motus without hang rod"
    };

    public static object StoreDescription() => new
    {
        slots = new[] { new { id = "feed", x = Store.X, y = Store.Y, z = Store.Z, pitch = Store.PitchDeg, yaw = Store.YawDeg } }
    };
}
