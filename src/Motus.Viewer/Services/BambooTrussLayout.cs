namespace Motus.Viewer.Services;

/// <summary>
/// Store/truss member poses for the /bamboo-truss variation.
/// <para>
/// Bridge lower chord in the vertical XY plane (constant <see cref="PlaneZ"/>) at
/// <see cref="StrutLength"/> = 0.5 m — Motus-reachable stand-in for ASCII <c>/_\-/_\</c>:
/// <code>
///    A--B--D     (baseL /  +  baseR _ )
/// </code>
/// Anchored at A and D (no earth between). Stock lays flat next to the span. No hang rod
/// (<c>IncludePole=false</c>); start dock at <see cref="StartDock"/>, then Motus-checked
/// re-approach after baseL (hang-side / Motus-filtered vias).
/// </para>
/// <para>
/// After baseL+baseR the lower chord is both-ends-fixed; the auto path brachiates onto
/// <see cref="WalkHang"/> (outboard L-grip at A — Motus-clear transfer from the dock).
/// <b>Motus limit:</b> sep / diags / five-triangle stay jaw-axis unreachable from every Motus-clear
/// on-chord hang (arm hangs down; apex mids sit above / too far laterally). Motus-clear members:
/// baseL+baseR only. /bamboo stays 0.30 m.
/// </para>
/// <para>
/// Build order: baseL <c>/</c> → baseR <c>_</c> → brachiate onto the double-fixed chord.
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
    /// <summary>Elevated hang Y used while transferring from <see cref="StartDock"/> onto the chord.</summary>
    public const double BrachiateViaY = 0.42;
    /// <summary>Hang-side Z for the elevated transfer vias (metres).</summary>
    public const double BrachiateViaZ = -0.05;
    /// <summary>
    /// Walk-rail hang on the planted lower chord at A (both ends fixed: A/D anchors + B junction).
    /// Outboard −X L-grip so Home is Motus-clear and the dock→chord hang transfer stays Motus-clear
    /// (inboard / −Z offsets leave a Motus collision band on the transfer).
    /// </summary>
    public static BambooIcdMotion.HangPose WalkHang
    {
        get
        {
            var a = FootNodes[0];
            return new BambooIcdMotion.HangPose(a.X - WalkHangOffset, a.Y, PlaneZ);
        }
    }

    /// <summary>Outboard L-grip on baseL at parametric t along A→B (0 at A, 1 at B).</summary>
    public static BambooIcdMotion.HangPose WalkHangAt(double t)
    {
        var a = FootNodes[0]; var b = FootNodes[1];
        t = Math.Clamp(t, 0, 1);
        var x = a.X + (b.X - a.X) * t;
        var y = a.Y + (b.Y - a.Y) * t;
        return new BambooIcdMotion.HangPose(x - WalkHangOffset, y, PlaneZ);
    }

    /// <summary>
    /// Motus-clear hang waypoints for brachiating from <see cref="StartDock"/> onto
    /// <see cref="WalkHang"/> and a short along-chord walk (Home posture throughout).
    /// </summary>
    public static IReadOnlyList<BambooIcdMotion.HangPose> BrachiateHangPath
    {
        get
        {
            var w = WalkHang;
            return
            [
                StartDock,
                new BambooIcdMotion.HangPose(0, BrachiateViaY, BrachiateViaZ),
                new BambooIcdMotion.HangPose(w.X, BrachiateViaY, BrachiateViaZ),
                w,
                WalkHangAt(0.5),
                w
            ];
        }
    }

    /// <summary>
    /// Phase label for each <see cref="BrachiateHangPath"/> waypoint (motion keys are
    /// <c>brachiate:&lt;label&gt;</c>): Home at the dock → Rise → Traverse (hang-side, above the span) →
    /// Grip (outboard L-grip on baseL at A) → Walk (along baseL) → Return (back to the A grip).
    /// </summary>
    public static IReadOnlyList<string> BrachiatePhaseLabels { get; } =
        ["Home", "Rise", "Traverse", "Grip", "Walk", "Return"];

    /// <summary>Notional sep / diag place poses (not in <see cref="Members"/> — Motus-unreachable from WalkHang).</summary>
    public static BambooIcdCell.StrutPlacement NotionalSep
    {
        get
        {
            var b = FootNodes[1]; var c = FootNodes[3];
            return AlongEdge((b.X, b.Y, b.Z), (c.X, c.Y, c.Z));
        }
    }
    public static BambooIcdCell.StrutPlacement NotionalDiagL
    {
        get
        {
            var a = FootNodes[0]; var c = FootNodes[3];
            return AlongEdge((a.X, a.Y, a.Z), (c.X, c.Y, c.Z));
        }
    }
    public static BambooIcdCell.StrutPlacement NotionalDiagR
    {
        get
        {
            var d = FootNodes[2]; var c = FootNodes[3];
            return AlongEdge((d.X, d.Y, d.Z), (c.X, c.Y, c.Z));
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
        // Hang-side of PlaneZ: Motus-reachable pick; clears planted baseL on re-approach better than z=0.05.
        new(0.0, BambooIcdMotion.Pick.Y, -0.02, 0, 90);

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
        var D = (HalfSpan, FootY0 + Height / 2, PlaneZ);
        // Motus-clear 0.5 m lower-chord prefix (sep/diags Motus-unreachable from WalkHang — see type summary).
        var edges = new (string Id, string Label, string Glyph, (double X, double Y, double Z) P, (double X, double Y, double Z) Q)[]
        {
            ("baseL", "base left", "/", A, B),
            ("baseR", "base right", "_", B, D)
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
        anchors = AnchorPads.Select(a => new { x = a.X, z = a.Z, half = AnchorPadHalf }),
        // Intended but Motus-unreachable at 0.5 m without a hang rod (see type summary).
        motusLimit = "lower chord baseL+baseR Motus-clear (no hang rod); brachiate onto outboard WalkHang at A; sep/diags/five-triangle jaw-axis unreachable from on-chord hangs"
    };

    public static object StoreDescription() => new
    {
        slots = new[] { new { id = "feed", x = Store.X, y = Store.Y, z = Store.Z, pitch = Store.PitchDeg, yaw = Store.YawDeg } }
    };
}
