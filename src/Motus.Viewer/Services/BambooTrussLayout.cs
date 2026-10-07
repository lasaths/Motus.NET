namespace Motus.Viewer.Services;

/// <summary>
/// Store/truss member poses for the /bamboo-truss variation.
/// <para>
/// Truss: two isosceles triangles A–B–C and B–D–C standing in the vertical XY plane (constant Z),
/// sharing edge B–C. Nodes A and D sit left/right of the shared upright; B is the lower node and C
/// the upper apex — a diamond / kite in the vertical plane standing on B, not five freestanding upright poles. Each member
/// is a true edge cane of length <see cref="StrutLength"/> centred on its edge midpoint
/// (pitch/yaw along the edge), ending short of each node so neighbouring canes never touch.
/// </para>
/// <para>
/// Scale: cane length stays <see cref="StrutLength"/> (0.08 m). A 0.5 m vertical edge truss cannot
/// keep all five midpoints Motus-graspable from the pole hang (inclined yaw±90 members leave the
/// ~0.04–0.16 m Y / |x|≈0.03–0.10 m band where jaw-axis IK succeeds); walk on a double-fixed rail
/// does not enlarge that grasp envelope enough for five true edge centres. /bamboo keeps 0.30 m.
/// </para>
/// <para>
/// Material store: single feed at the proven ICD pick pose. One cane per cycle.
/// </para>
/// </summary>
public static class BambooTrussLayout
{
    /// <summary>Truss cane length (metres). The /bamboo-truss cell is loaded with this strut length.</summary>
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

    public sealed record MemberDef(
        string Id,
        string Label,
        BambooIcdCell.StrutPlacement Store,
        BambooIcdCell.StrutPlacement Place,
        (double X, double Y, double Z) EdgeA,
        (double X, double Y, double Z) EdgeB);

    /// <summary>Single feed pose (proven ICD pick): radial cane at x = 0 on the ground.</summary>
    /// <summary>Feed: offset +X / −Z from the vertical truss plane with cane along +X (yaw 90)
    /// so Lift/Pitch stay Motus-clear of placed edge members. Same proven pick height as /bamboo.</summary>
    public static BambooIcdCell.StrutPlacement Store { get; } =
        new(0.0, BambooIcdMotion.Pick.Y, 0.05, 0, 90);

    /// <summary>Truss nodes in viewer XYZ: B bottom, C top, A left, D right; shared edge B–C vertical.</summary>
    public static readonly (string Name, double X, double Y, double Z)[] FootNodes =
    [
        ("A", -HalfSpan, FootY0 + Height / 2, PlaneZ),
        ("B", 0, FootY0, PlaneZ),
        ("D", HalfSpan, FootY0 + Height / 2, PlaneZ),
        ("C", 0, FootY0 + Height, PlaneZ)
    ];

    public static IReadOnlyList<MemberDef> Members { get; } = BuildMembers();

    static IReadOnlyList<MemberDef> BuildMembers()
    {
        var A = (-HalfSpan, FootY0 + Height / 2, PlaneZ);
        var B = (0.0, FootY0, PlaneZ);
        var D = (HalfSpan, FootY0 + Height / 2, PlaneZ);
        var C = (0.0, FootY0 + Height, PlaneZ);
        var edges = new (string Id, string Label, (double X, double Y, double Z) P, (double X, double Y, double Z) Q)[]
        {
            ("baseL", "base left", A, B),
            ("baseR", "base right", B, D),
            ("sep", "shared upright", B, C),
            ("diagL", "diag left", A, C),
            ("diagR", "diag right", D, C)
        };
        var list = new List<MemberDef>();
        foreach (var e in edges)
            list.Add(new MemberDef(e.Id, e.Label, Store, AlongEdge(e.P, e.Q), e.P, e.Q));
        return list;
    }

    /// <summary>
    /// Cane of <see cref="StrutLength"/> centred on edge P→Q, axis along the edge.
    /// Pitch/yaw chosen so the strut axis matches ±(Q−P); the Motus planner may approach with a
    /// different jaw tilt than <see cref="BambooIcdCell.StrutPlacement.PitchDeg"/>.
    /// </summary>
    public static BambooIcdCell.StrutPlacement AlongEdge(
        (double X, double Y, double Z) p, (double X, double Y, double Z) q)
    {
        var dx = q.X - p.X; var dy = q.Y - p.Y; var dz = q.Z - p.Z;
        var len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        if (len < StrutLength - 1e-6)
            throw new InvalidOperationException($"edge length {len:F4} m < strut {StrutLength} m");
        dx /= len; dy /= len; dz /= len;
        // Prefer the sign that keeps |pitch| description stable; Motus accepts ±axis.
        var (pitch, yaw) = PitchYawForAxis(dx, dy, dz);
        // If the opposite sign yields a Motus-friendlier yaw hemisphere, prefer it when both are valid.
        var alt = PitchYawForAxis(-dx, -dy, -dz);
        if (PreferAlt(pitch, yaw, alt.Pitch, alt.Yaw))
            (pitch, yaw) = alt;
        var mx = (p.X + q.X) / 2;
        var my = (p.Y + q.Y) / 2;
        var mz = (p.Z + q.Z) / 2;
        return new BambooIcdCell.StrutPlacement(mx, my, mz, pitch, yaw);
    }

    static (double Pitch, double Yaw) PitchYawForAxis(double dx, double dy, double dz)
    {
        if (dx * dx + dz * dz < 1e-12)
            return (dy >= 0 ? -90 : 90, 0);
        var pitch = Math.Atan2(-dy, Math.Sqrt(dx * dx + dz * dz)) * 180 / Math.PI;
        var yaw = Math.Atan2(dx, dz) * 180 / Math.PI;
        return (pitch, yaw);
    }

    /// <summary>Prefer representations that Motus jaw-axis IK finds at the edge midpoint (yaw±90 / steep).</summary>
    static bool PreferAlt(double p0, double y0, double p1, double y1)
    {
        // Prefer pitch ≤ 0 when |yaw| is near 90 (ay ≥ 0 hemisphere for upright-ish canes).
        if (Math.Abs(Math.Abs(y1) - 90) < 1 && p1 <= 0 && p0 > 0) return true;
        if (Math.Abs(y1) < 1e-9 && Math.Abs(p1 + 90) < 1) return true; // pure up
        return false;
    }

    public static object FootprintDescription() => new
    {
        nodes = FootNodes.Select(n => new { name = n.Name, x = n.X, y = n.Y, z = n.Z }),
        edges = Members.Select(m => new
        {
            id = m.Id,
            ax = m.EdgeA.X, ay = m.EdgeA.Y, az = m.EdgeA.Z,
            bx = m.EdgeB.X, by = m.EdgeB.Y, bz = m.EdgeB.Z
        })
    };

    public static object StoreDescription() => new
    {
        slots = new[] { new { id = "feed", x = Store.X, y = Store.Y, z = Store.Z, pitch = Store.PitchDeg, yaw = Store.YawDeg } }
    };
}
