using Motus.Geometry;

namespace Motus.Construction;

/// <summary>
/// EXPERIMENTAL closed-form FK/IK for the 9-DOF bamboo handler (docs/handler9.md, tests/fixtures/handler9/handler9.urdf).
/// Each clamp frame has its origin on the cane axis and +x along the cane. Clamp open/close is gripper state, not a joint.
/// Joint positions are tree driver order (<see cref="KinematicTree.DriverJointIndices"/>).
/// </summary>
public sealed class Handler9Kinematics
{
    public static readonly string[] ClampLinks = { "handler_1_clamp", "main_clamp_blue", "main_clamp_yellow", "handler_2_clamp" };

    private const double Tol = 1e-6;
    private static readonly string[] Stations = { "handler_1", "main_clamp", "handler_2" };

    private readonly KinematicTree _tree;
    private readonly TreeForwardKinematics _fk;
    private readonly double[][] _mats;

    public Handler9Kinematics(KinematicTree tree)
    {
        _tree = tree;
        // The closed form assumes unrotated joint frames and these axes; anything else fails closed.
        foreach (var j in tree.Joints)
            if (j.Roll != 0 || j.Pitch != 0 || j.Yaw != 0)
                throw new ArgumentException($"handler9: joint '{j.Name}' must have rpy = 0.");
        foreach (var s in Stations)
        {
            Expect($"{s}_yaw", KinematicJointType.Revolute, 0, 0, 1);
            Expect($"{s}_lift", KinematicJointType.Prismatic, 0, 0, 1);
        }
        Expect("main_clamp_slide", KinematicJointType.Prismatic, 1, 0, 0);
        Expect("main_clamp_blue_roll", KinematicJointType.Revolute, 1, 0, 0);
        Expect("main_clamp_yellow_roll", KinematicJointType.Revolute, 1, 0, 0);
        if (tree.DriverCount != 9)
            throw new ArgumentException($"handler9: expected 9 actuated joints, got {tree.DriverCount}.");
        foreach (var c in ClampLinks)
            tree.IndexOfLink(c); // throws on a missing clamp link

        _fk = new TreeForwardKinematics(tree);
        _mats = new double[tree.Links.Count][];
        for (var i = 0; i < _mats.Length; i++)
            _mats[i] = new double[16];
    }

    public KinematicTree Tree => _tree;

    /// <summary>Driver slot of a named joint, for example <c>main_clamp_slide</c>.</summary>
    public int Slot(string jointName) => Joint(jointName).DriverIndex;

    /// <summary>World transform (row-major 4x4) of a clamp frame.</summary>
    public double[] ClampFrame(IReadOnlyList<double> q, string clampLink)
    {
        _fk.ComputeLinkTransformsInto(q, _mats);
        return (double[])_mats[_tree.IndexOfLink(clampLink)].Clone();
    }

    /// <summary>
    /// Puts one clamp frame origin on <paramref name="point"/> with +x parallel to <paramref name="axis"/> (either sign).
    /// Only that clamp's station joints change; rolls keep the seed (the cane line has no roll).
    /// </summary>
    public bool TrySolveClamp(string clampLink, double[] point, double[] axis, IReadOnlyList<double> seed, out double[] q, out string? reason)
    {
        q = seed.ToArray();
        return Validate(point, axis, seed, out reason) && SolveStation(clampLink, point, axis, q, alongFree: false, out reason);
    }

    /// <summary>
    /// Coupled solve: all three stations hold one straight cane. Every clamp origin lands on the cane line and every
    /// clamp +x is parallel to it. Where a clamp grips along the cane is free; the MainClamp slide keeps the seed.
    /// </summary>
    public bool TrySolveCane(double[] point, double[] axis, IReadOnlyList<double> seed, out double[] q, out string? reason)
    {
        q = seed.ToArray();
        if (!Validate(point, axis, seed, out reason))
            return false;
        foreach (var clamp in new[] { "handler_1_clamp", "main_clamp_blue", "handler_2_clamp" })
            if (!SolveStation(clamp, point, axis, q, alongFree: true, out reason))
                return false;
        return true;
    }

    private bool SolveStation(string clampLink, double[] p, double[] axis, double[] q, bool alongFree, out string? reason)
    {
        if (!ClampLinks.Contains(clampLink))
        {
            reason = $"Unknown clamp link '{clampLink}'.";
            return false;
        }
        var station = clampLink.StartsWith("main_clamp", StringComparison.Ordinal) ? "main_clamp" : clampLink[..^"_clamp".Length];
        var n = Math.Sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
        if (Math.Abs(axis[2] / n) > Tol)
        {
            reason = "Cane axis must be horizontal: the clamps have no pitch joint.";
            return false;
        }

        var yaw = Joint($"{station}_yaw");
        var lift = Joint($"{station}_lift");
        var slide = station == "main_clamp" ? Joint("main_clamp_slide") : null;

        // Reference: station yaw/lift/slide at 0. Clamp = C + Rz(yaw)(o + slide x) + lift z.
        var refQ = q.ToArray();
        refQ[yaw.DriverIndex] = 0;
        refQ[lift.DriverIndex] = 0;
        if (slide != null)
            refQ[slide.DriverIndex] = 0;
        _fk.ComputeLinkTransformsInto(refQ, _mats);
        var c = _mats[yaw.ChildLinkIndex];
        var p0 = _mats[_tree.IndexOfLink(clampLink)];
        double ox = p0[3] - c[3], oy = p0[7] - c[7];
        var liftQ = p[2] - p0[11];
        var slideSeed = slide == null ? 0 : q[slide.DriverIndex];

        // The cane line has no direction: try both headings, nearest the seed yaw first.
        var a = Math.Atan2(axis[1], axis[0]);
        var cands = new[] { a, a > 0 ? a - Math.PI : a + Math.PI }
            .OrderBy(t => Math.Abs(Math.IEEERemainder(t - q[yaw.DriverIndex], 2 * Math.PI)));
        reason = null;
        foreach (var t in cands)
        {
            double cs = Math.Cos(t), sn = Math.Sin(t), dx = p[0] - c[3], dy = p[1] - c[7];
            double u = cs * dx + sn * dy - ox, v = -sn * dx + cs * dy - oy;
            var s = slide == null ? 0 : alongFree ? slideSeed : u;
            if (Math.Abs(v) > Tol || (slide == null && !alongFree && Math.Abs(u) > Tol))
                reason ??= $"{station}: target is {Math.Max(Math.Abs(v), slide == null && !alongFree ? Math.Abs(u) : 0):F4} m off the station's reachable line.";
            else if (!InLimits(yaw, t))
                reason ??= $"{station}: yaw {t:F4} rad is outside its limits.";
            else if (!InLimits(lift, liftQ))
                reason ??= $"{station}: lift {liftQ:F4} m is outside [{lift.Lower}, {lift.Upper}].";
            else if (slide != null && !InLimits(slide, s))
                reason ??= $"{station}: slide {s:F4} m is outside [{slide.Lower}, {slide.Upper}].";
            else
            {
                q[yaw.DriverIndex] = t;
                q[lift.DriverIndex] = liftQ;
                if (slide != null)
                    q[slide.DriverIndex] = s;
                reason = null;
                return true;
            }
        }
        return false;
    }

    private bool Validate(double[] point, double[] axis, IReadOnlyList<double> seed, out string? reason)
    {
        reason = point.Length != 3 || axis.Length != 3 || seed.Count != _tree.DriverCount ? "Expected point[3], axis[3] and a full seed."
            : !point.Concat(axis).Concat(seed).All(double.IsFinite) ? "Inputs must be finite."
            : Math.Sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]) < 1e-12 ? "Cane axis must be non-zero."
            : null;
        return reason == null;
    }

    private static bool InLimits(KinematicJoint j, double v) => v >= j.Lower - 1e-12 && v <= j.Upper + 1e-12;

    private KinematicJoint Joint(string name) =>
        _tree.Joints.FirstOrDefault(j => j.Name == name) ?? throw new ArgumentException($"handler9: missing joint '{name}'.");

    private void Expect(string name, KinematicJointType type, double ax, double ay, double az)
    {
        var j = Joint(name);
        if (j.Type != type || j.AxisX != ax || j.AxisY != ay || j.AxisZ != az)
            throw new ArgumentException($"handler9: joint '{name}' must be {type} about ({ax}, {ay}, {az}).");
    }
}
