using Motus.Geometry;

namespace Motus.Construction;

/// <summary>
/// One gripper of the two-gripper climbing arm. Its grip frame is the URDF frame of <see cref="Link"/>: origin = grasp
/// centre, local Z = jaw slot axis, local X = jaw opening direction. Jaw strokes (open / closed mm) are in
/// <see cref="BambooTrussLayout.SceneDescription"/>.
/// </summary>
/// <param name="Name">"L" (the base gripper of the chain) or "R" (the tip gripper).</param>
/// <param name="Link">URDF link whose frame is the grip frame.</param>
/// <param name="JawPlates">Collision part names of the two jaw plates ([0] moves to −X, [1] to +X when opening).</param>
/// <param name="PartNames">Other collision parts rigid with this gripper (exact names).</param>
/// <param name="PartPrefixes">Collision part name prefixes of this gripper's parts.</param>
public sealed record GripperDefinition(
    string Name, string Link, IReadOnlyList<string> JawPlates, IReadOnlyList<string> PartNames, IReadOnlyList<string> PartPrefixes)
{
    internal string[] Plates { get; } = JawPlates.ToArray();

    /// <summary>Value equality (the record's own compares the lists by reference).</summary>
    public bool SameAs(GripperDefinition o) =>
        Name == o.Name && Link == o.Link && JawPlates.SequenceEqual(o.JawPlates) && PartNames.SequenceEqual(o.PartNames) && PartPrefixes.SequenceEqual(o.PartPrefixes);

    /// <summary>True when collision part <paramref name="part"/> belongs to this gripper.</summary>
    public bool Owns(string part)
    {
        foreach (var n in PartNames) if (part == n) return true;
        foreach (var p in PartPrefixes) if (part.StartsWith(p, StringComparison.Ordinal)) return true;
        return false;
    }
}

/// <summary>
/// The robot: URDF base link, the two grippers, the serial chain L → R and contacts that are part of the mechanism.
/// The analytic IK needs a 5-joint chain with axes Y, X, X, X, Y (L-grip local frame) and joint offsets along −Y; the
/// segment lengths and joint limits are read from the URDF (<see cref="ReadChain"/>).
/// </summary>
public sealed record RobotDefinition(
    string BaseLink,
    GripperDefinition L,
    GripperDefinition R,
    IReadOnlyList<string> ChainJoints,
    IReadOnlyList<(string A, string B)> MechanismContacts)
{
    /// <summary>The bamboo_icd.urdf arm (default).</summary>
    public static RobotDefinition BambooIcd { get; } = new(
        "base_link",
        new GripperDefinition("L", "mount", ["L-grip jaw-", "L-grip jaw+"], ["base tube"], ["L-grip"]),
        new GripperDefinition("R", "tool0", ["R-grip jaw-", "R-grip jaw+"], ["neck"], ["R-grip"]),
        ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "tool_joint"],
        // θ1 housing is journaled in the base tube; the opened L-grip carriage sits in the joint-1 housing.
        [("θ1", "base tube"), ("θ1", "L-grip body")]);

    /// <summary>Value equality (the record's own compares the lists by reference).</summary>
    public bool SameAs(RobotDefinition o) =>
        BaseLink == o.BaseLink && L.SameAs(o.L) && R.SameAs(o.R) && ChainJoints.SequenceEqual(o.ChainJoints) && MechanismContacts.SequenceEqual(o.MechanismContacts);

    /// <summary>Robot the cells and collision checks use (process-wide; <see cref="AssemblyPlanner"/> scopes it per call).</summary>
    public static RobotDefinition Current { get; internal set; } = BambooIcd;

    /// <summary>Joint axes (L-grip local frame) the analytic IK is written for.</summary>
    public static readonly char[] SupportedAxes = { 'Y', 'X', 'X', 'X', 'Y' };

    /// <summary>
    /// Segment offsets (along −Y: base → θ1, …, θ5 → tool) and joint limits (degrees) of <see cref="ChainJoints"/> in
    /// <paramref name="tree"/>. Throws <see cref="NotSupportedException"/> when the chain does not fit the analytic IK.
    /// </summary>
    public (double[] Seg, (double Min, double Max)[] LimitsDeg) ReadChain(KinematicTree tree)
    {
        if (ChainJoints.Count != SupportedAxes.Length + 1)
            throw new NotSupportedException($"robot chain needs {SupportedAxes.Length} revolute joints + a tool joint, got {ChainJoints.Count} names");
        var seg = new double[ChainJoints.Count];
        var lim = new (double, double)[SupportedAxes.Length];
        for (var i = 0; i < ChainJoints.Count; i++)
        {
            var j = tree.Joints.FirstOrDefault(x => x.Name == ChainJoints[i])
                    ?? throw new NotSupportedException($"robot chain: joint '{ChainJoints[i]}' not in the URDF");
            if (Math.Abs(j.OriginX) > 1e-9 || Math.Abs(j.OriginZ) > 1e-9)
                throw new NotSupportedException($"robot chain: joint '{j.Name}' offset must lie along −Y");
            seg[i] = -j.OriginY;
            if (i == SupportedAxes.Length) break;
            var axis = Math.Abs(j.AxisX) > 1 - 1e-9 ? 'X' : Math.Abs(j.AxisY) > 1 - 1e-9 ? 'Y' : Math.Abs(j.AxisZ) > 1 - 1e-9 ? 'Z' : '?';
            if (axis != SupportedAxes[i])
                throw new NotSupportedException($"robot chain: joint '{j.Name}' axis {axis}, the analytic IK needs {SupportedAxes[i]}");
            lim[i] = (j.Lower * 180 / Math.PI, j.Upper * 180 / Math.PI);
        }
        return (seg, lim);
    }
}
