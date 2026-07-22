using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Session-level helpers for composing a <see cref="RobotDescription"/> (arm + tool mechanism, e.g. a
/// gripper or turntable that carries its own joints) and projecting it to a <see cref="KinematicTree"/>
/// for FK / planning.
/// </summary>
/// <remarks>
/// This complements <see cref="RobotModel.WithTool"/>, which only merges <em>static</em> tool collision
/// geometry onto an existing preset/serial chain. Use <see cref="AttachTool"/> instead when the tool
/// itself is an actuated mechanism (its own links/joints) that must be merged into the arm's kinematic
/// tree before FK/IK/planning — e.g. a driven gripper or a turntable the arm is mounted on.
///
/// Typical Grasshopper flow: build arm and tool as two <see cref="RobotDescription"/> instances via
/// <see cref="RobotDescription.Assemble"/>, call <see cref="AttachTool"/> (or Robot <c>Tl</c> →
/// <see cref="RobotDescription.Attach"/>), then <see cref="Project"/> for TreeFK + optional tip extract.
/// </remarks>
public static class RobotDescriptionSession
{
    /// <summary>Attach tool mechanism onto arm description at parentLink with attachFrame; returns merged description.</summary>
    public static RobotDescription AttachTool(RobotDescription arm, RobotDescription toolMechanism, string parentLink, Frame attachFrame)
    {
        if (arm is null) throw new ArgumentNullException(nameof(arm));
        if (toolMechanism is null) throw new ArgumentNullException(nameof(toolMechanism));
        return arm.Attach(toolMechanism, parentLink, attachFrame);
    }

    /// <summary>
    /// Project description to <see cref="KinematicTree"/> + optional tip <see cref="SerialTipExtraction"/>.
    /// <paramref name="baseLink"/> defaults to the tree's root link; if <paramref name="tipLink"/> is null,
    /// no tip extraction is performed and <c>Tip</c> is null (use the tree directly with TreeFK for
    /// branching/multi-tip assemblies such as an arm attached to a turntable).
    /// </summary>
    public static (KinematicTree Tree, SerialTipExtraction? Tip) Project(RobotDescription description, string? baseLink = null, string? tipLink = null)
    {
        if (description is null) throw new ArgumentNullException(nameof(description));
        var tree = description.ToKinematicTree();

        var resolvedTip = string.IsNullOrWhiteSpace(tipLink) ? description.TipLink : tipLink;
        if (string.IsNullOrWhiteSpace(resolvedTip))
            return (tree, null);

        var resolvedBase = string.IsNullOrWhiteSpace(baseLink) ? tree.Links[tree.RootLinkIndex].Name : baseLink;
        return (tree, tree.ExtractSerialTip(resolvedBase, resolvedTip));
    }
}
