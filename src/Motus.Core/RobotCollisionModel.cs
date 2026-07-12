namespace Motus.Core;

/// <summary>Collision primitive in a link-local frame (attached to FK link index).</summary>
public sealed class LinkCollisionGeometry
{
    public int LinkIndex { get; }
    public string LinkName { get; }
    public CollisionObject LocalGeometry { get; }

    public LinkCollisionGeometry(int linkIndex, string linkName, CollisionObject localGeometry)
    {
        LinkIndex = linkIndex;
        LinkName = linkName;
        LocalGeometry = localGeometry;
    }
}

/// <summary>Per-link collision geometry for a robot (URDF, JSON preset, or host-supplied).</summary>
public sealed class RobotCollisionModel
{
    public IReadOnlyList<LinkCollisionGeometry> Links { get; }
    /// <summary>Optional tool collision mesh (TCP-local unless <see cref="ToolGeometryInFlangeFrame"/>).</summary>
    public CollisionObject? ToolGeometry { get; }
    /// <summary>When true, <see cref="ToolGeometry"/> is in flange/tool0 frame and placed at the FK chain tip.</summary>
    public bool ToolGeometryInFlangeFrame { get; }
    /// <summary>Fixed URDF offset from the last actuated FK link to <see cref="ToolGeometry"/> (e.g. wrist_3 → tool0).</summary>
    public Frame? ToolGeometryAttachOffset { get; }

    public RobotCollisionModel(
        IReadOnlyList<LinkCollisionGeometry> links,
        CollisionObject? toolGeometry = null,
        bool toolGeometryInFlangeFrame = false,
        Frame? toolGeometryAttachOffset = null)
    {
        Links = links ?? Array.Empty<LinkCollisionGeometry>();
        ToolGeometry = toolGeometry;
        ToolGeometryInFlangeFrame = toolGeometryInFlangeFrame;
        ToolGeometryAttachOffset = toolGeometryAttachOffset;
    }
}
