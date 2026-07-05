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
    /// <summary>Optional tool collision in TCP-local frame (static gripper volume).</summary>
    public CollisionObject? ToolGeometry { get; }

    public RobotCollisionModel(IReadOnlyList<LinkCollisionGeometry> links, CollisionObject? toolGeometry = null)
    {
        Links = links ?? Array.Empty<LinkCollisionGeometry>();
        ToolGeometry = toolGeometry;
    }
}
