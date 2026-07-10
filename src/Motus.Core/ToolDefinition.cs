namespace Motus.Core;

/// <summary>Static end-effector: TCP offset from flange plus optional gripper collision volume.</summary>
public sealed class ToolDefinition
{
    public string Name { get; }
    public Frame Tcp { get; }
    /// <summary>Optional gripper collision in TCP-local frame.</summary>
    public CollisionObject? Geometry { get; }

    public ToolDefinition(string name, Frame tcp, CollisionObject? geometry = null)
    {
        Name = string.IsNullOrWhiteSpace(name) ? "tool" : name.Trim();
        Tcp = tcp;
        Geometry = geometry;
    }

    public ToolFrame ToToolFrame() => new(Tcp, Name);

    /// <summary>Build from a preset model's tool frame and bundled tool collision, if any.</summary>
    public static ToolDefinition? FromPreset(RobotModel robot)
    {
        var tf = robot.Preset.ToolFrame;
        var geom = robot.CollisionModel?.ToolGeometry;
        if (geom is null && tf.Frame.Equals(Frame.Identity) && string.IsNullOrWhiteSpace(tf.Name))
            return null;
        return new ToolDefinition(tf.Name ?? "tool", tf.Frame, geom);
    }
}
