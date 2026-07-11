namespace Motus.Core;

/// <summary>Static end-effector: TCP offset from flange plus optional gripper collision volume.</summary>
public sealed class ToolDefinition
{
    public string Name { get; }
    public Frame Tcp { get; }
    /// <summary>Optional gripper collision in TCP-local frame.</summary>
    public CollisionObject? Geometry { get; }
    /// <summary>Actuated parameters exposed along trajectories (e.g. gripper width).</summary>
    public ToolCapabilities? Capabilities { get; }

    public ToolDefinition(string name, Frame tcp, CollisionObject? geometry = null, ToolCapabilities? capabilities = null)
    {
        Name = string.IsNullOrWhiteSpace(name) ? "tool" : name.Trim();
        Tcp = tcp;
        Geometry = geometry;
        Capabilities = capabilities;
    }

    /// <summary>Collision geometry for a tool state; falls back to static geometry when no state mapping exists.</summary>
    public CollisionObject? GeometryForState(EndEffectorState? state)
    {
        if (Geometry is null || state is null || Capabilities is null)
            return Geometry;

        if (!state.Values.TryGetValue("width", out var width))
            return Geometry;

        var openWidth = Capabilities.Parameters.FirstOrDefault(p =>
            string.Equals(p.Name, "width", StringComparison.Ordinal))?.Max ?? 0.085;
        if (openWidth <= 1e-9) return Geometry;

        var ratio = Math.Clamp(width / openWidth, 0, 1);
        return ScaleMeshWidth(Geometry, ratio);
    }

    private static CollisionObject ScaleMeshWidth(CollisionObject source, double widthRatio)
    {
        if (source.Shape != CollisionShape.Mesh || source.MeshVertices is not { Count: > 0 } vertices)
        {
            if (source.Shape == CollisionShape.Box)
                return CollisionObject.Box(source.Name, source.Pose, source.ExtentX * widthRatio, source.ExtentY, source.ExtentZ);
            return source;
        }

        var scaled = new List<double[]>(vertices.Count);
        foreach (var v in vertices)
        {
            scaled.Add(new[] { v[0] * widthRatio, v[1], v[2] });
        }

        return CollisionObject.Mesh(source.Name, source.Pose, scaled, source.MeshIndices ?? new List<int>());
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
