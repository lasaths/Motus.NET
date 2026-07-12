namespace Motus.Core;

public sealed class RobotModel
{
    public RobotPreset Preset { get; }
    public RobotCollisionModel? CollisionModel { get; }
    /// <summary>Actuated joint names in chain order (base to tip), when known.</summary>
    public IReadOnlyList<string>? JointNames { get; }
    public string DisplayName => $"{Preset.Manufacturer} {Preset.ModelName}";

    public RobotModel(RobotPreset preset, RobotCollisionModel? collisionModel = null, IReadOnlyList<string>? jointNames = null)
    {
        Preset = preset ?? throw new ArgumentNullException(nameof(preset));
        if (preset.AxisCount != preset.JointLimits.Count)
            throw new ArgumentException("Axis count must match joint limit count.");
        CollisionModel = collisionModel;
        JointNames = jointNames;
    }

    public ValidationResult ValidateJointState(JointState state) => state.Validate(Preset.JointLimits);

    /// <summary>Session model with optional base/tool overrides and tool geometry merged into collision.</summary>
    public RobotModel WithTool(ToolDefinition? tool, BaseFrame? baseOverride = null)
    {
        var preset = Preset;
        if (baseOverride is not null || tool is not null)
        {
            preset = new RobotPreset
            {
                Manufacturer = Preset.Manufacturer,
                ModelName = Preset.ModelName,
                Family = Preset.Family,
                AxisCount = Preset.AxisCount,
                JointLimits = Preset.JointLimits,
                ReachMeters = Preset.ReachMeters,
                PayloadKg = Preset.PayloadKg,
                BaseFrame = baseOverride ?? Preset.BaseFrame,
                ToolFrame = tool?.ToToolFrame() ?? Preset.ToolFrame,
                Notes = Preset.Notes,
                SourceNote = Preset.SourceNote,
                Disclaimer = Preset.Disclaimer
            };
        }

        RobotCollisionModel? collision = CollisionModel;
        if (tool?.Geometry is not null)
        {
            var links = CollisionModel?.Links ?? Array.Empty<LinkCollisionGeometry>();
            collision = new RobotCollisionModel(
                links,
                tool.Geometry,
                tool.GeometryInFlangeFrame || UsesFlangeToolGeometry(tool.Geometry),
                tool.GeometryAttachOffset);
        }

        return new RobotModel(preset, collision, JointNames);
    }

    private static bool UsesFlangeToolGeometry(CollisionObject geometry) =>
        string.Equals(geometry.Name, "robotiq_2f85", StringComparison.Ordinal);
}
