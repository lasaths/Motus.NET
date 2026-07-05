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
}
