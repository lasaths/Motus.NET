namespace Motus.Core;

public sealed class RobotModel
{
    public RobotPreset Preset { get; }
    public RobotCollisionModel? CollisionModel { get; }
    public string DisplayName => $"{Preset.Manufacturer} {Preset.ModelName}";

    public RobotModel(RobotPreset preset, RobotCollisionModel? collisionModel = null)
    {
        Preset = preset ?? throw new ArgumentNullException(nameof(preset));
        if (preset.AxisCount != preset.JointLimits.Count)
            throw new ArgumentException("Axis count must match joint limit count.");
        CollisionModel = collisionModel;
    }

    public ValidationResult ValidateJointState(JointState state) => state.Validate(Preset.JointLimits);
}
