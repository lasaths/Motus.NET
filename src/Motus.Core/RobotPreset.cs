namespace Motus.Core;

public sealed class RobotPreset
{
    public RobotManufacturer Manufacturer { get; init; }
    public string ModelName { get; init; } = "";
    public string Family { get; init; } = "";
    public int AxisCount { get; init; }
    public IReadOnlyList<JointLimit> JointLimits { get; init; } = Array.Empty<JointLimit>();
    public double? ReachMeters { get; init; }
    public double? PayloadKg { get; init; }
    public BaseFrame BaseFrame { get; init; } = BaseFrame.Identity;
    public ToolFrame ToolFrame { get; init; } = ToolFrame.Identity;
    public string? Notes { get; init; }
    public string? SourceNote { get; init; }
    public string Disclaimer { get; init; } =
        "Preset values are planning/visualization defaults only, not physical compatibility guarantees.";

    public RobotModel ToModel() => new(this);
}
