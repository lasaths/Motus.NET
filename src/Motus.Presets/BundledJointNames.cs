namespace Motus.Presets;

/// <summary>Default actuated joint names for bundled presets (UR convention).</summary>
public static class BundledJointNames
{
    private static readonly string[] Ur6 =
        ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"];

    public static IReadOnlyList<string>? TryGet(string modelName)
    {
        if (string.IsNullOrWhiteSpace(modelName)) return null;
        if (modelName.StartsWith("UR", StringComparison.OrdinalIgnoreCase) && modelName.Length > 2)
            return Ur6;
        return null;
    }
}
