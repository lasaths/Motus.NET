namespace Motus.Core;

/// <summary>
/// Maps tool state parameters onto kinematic-tree driver q (Wave 2).
/// Robotiq 2F-85: width → left knuckle driver; URDF mimic owns the rest.
/// </summary>
public static class ToolParameterBinding
{
    public const double Robotiq2F85OpenWidthMeters = 0.085;
    public const double Robotiq2F85ClosedDriverRadians = 0.8;

    /// <summary>Width (m) → Robotiq 2F-85 primary knuckle driver angle (rad).</summary>
    public static double Robotiq2F85DriverAngleRadians(
        double widthMeters,
        double openWidthMeters = Robotiq2F85OpenWidthMeters)
    {
        var open = openWidthMeters > 1e-9 ? openWidthMeters : Robotiq2F85OpenWidthMeters;
        var ratio = Math.Clamp(widthMeters / open, 0, 1);
        return (1.0 - ratio) * Robotiq2F85ClosedDriverRadians;
    }

    /// <summary>
    /// True when this driver joint is the Robotiq 2F-85 primary knuckle (width maps here).
    /// </summary>
    public static bool IsRobotiq2F85PrimaryDriver(string jointName)
    {
        if (string.IsNullOrEmpty(jointName)) return false;
        return jointName.Contains("robotiq", StringComparison.OrdinalIgnoreCase)
            && jointName.Contains("left_knuckle", StringComparison.OrdinalIgnoreCase)
            && !jointName.Contains("finger", StringComparison.OrdinalIgnoreCase)
            && !jointName.Contains("inner", StringComparison.OrdinalIgnoreCase);
    }

    /// <summary>
    /// Apply <paramref name="state"/> onto driver q for known capabilities.
    /// Currently: Robotiq2F85 <c>width</c> → primary knuckle driver.
    /// Returns number of drivers written.
    /// </summary>
    public static int ApplyInto(
        ToolCapabilities? capabilities,
        EndEffectorState? state,
        IReadOnlyList<string> driverJointNames,
        Span<double> driverQ,
        double openWidthMeters = Robotiq2F85OpenWidthMeters)
    {
        if (capabilities is null || state is null || driverJointNames.Count == 0)
            return 0;
        if (!ReferenceEquals(capabilities, ToolCapabilities.Robotiq2F85)
            && !capabilities.Parameters.Any(p => p.Name.Equals("width", StringComparison.Ordinal)))
            return 0;
        if (!state.Values.TryGetValue("width", out var width))
            return 0;

        var jaw = Robotiq2F85DriverAngleRadians(width, openWidthMeters);
        var n = Math.Min(driverJointNames.Count, driverQ.Length);
        var written = 0;
        for (var i = 0; i < n; i++)
        {
            if (!IsRobotiq2F85PrimaryDriver(driverJointNames[i])) continue;
            driverQ[i] = jaw;
            written++;
        }
        return written;
    }
}
