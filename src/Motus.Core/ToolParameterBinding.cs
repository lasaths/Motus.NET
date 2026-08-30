namespace Motus.Core;

/// <summary>
/// Declarative capability-parameter → kinematic driver-joint mapping (Wave 3).
/// State value <c>Parameter</c> maps linearly from <c>OpenValue</c>→0 driver to <c>ClosedDriverValue</c>,
/// written onto the driver q entry whose joint name equals <c>DriverJoint</c>.
/// </summary>
public readonly record struct ToolDriverBinding(
    string Parameter,
    string DriverJoint,
    double OpenValue,
    double ClosedDriverValue);

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
    /// Build a width→driver binding from Cap open width (m) and closed driver value (rad or m).
    /// Cap-agnostic; Robotiq defaults match <see cref="Robotiq2F85OpenWidthMeters"/> /
    /// <see cref="Robotiq2F85ClosedDriverRadians"/>.
    /// </summary>
    public static ToolDriverBinding WidthBinding(
        string driverJoint,
        double openWidthMeters,
        double closedDriverValue)
    {
        if (string.IsNullOrWhiteSpace(driverJoint))
            throw new ArgumentException("Driver joint name is required.", nameof(driverJoint));
        if (!(openWidthMeters > 1e-12) || double.IsNaN(openWidthMeters) || double.IsInfinity(openWidthMeters))
            throw new ArgumentException("Open width must be a positive finite value.", nameof(openWidthMeters));
        if (double.IsNaN(closedDriverValue) || double.IsInfinity(closedDriverValue))
            throw new ArgumentException("Closed driver value must be finite.", nameof(closedDriverValue));
        return new ToolDriverBinding("width", driverJoint.Trim(), openWidthMeters, closedDriverValue);
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

    /// <summary>
    /// Apply <paramref name="state"/> onto driver q using declarative <paramref name="bindings"/> when supplied
    /// (Cap→driver-joint map: exact joint-name match, linear OpenValue→0 to ClosedDriverValue).
    /// Falls back to the legacy Robotiq2F85 heuristic when <paramref name="bindings"/> is null/empty, for compat.
    /// Returns number of drivers written.
    /// </summary>
    public static int ApplyInto(
        ToolCapabilities? capabilities,
        EndEffectorState? state,
        IReadOnlyList<string> driverJointNames,
        Span<double> driverQ,
        IReadOnlyList<ToolDriverBinding>? bindings,
        double openWidthMeters = Robotiq2F85OpenWidthMeters)
    {
        if (state is null || driverJointNames.Count == 0)
            return 0;

        if (bindings is { Count: > 0 })
            return ApplyBindings(state, driverJointNames, driverQ, bindings);

        return ApplyInto(capabilities, state, driverJointNames, driverQ, openWidthMeters);
    }

    private static int ApplyBindings(
        EndEffectorState state,
        IReadOnlyList<string> driverJointNames,
        Span<double> driverQ,
        IReadOnlyList<ToolDriverBinding> bindings)
    {
        var n = Math.Min(driverJointNames.Count, driverQ.Length);
        var written = 0;
        foreach (var binding in bindings)
        {
            if (!state.Values.TryGetValue(binding.Parameter, out var value))
                continue;

            var open = binding.OpenValue;
            var ratio = Math.Abs(open) > 1e-9 ? Math.Clamp(value / open, 0, 1) : 0.0;
            var driverValue = (1.0 - ratio) * binding.ClosedDriverValue;

            for (var i = 0; i < n; i++)
            {
                if (!string.Equals(driverJointNames[i], binding.DriverJoint, StringComparison.OrdinalIgnoreCase))
                    continue;
                driverQ[i] = driverValue;
                written++;
            }
        }
        return written;
    }
}
