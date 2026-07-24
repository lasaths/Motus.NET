namespace Motus.Core;

/// <summary>
/// Axis limit in the coordinate unit of that axis (<see cref="Unit"/>).
/// Legacy <see cref="MinRadians"/> / <see cref="MaxRadians"/> aliases return the same numeric bounds
/// (historical name; not always radians — check <see cref="Unit"/>).
/// </summary>
public sealed class JointLimit
{
    public double Min { get; }
    public double Max { get; }
    public JointCoordinateUnit Unit { get; }
    public double? MaxVelocity { get; }
    public double? MaxAcceleration { get; }

    /// <summary>Legacy alias of <see cref="Min"/> (unit = <see cref="Unit"/>).</summary>
    public double MinRadians => Min;

    /// <summary>Legacy alias of <see cref="Max"/> (unit = <see cref="Unit"/>).</summary>
    public double MaxRadians => Max;

    /// <summary>Legacy alias of <see cref="MaxVelocity"/>.</summary>
    public double? MaxVelocityRadiansPerSecond => MaxVelocity;

    /// <summary>Legacy alias of <see cref="MaxAcceleration"/>.</summary>
    public double? MaxAccelerationRadiansPerSecondSquared => MaxAcceleration;

    /// <summary>
    /// Revolute-compatible constructor (defaults to <see cref="JointCoordinateUnit.Radians"/>).
    /// Prefer <see cref="Radians"/> / <see cref="Meters"/> factories for clarity.
    /// </summary>
    public JointLimit(
        double min,
        double max,
        double? maxVelocity = null,
        double? maxAcceleration = null)
        : this(min, max, JointCoordinateUnit.Radians, maxVelocity, maxAcceleration)
    {
    }

    public JointLimit(
        double min,
        double max,
        JointCoordinateUnit unit,
        double? maxVelocity = null,
        double? maxAcceleration = null)
    {
        if (!double.IsFinite(min) || !double.IsFinite(max))
            throw new ArgumentException("JointLimit bounds must be finite.");
        if (max < min)
            throw new ArgumentException($"JointLimit max ({max}) must be >= min ({min}).");
        Min = min;
        Max = max;
        Unit = unit;
        MaxVelocity = maxVelocity;
        MaxAcceleration = maxAcceleration;
    }

    public static JointLimit Radians(
        double min,
        double max,
        double? maxVelocity = null,
        double? maxAcceleration = null) =>
        new(min, max, JointCoordinateUnit.Radians, maxVelocity, maxAcceleration);

    public static JointLimit Meters(
        double min,
        double max,
        double? maxVelocity = null,
        double? maxAcceleration = null) =>
        new(min, max, JointCoordinateUnit.Meters, maxVelocity, maxAcceleration);

    public bool Contains(double value) => value >= Min && value <= Max;

    public string UnitLabel => Unit == JointCoordinateUnit.Meters ? "m" : "rad";
}
