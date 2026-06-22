namespace Motus.Core;

public sealed class JointLimit
{
    public double MinRadians { get; }
    public double MaxRadians { get; }
    public double? MaxVelocityRadiansPerSecond { get; }
    public double? MaxAccelerationRadiansPerSecondSquared { get; }

    public JointLimit(double minRadians, double maxRadians,
        double? maxVelocityRadiansPerSecond = null,
        double? maxAccelerationRadiansPerSecondSquared = null)
    {
        MinRadians = minRadians;
        MaxRadians = maxRadians;
        MaxVelocityRadiansPerSecond = maxVelocityRadiansPerSecond;
        MaxAccelerationRadiansPerSecondSquared = maxAccelerationRadiansPerSecondSquared;
    }

    public bool Contains(double radians) => radians >= MinRadians && radians <= MaxRadians;
}
