namespace Motus.Core;

/// <summary>
/// Base mobility for non-fixed cells (Wave 2 hook).
/// Holonomic SE(2) only — nonholonomic / climbing later.
/// </summary>
public abstract class MobilityModel
{
    /// <summary>World pose of the kinematic tree root / robot base.</summary>
    public abstract Frame BaseFrame { get; }

    /// <summary>Holonomic planar base: (x, y, yaw[, z]) → Motus <see cref="Frame"/>. Z elevates for terrain.</summary>
    public sealed class HolonomicSE2 : MobilityModel
    {
        public double X { get; }
        public double Y { get; }
        public double Z { get; }
        public double YawRadians { get; }

        public HolonomicSE2(double x, double y, double yawRadians, double z = 0)
        {
            X = x;
            Y = y;
            Z = z;
            YawRadians = yawRadians;
        }

        public override Frame BaseFrame
        {
            get
            {
                var half = YawRadians * 0.5;
                var qw = Math.Cos(half);
                var qz = Math.Sin(half);
                return new Frame(X, Y, Z, qw, 0, 0, qz);
            }
        }
    }

    /// <summary>Fixed base (identity) — same as no mobility.</summary>
    public sealed class Fixed : MobilityModel
    {
        public static Fixed Instance { get; } = new();
        public override Frame BaseFrame => Frame.Identity;
    }
}

/// <summary>
/// Bounds for holonomic SE(2) mobile-base planning. Position units are meters and yaw is radians.
/// Defaults intentionally fence preview plans to a small local cell; widen explicitly for larger maps.
/// </summary>
public sealed class MobilityBounds
{
    public double MinX { get; init; } = -2.0;
    public double MaxX { get; init; } = 2.0;
    public double MinY { get; init; } = -2.0;
    public double MaxY { get; init; } = 2.0;
    public double MinYawRadians { get; init; } = -Math.PI;
    public double MaxYawRadians { get; init; } = Math.PI;

    public static MobilityBounds Default { get; } = new();

    public IReadOnlyList<JointLimit> ToJointLimits() =>
    [
        JointLimit.Meters(MinX, MaxX),
        JointLimit.Meters(MinY, MaxY),
        JointLimit.Radians(MinYawRadians, MaxYawRadians)
    ];

    public string? Validate(MobilityModel.HolonomicSE2 pose, string label)
    {
        if (!double.IsFinite(pose.X) || !double.IsFinite(pose.Y) ||
            !double.IsFinite(pose.Z) || !double.IsFinite(pose.YawRadians))
            return $"{label} HolonomicSE2 contains non-finite x/y/z/yaw.";
        if (!double.IsFinite(MinX) || !double.IsFinite(MaxX) || MaxX < MinX ||
            !double.IsFinite(MinY) || !double.IsFinite(MaxY) || MaxY < MinY ||
            !double.IsFinite(MinYawRadians) || !double.IsFinite(MaxYawRadians) || MaxYawRadians < MinYawRadians)
            return "MobilityBounds must be finite and ordered.";
        if (pose.X < MinX || pose.X > MaxX)
            return $"{label} HolonomicSE2 X={pose.X:F4} m outside [{MinX:F4}, {MaxX:F4}] m.";
        if (pose.Y < MinY || pose.Y > MaxY)
            return $"{label} HolonomicSE2 Y={pose.Y:F4} m outside [{MinY:F4}, {MaxY:F4}] m.";
        if (pose.YawRadians < MinYawRadians || pose.YawRadians > MaxYawRadians)
            return $"{label} HolonomicSE2 yaw={pose.YawRadians:F4} rad outside [{MinYawRadians:F4}, {MaxYawRadians:F4}] rad.";
        return null;
    }
}
