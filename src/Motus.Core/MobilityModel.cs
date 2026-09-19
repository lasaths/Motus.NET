namespace Motus.Core;

/// <summary>
/// Base mobility for non-fixed cells (Wave 2 hook).
/// Holonomic SE(2) planar + Holonomic SE(3) free-flyer; nonholonomic / climbing later.
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

    /// <summary>
    /// Holonomic free-flyer in SE(3): position (m) + fixed-axis XYZ RPY (rad), URDF/ROS ZYX convention.
    /// Ponytail orientation is RPY with singularity Status near pitch ±π/2; quaternion sampling is deferred.
    /// Motus-honest aerial planning only — not a flight controller.
    /// </summary>
    public sealed class HolonomicSE3 : MobilityModel
    {
        /// <summary>Reject RPY when |cos(pitch)| is below this (gimbal lock for ZYX).</summary>
        public const double PitchSingularityCosEpsilon = 1e-3;

        public double X { get; }
        public double Y { get; }
        public double Z { get; }
        public double RollRadians { get; }
        public double PitchRadians { get; }
        public double YawRadians { get; }

        public HolonomicSE3(
            double x,
            double y,
            double z,
            double rollRadians,
            double pitchRadians,
            double yawRadians)
        {
            X = x;
            Y = y;
            Z = z;
            RollRadians = rollRadians;
            PitchRadians = pitchRadians;
            YawRadians = yawRadians;
        }

        public override Frame BaseFrame
        {
            get
            {
                var (qw, qx, qy, qz) = RpyToQuaternion(RollRadians, PitchRadians, YawRadians);
                return new Frame(X, Y, Z, qw, qx, qy, qz);
            }
        }

        /// <summary>
        /// True when fixed-axis XYZ RPY is near pitch singularity (gimbal lock).
        /// Callers should surface this as Status rather than silently clamping.
        /// </summary>
        public static bool IsPitchSingular(double pitchRadians) =>
            Math.Abs(Math.Cos(pitchRadians)) < PitchSingularityCosEpsilon;

        /// <summary>
        /// Build from a Motus frame. Fails with a named Status when orientation is non-finite
        /// or near RPY pitch singularity.
        /// </summary>
        public static bool TryFromFrame(Frame frame, out HolonomicSE3 pose, out string? status)
        {
            pose = default!;
            status = null;
            if (!double.IsFinite(frame.X) || !double.IsFinite(frame.Y) || !double.IsFinite(frame.Z) ||
                !double.IsFinite(frame.Qw) || !double.IsFinite(frame.Qx) ||
                !double.IsFinite(frame.Qy) || !double.IsFinite(frame.Qz))
            {
                status = "HolonomicSE3 frame contains non-finite position/orientation.";
                return false;
            }

            if (!TryQuaternionToRpy(frame.Qw, frame.Qx, frame.Qy, frame.Qz, out var roll, out var pitch, out var yaw, out status))
                return false;

            pose = new HolonomicSE3(frame.X, frame.Y, frame.Z, roll, pitch, yaw);
            return true;
        }

        /// <summary>Fixed-axis XYZ RPY → unit quaternion (w,x,y,z), matching URDF / <c>Transforms.FromRpy</c>.</summary>
        public static (double qw, double qx, double qy, double qz) RpyToQuaternion(
            double roll,
            double pitch,
            double yaw)
        {
            var cr = Math.Cos(roll * 0.5);
            var sr = Math.Sin(roll * 0.5);
            var cp = Math.Cos(pitch * 0.5);
            var sp = Math.Sin(pitch * 0.5);
            var cy = Math.Cos(yaw * 0.5);
            var sy = Math.Sin(yaw * 0.5);
            var qw = cr * cp * cy + sr * sp * sy;
            var qx = sr * cp * cy - cr * sp * sy;
            var qy = cr * sp * cy + sr * cp * sy;
            var qz = cr * cp * sy - sr * sp * cy;
            var n = Math.Sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
            if (n < 1e-15)
                return (1, 0, 0, 0);
            return (qw / n, qx / n, qy / n, qz / n);
        }

        /// <summary>Unit quaternion → fixed-axis XYZ RPY. Rejects pitch singularity with Status.</summary>
        public static bool TryQuaternionToRpy(
            double qw,
            double qx,
            double qy,
            double qz,
            out double roll,
            out double pitch,
            out double yaw,
            out string? status)
        {
            roll = pitch = yaw = 0;
            status = null;
            var n = Math.Sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
            if (!double.IsFinite(n) || n < 1e-15)
            {
                status = "HolonomicSE3 orientation quaternion is zero or non-finite.";
                return false;
            }

            qw /= n;
            qx /= n;
            qy /= n;
            qz /= n;

            // ZYX yaw-pitch-roll (same as Stewart FK / URDF FrameToRpy).
            var sinr = 2 * (qw * qx + qy * qz);
            var cosr = 1 - 2 * (qx * qx + qy * qy);
            roll = Math.Atan2(sinr, cosr);

            var sinp = 2 * (qw * qy - qz * qx);
            if (Math.Abs(sinp) >= 1.0 - PitchSingularityCosEpsilon * PitchSingularityCosEpsilon)
            {
                status =
                    $"HolonomicSE3 RPY singular: |sin(pitch)|≈1 (gimbal lock near pitch ±π/2). " +
                    "Narrow pitch bounds or choose a non-singular orientation.";
                return false;
            }

            pitch = Math.Asin(sinp);
            if (IsPitchSingular(pitch))
            {
                status =
                    $"HolonomicSE3 RPY singular: pitch={pitch:F4} rad near ±π/2 (cos(pitch)≈0).";
                return false;
            }

            var siny = 2 * (qw * qz + qx * qy);
            var cosy = 1 - 2 * (qy * qy + qz * qz);
            yaw = Math.Atan2(siny, cosy);
            return true;
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

/// <summary>
/// Bounds for holonomic SE(3) free-flyer planning. Position meters; roll/pitch/yaw radians (fixed-axis XYZ).
/// Default pitch stays inside (−π/2+ε, π/2−ε) so sampling does not land on RPY singularity.
/// </summary>
public sealed class MobilityBoundsSE3
{
    public double MinX { get; init; } = -2.0;
    public double MaxX { get; init; } = 2.0;
    public double MinY { get; init; } = -2.0;
    public double MaxY { get; init; } = 2.0;
    public double MinZ { get; init; } = 0.0;
    public double MaxZ { get; init; } = 3.0;
    public double MinRollRadians { get; init; } = -Math.PI;
    public double MaxRollRadians { get; init; } = Math.PI;
    /// <summary>Default ±1.4 rad keeps |cos(pitch)| well above singularity epsilon.</summary>
    public double MinPitchRadians { get; init; } = -1.4;
    public double MaxPitchRadians { get; init; } = 1.4;
    public double MinYawRadians { get; init; } = -Math.PI;
    public double MaxYawRadians { get; init; } = Math.PI;

    public static MobilityBoundsSE3 Default { get; } = new();

    public IReadOnlyList<JointLimit> ToJointLimits() =>
    [
        JointLimit.Meters(MinX, MaxX),
        JointLimit.Meters(MinY, MaxY),
        JointLimit.Meters(MinZ, MaxZ),
        JointLimit.Radians(MinRollRadians, MaxRollRadians),
        JointLimit.Radians(MinPitchRadians, MaxPitchRadians),
        JointLimit.Radians(MinYawRadians, MaxYawRadians)
    ];

    public string? Validate(MobilityModel.HolonomicSE3 pose, string label)
    {
        if (!double.IsFinite(pose.X) || !double.IsFinite(pose.Y) || !double.IsFinite(pose.Z) ||
            !double.IsFinite(pose.RollRadians) || !double.IsFinite(pose.PitchRadians) ||
            !double.IsFinite(pose.YawRadians))
            return $"{label} HolonomicSE3 contains non-finite x/y/z/roll/pitch/yaw.";

        if (MobilityModel.HolonomicSE3.IsPitchSingular(pose.PitchRadians))
            return $"{label} HolonomicSE3 RPY singular: pitch={pose.PitchRadians:F4} rad near ±π/2.";

        if (!BoundsFiniteAndOrdered())
            return "MobilityBoundsSE3 must be finite and ordered.";

        if (MobilityModel.HolonomicSE3.IsPitchSingular(MinPitchRadians) ||
            MobilityModel.HolonomicSE3.IsPitchSingular(MaxPitchRadians) ||
            MinPitchRadians <= -Math.PI / 2 || MaxPitchRadians >= Math.PI / 2)
            return "MobilityBoundsSE3 pitch limits must stay strictly inside (−π/2, π/2) away from singularity.";

        if (pose.X < MinX || pose.X > MaxX)
            return $"{label} HolonomicSE3 X={pose.X:F4} m outside [{MinX:F4}, {MaxX:F4}] m.";
        if (pose.Y < MinY || pose.Y > MaxY)
            return $"{label} HolonomicSE3 Y={pose.Y:F4} m outside [{MinY:F4}, {MaxY:F4}] m.";
        if (pose.Z < MinZ || pose.Z > MaxZ)
            return $"{label} HolonomicSE3 Z={pose.Z:F4} m outside [{MinZ:F4}, {MaxZ:F4}] m.";
        if (pose.RollRadians < MinRollRadians || pose.RollRadians > MaxRollRadians)
            return $"{label} HolonomicSE3 roll={pose.RollRadians:F4} rad outside [{MinRollRadians:F4}, {MaxRollRadians:F4}] rad.";
        if (pose.PitchRadians < MinPitchRadians || pose.PitchRadians > MaxPitchRadians)
            return $"{label} HolonomicSE3 pitch={pose.PitchRadians:F4} rad outside [{MinPitchRadians:F4}, {MaxPitchRadians:F4}] rad.";
        if (pose.YawRadians < MinYawRadians || pose.YawRadians > MaxYawRadians)
            return $"{label} HolonomicSE3 yaw={pose.YawRadians:F4} rad outside [{MinYawRadians:F4}, {MaxYawRadians:F4}] rad.";
        return null;
    }

    private bool BoundsFiniteAndOrdered() =>
        double.IsFinite(MinX) && double.IsFinite(MaxX) && MaxX >= MinX &&
        double.IsFinite(MinY) && double.IsFinite(MaxY) && MaxY >= MinY &&
        double.IsFinite(MinZ) && double.IsFinite(MaxZ) && MaxZ >= MinZ &&
        double.IsFinite(MinRollRadians) && double.IsFinite(MaxRollRadians) && MaxRollRadians >= MinRollRadians &&
        double.IsFinite(MinPitchRadians) && double.IsFinite(MaxPitchRadians) && MaxPitchRadians >= MinPitchRadians &&
        double.IsFinite(MinYawRadians) && double.IsFinite(MaxYawRadians) && MaxYawRadians >= MinYawRadians;
}
