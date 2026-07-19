namespace Motus.Core;

/// <summary>
/// Base mobility for non-fixed cells (Wave 2 hook).
/// Holonomic SE(2) only — nonholonomic / climbing later.
/// </summary>
public abstract class MobilityModel
{
    /// <summary>World pose of the kinematic tree root / robot base.</summary>
    public abstract Frame BaseFrame { get; }

    /// <summary>Holonomic planar base: (x, y, yaw) → Motus <see cref="Frame"/>.</summary>
    public sealed class HolonomicSE2 : MobilityModel
    {
        public double X { get; }
        public double Y { get; }
        public double YawRadians { get; }

        public HolonomicSE2(double x, double y, double yawRadians)
        {
            X = x;
            Y = y;
            YawRadians = yawRadians;
        }

        public override Frame BaseFrame
        {
            get
            {
                var half = YawRadians * 0.5;
                var qw = Math.Cos(half);
                var qz = Math.Sin(half);
                return new Frame(X, Y, 0, qw, 0, 0, qz);
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
