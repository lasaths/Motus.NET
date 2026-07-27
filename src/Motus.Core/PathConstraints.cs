namespace Motus.Core;

/// <summary>Cartesian TCP path constraints shaped after MoveIt messages, without ROS dependencies.</summary>
public sealed class PathConstraints : IConstraintChecker
{
    public string Name { get; init; } = string.Empty;
    public IReadOnlyList<PositionConstraint> PositionConstraints { get; init; } = Array.Empty<PositionConstraint>();
    public IReadOnlyList<OrientationConstraint> OrientationConstraints { get; init; } = Array.Empty<OrientationConstraint>();

    public bool TryValidate(Frame tcp, out string reason)
    {
        if (!FrameIsFinite(tcp))
        {
            reason = "ConstraintViolation: TCP frame contains NaN/Inf.";
            return false;
        }

        foreach (var constraint in PositionConstraints)
        {
            if (!constraint.TryValidate(tcp, out reason))
                return false;
        }

        foreach (var constraint in OrientationConstraints)
        {
            if (!constraint.TryValidate(tcp, out reason))
                return false;
        }

        reason = string.Empty;
        return true;
    }

    internal static bool FrameIsFinite(Frame frame) =>
        double.IsFinite(frame.X) && double.IsFinite(frame.Y) && double.IsFinite(frame.Z) &&
        double.IsFinite(frame.Qw) && double.IsFinite(frame.Qx) &&
        double.IsFinite(frame.Qy) && double.IsFinite(frame.Qz);
}

public sealed class PositionConstraint : IConstraintChecker
{
    public string Name { get; init; } = string.Empty;
    public string LinkName { get; init; } = "tool0";
    public string HeaderFrameId { get; init; } = "world";
    public Frame Target { get; init; } = Frame.Identity;
    public double ToleranceXMeters { get; init; }
    public double ToleranceYMeters { get; init; }
    public double ToleranceZMeters { get; init; }
    public double Weight { get; init; } = 1.0;

    public bool TryValidate(Frame tcp, out string reason)
    {
        if (!PathConstraints.FrameIsFinite(Target) || !PathConstraints.FrameIsFinite(tcp))
        {
            reason = "ConstraintViolation: PositionConstraint frame contains NaN/Inf.";
            return false;
        }

        if (!FiniteNonNegative(ToleranceXMeters) ||
            !FiniteNonNegative(ToleranceYMeters) ||
            !FiniteNonNegative(ToleranceZMeters))
        {
            reason = "ConstraintViolation: PositionConstraint tolerances must be finite meters >= 0.";
            return false;
        }

        var dx = Math.Abs(tcp.X - Target.X);
        var dy = Math.Abs(tcp.Y - Target.Y);
        var dz = Math.Abs(tcp.Z - Target.Z);
        if (dx <= ToleranceXMeters && dy <= ToleranceYMeters && dz <= ToleranceZMeters)
        {
            reason = string.Empty;
            return true;
        }

        reason =
            $"ConstraintViolation: PositionConstraint '{DisplayName}' exceeded meters tolerance " +
            $"(dx={dx:G6}/{ToleranceXMeters:G6}, dy={dy:G6}/{ToleranceYMeters:G6}, dz={dz:G6}/{ToleranceZMeters:G6}).";
        return false;
    }

    private string DisplayName => string.IsNullOrWhiteSpace(Name) ? LinkName : Name;

    private static bool FiniteNonNegative(double value) => double.IsFinite(value) && value >= 0;
}

public sealed class OrientationConstraint : IConstraintChecker
{
    public string Name { get; init; } = string.Empty;
    public string LinkName { get; init; } = "tool0";
    public string HeaderFrameId { get; init; } = "world";
    public Frame Target { get; init; } = Frame.Identity;
    public double AbsoluteXAxisToleranceRadians { get; init; }
    public double AbsoluteYAxisToleranceRadians { get; init; }
    public double AbsoluteZAxisToleranceRadians { get; init; }
    public double Weight { get; init; } = 1.0;

    public bool TryValidate(Frame tcp, out string reason)
    {
        if (!PathConstraints.FrameIsFinite(Target) || !PathConstraints.FrameIsFinite(tcp))
        {
            reason = "ConstraintViolation: OrientationConstraint frame contains NaN/Inf.";
            return false;
        }

        var tol = Math.Min(
            Math.Min(AbsoluteXAxisToleranceRadians, AbsoluteYAxisToleranceRadians),
            AbsoluteZAxisToleranceRadians);
        if (!double.IsFinite(tol) || tol < 0)
        {
            reason = "ConstraintViolation: OrientationConstraint tolerances must be finite radians >= 0.";
            return false;
        }

        var angle = QuaternionAngularDistanceRadians(Target, tcp);
        if (angle <= tol)
        {
            reason = string.Empty;
            return true;
        }

        reason =
            $"ConstraintViolation: OrientationConstraint '{DisplayName}' exceeded radians tolerance " +
            $"(angle={angle:G6}/{tol:G6}).";
        return false;
    }

    private string DisplayName => string.IsNullOrWhiteSpace(Name) ? LinkName : Name;

    private static double QuaternionAngularDistanceRadians(Frame a, Frame b)
    {
        var an = Normalize(a.Qw, a.Qx, a.Qy, a.Qz);
        var bn = Normalize(b.Qw, b.Qx, b.Qy, b.Qz);
        var dot = Math.Abs(an.w * bn.w + an.x * bn.x + an.y * bn.y + an.z * bn.z);
        return 2.0 * Math.Acos(Math.Clamp(dot, -1.0, 1.0));
    }

    private static (double w, double x, double y, double z) Normalize(double w, double x, double y, double z)
    {
        var norm = Math.Sqrt(w * w + x * x + y * y + z * z);
        if (!double.IsFinite(norm) || norm < 1e-12)
            return (1, 0, 0, 0);
        return (w / norm, x / norm, y / norm, z / norm);
    }
}
