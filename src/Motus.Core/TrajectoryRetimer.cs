namespace Motus.Core;

public enum RetimerAlgorithm
{
    TotgLite,
    SegmentTrapezoid,
    Bottleneck
}

public sealed class TrajectoryRetimerOptions
{
    public RetimerAlgorithm Algorithm { get; init; } = RetimerAlgorithm.Bottleneck;
    public double DefaultMaxVelocityRadiansPerSecond { get; init; } = 1.5;
    public double DefaultMaxAccelerationRadiansPerSecondSquared { get; init; } = 3.0;
    public double DefaultMaxJerkRadiansPerSecondCubed { get; init; } = 15.0;
}

/// <summary>Joint-space retiming: segment trapezoid or path-wide bottleneck.</summary>
public static class TrajectoryRetimer
{
    public static Trajectory Retime(Trajectory geometric, TrajectoryRetimerOptions? options = null)
    {
        options ??= new TrajectoryRetimerOptions();
        if (options.Algorithm == RetimerAlgorithm.TotgLite || options.Algorithm == RetimerAlgorithm.Bottleneck)
            return RetimeBottleneck(geometric, options);
        return RetimeSegmentTrapezoid(geometric, options);
    }

    private static Trajectory RetimeBottleneck(Trajectory geometric, TrajectoryRetimerOptions options)
    {
        var points = geometric.Points;
        if (points.Count <= 1) return geometric;

        var limits = geometric.Robot.Preset.JointLimits;
        var n = limits.Count;
        var count = points.Count;

        var s = new double[count];
        for (var i = 1; i < count; i++)
        {
            var seg = 0.0;
            for (var j = 0; j < n; j++)
            {
                var dq = points[i].JointState.Positions[j] - points[i - 1].JointState.Positions[j];
                seg += dq * dq;
            }
            s[i] = s[i - 1] + Math.Sqrt(seg);
        }
        if (s[^1] < 1e-12) return geometric;

        var vLimit = new double[count];
        for (var i = 0; i < count; i++)
        {
            vLimit[i] = double.MaxValue;
            if (i == 0 || i == count - 1) { vLimit[i] = 0; continue; }
            for (var j = 0; j < n; j++)
            {
                var ds = s[i] - s[i - 1];
                if (ds < 1e-12) continue;
                var dqds = Math.Abs((points[i].JointState.Positions[j] - points[i - 1].JointState.Positions[j]) / ds);
                if (dqds < 1e-12) continue;
                var vmax = limits[j].MaxVelocityRadiansPerSecond ?? options.DefaultMaxVelocityRadiansPerSecond;
                vLimit[i] = Math.Min(vLimit[i], vmax / dqds);
            }
            if (double.IsPositiveInfinity(vLimit[i])) vLimit[i] = options.DefaultMaxVelocityRadiansPerSecond;
        }

        var vFwd = (double[])vLimit.Clone();
        for (var i = 1; i < count; i++)
        {
            var ds = s[i] - s[i - 1];
            if (ds < 1e-12) continue;
            var amax = MinAccel(limits, n, options);
            vFwd[i] = Math.Min(vFwd[i], Math.Sqrt(vFwd[i - 1] * vFwd[i - 1] + 2 * amax * ds));
        }

        var v = (double[])vFwd.Clone();
        for (var i = count - 2; i >= 0; i--)
        {
            var ds = s[i + 1] - s[i];
            if (ds < 1e-12) continue;
            var amax = MinAccel(limits, n, options);
            v[i] = Math.Min(v[i], Math.Sqrt(v[i + 1] * v[i + 1] + 2 * amax * ds));
        }

        var retimed = new List<TrajectoryPoint>(count) { new(0, points[0].JointState) };
        var t = 0.0;
        for (var i = 1; i < count; i++)
        {
            var ds = s[i] - s[i - 1];
            var vAvg = Math.Max(1e-3, (v[i - 1] + v[i]) * 0.5);
            t += ds / vAvg;
            retimed.Add(new TrajectoryPoint(t, points[i].JointState));
        }

        return new Trajectory(geometric.Robot, retimed);
    }

    private static double MinAccel(IReadOnlyList<JointLimit> limits, int n, TrajectoryRetimerOptions options)
    {
        var min = options.DefaultMaxAccelerationRadiansPerSecondSquared;
        for (var j = 0; j < n; j++)
            min = Math.Min(min, limits[j].MaxAccelerationRadiansPerSecondSquared ?? options.DefaultMaxAccelerationRadiansPerSecondSquared);
        return Math.Max(min, 1e-3);
    }

    private static Trajectory RetimeSegmentTrapezoid(Trajectory geometric, TrajectoryRetimerOptions options)
    {
        var points = geometric.Points;
        if (points.Count <= 1) return geometric;

        var limits = geometric.Robot.Preset.JointLimits;
        var n = limits.Count;
        var retimed = new List<TrajectoryPoint>(points.Count) { new(0, points[0].JointState) };
        var t = 0.0;
        double[]? prevVel = null;

        for (var i = 1; i < points.Count; i++)
        {
            var dt = MinSegmentDuration(points[i - 1].JointState, points[i].JointState, limits, n, options);
            if (prevVel is not null && dt > 0)
            {
                var jerkDt = MinJerkDuration(points[i - 1].JointState, points[i].JointState, prevVel, limits, n, options, dt);
                dt = Math.Max(dt, jerkDt);
            }

            t += dt;
            retimed.Add(new TrajectoryPoint(t, points[i].JointState));

            prevVel = new double[n];
            for (var j = 0; j < n; j++)
                prevVel[j] = (points[i].JointState.Positions[j] - points[i - 1].JointState.Positions[j]) / dt;
        }

        return new Trajectory(geometric.Robot, retimed);
    }

    private static double MinSegmentDuration(
        JointState from, JointState to, IReadOnlyList<JointLimit> limits, int n, TrajectoryRetimerOptions options)
    {
        var maxDuration = 1e-3;
        for (var j = 0; j < n; j++)
        {
            var dq = Math.Abs(to.Positions[j] - from.Positions[j]);
            if (dq < 1e-12) continue;

            var vmax = limits[j].MaxVelocityRadiansPerSecond ?? options.DefaultMaxVelocityRadiansPerSecond;
            var amax = limits[j].MaxAccelerationRadiansPerSecondSquared ?? options.DefaultMaxAccelerationRadiansPerSecondSquared;
            if (vmax <= 0 || amax <= 0) continue;

            var tTri = 2.0 * Math.Sqrt(dq / amax);
            var tTrap = (dq / vmax) + (vmax / amax);
            maxDuration = Math.Max(maxDuration, dq < (vmax * vmax / amax) ? tTri : tTrap);
        }

        return maxDuration;
    }

    private static double MinJerkDuration(
        JointState from, JointState to, double[] prevVel, IReadOnlyList<JointLimit> limits, int n,
        TrajectoryRetimerOptions options, double dt)
    {
        var maxExtra = 0.0;
        for (var j = 0; j < n; j++)
        {
            var dq = to.Positions[j] - from.Positions[j];
            var vel = dq / dt;
            var acc = (vel - prevVel[j]) / dt;
            var jmax = options.DefaultMaxJerkRadiansPerSecondCubed;
            if (jmax <= 0) continue;
            var minDt = Math.Pow(Math.Abs(acc) / jmax, 1.0 / 3.0);
            maxExtra = Math.Max(maxExtra, minDt);
        }
        return maxExtra;
    }
}
