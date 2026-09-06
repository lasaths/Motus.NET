namespace Motus.Core;

public enum RetimerAlgorithm
{
    TotgLite,
    Totg,
    SegmentTrapezoid,
    Bottleneck
}

public sealed class TrajectoryRetimerOptions
{
    public RetimerAlgorithm Algorithm { get; init; } = RetimerAlgorithm.Bottleneck;
    /// <summary>
    /// Default velocity limit in the joint coordinate unit per second when a <see cref="JointLimit"/>
    /// omits <see cref="JointLimit.MaxVelocity"/>. For revolute axes this is rad/s; for prismatic
    /// or Stewart axes this is m/s. The historical property name is revolute-biased.
    /// </summary>
    public double DefaultMaxVelocityRadiansPerSecond { get; init; } = 1.5;
    /// <summary>
    /// Default acceleration limit in the joint coordinate unit per second squared when a
    /// <see cref="JointLimit"/> omits <see cref="JointLimit.MaxAcceleration"/>. Revolute axes use
    /// rad/s^2; prismatic or Stewart axes use m/s^2.
    /// </summary>
    public double DefaultMaxAccelerationRadiansPerSecondSquared { get; init; } = 3.0;
    public double DefaultMaxJerkRadiansPerSecondCubed { get; init; } = 15.0;
}

/// <summary>Joint-space retiming: segment trapezoid, path-wide bottleneck, or managed TOPP-RA-style TOTG.</summary>
public static class TrajectoryRetimer
{
    public static Trajectory Retime(Trajectory geometric, TrajectoryRetimerOptions? options = null)
    {
        options ??= new TrajectoryRetimerOptions();
        ValidateGeometricTrajectory(geometric);
        var points = geometric.Points;
        if (points.Count <= 1) return geometric;
        // Program dwells and attachment transitions are exact-stop boundaries. Retime each
        // motion block separately so velocity cannot carry through a grasp/release or WAIT.
        var boundaries = new SortedSet<int> { 0, points.Count - 1 };
        for (var i = 1; i < points.Count; i++)
        {
            if (IsDwell(points[i]))
            {
                boundaries.Add(i - 1);
                boundaries.Add(i);
            }
            if (points[i].SegmentIndex != points[i - 1].SegmentIndex &&
                points[i - 1].SegmentIndex is not null && (points[i - 1].BlendRadiusMeters ?? 0) == 0)
                boundaries.Add(i - 1);
        }
        foreach (var span in geometric.AttachSpans)
        {
            boundaries.Add(EventIndex(span.StartSeconds));
            boundaries.Add(EventIndex(span.EndSeconds));
        }

        var retimed = new List<TrajectoryPoint> { CopyWithTime(points[0], 0) };
        var indices = boundaries.ToArray();
        for (var b = 1; b < indices.Length; b++)
        {
            var first = indices[b - 1];
            var last = indices[b];
            var offset = retimed[^1].TimeSeconds;
            if (last == first + 1 && IsDwell(points[last]))
            {
                if (!points[first].JointState.Positions.SequenceEqual(points[last].JointState.Positions))
                    throw new ArgumentException("SET/WAIT dwell must hold joint position.", nameof(geometric));
                retimed.Add(CopyWithTime(points[last], offset + points[last].TimeSeconds - points[first].TimeSeconds));
                continue;
            }
            var block = new Trajectory(geometric.Robot, points.Skip(first).Take(last - first + 1)
                .Select(p => CopyWithTime(p, p.TimeSeconds - points[first].TimeSeconds)).ToArray());
            var timed = RetimeMotion(block, options);
            for (var i = 1; i < timed.Points.Count; i++)
                retimed.Add(CopyWithTime(timed.Points[i], offset + timed.Points[i].TimeSeconds));
        }

        var spans = geometric.AttachSpans.Select(span => new AttachTimeSpan(
            retimed[EventIndex(span.StartSeconds)].TimeSeconds,
            retimed[EventIndex(span.EndSeconds)].TimeSeconds, span.Bodies, span.ReleaseWorldPose)).ToArray();
        return new Trajectory(geometric.Robot, retimed, spans);

        int EventIndex(double seconds)
        {
            for (var i = 0; i < points.Count; i++)
                if (Math.Abs(points[i].TimeSeconds - seconds) <= 1e-9) return i;
            throw new ArgumentException("Attachment events must coincide with trajectory waypoints.", nameof(geometric));
        }
    }

    private static bool IsDwell(TrajectoryPoint point) =>
        point.MotionType is MotionPrimitiveType.Set or MotionPrimitiveType.Wait;

    private static Trajectory RetimeMotion(Trajectory geometric, TrajectoryRetimerOptions options)
    {
        if (options.Algorithm == RetimerAlgorithm.TotgLite || options.Algorithm == RetimerAlgorithm.Bottleneck)
            return RetimeBottleneck(geometric, options);
        if (options.Algorithm == RetimerAlgorithm.Totg)
            return RetimeTotg(geometric, options);
        return RetimeSegmentTrapezoid(geometric, options);
    }

    private static Trajectory RetimeTotg(Trajectory geometric, TrajectoryRetimerOptions options)
    {
        var points = geometric.Points;
        if (points.Count <= 1) return geometric;

        ValidateGeometricTrajectory(geometric);

        var limits = geometric.Robot.Preset.JointLimits;
        var n = limits.Count;
        var count = points.Count;
        var s = new double[count];
        var segmentAccel = new double[count - 1];
        var segmentVelocitySquared = new double[count - 1];

        for (var i = 1; i < count; i++)
        {
            var ds2 = 0.0;
            for (var j = 0; j < n; j++)
            {
                var dq = points[i].JointState.Positions[j] - points[i - 1].JointState.Positions[j];
                ds2 += dq * dq;
            }

            var ds = Math.Sqrt(ds2);
            s[i] = s[i - 1] + ds;
            if (ds < 1e-12)
            {
                segmentAccel[i - 1] = double.PositiveInfinity;
                segmentVelocitySquared[i - 1] = double.PositiveInfinity;
                continue;
            }

            var accel = double.PositiveInfinity;
            var velocitySq = double.PositiveInfinity;
            for (var j = 0; j < n; j++)
            {
                var dqds = Math.Abs((points[i].JointState.Positions[j] - points[i - 1].JointState.Positions[j]) / ds);
                if (dqds < 1e-12) continue;

                var vmax = PositiveLimit(
                    limits[j].MaxVelocityRadiansPerSecond,
                    options.DefaultMaxVelocityRadiansPerSecond,
                    $"joint {j + 1} velocity");
                var amax = PositiveLimit(
                    limits[j].MaxAccelerationRadiansPerSecondSquared,
                    options.DefaultMaxAccelerationRadiansPerSecondSquared,
                    $"joint {j + 1} acceleration");

                velocitySq = Math.Min(velocitySq, (vmax / dqds) * (vmax / dqds));
                accel = Math.Min(accel, amax / dqds);
            }

            segmentVelocitySquared[i - 1] = double.IsPositiveInfinity(velocitySq)
                ? options.DefaultMaxVelocityRadiansPerSecond * options.DefaultMaxVelocityRadiansPerSecond
                : velocitySq;
            segmentAccel[i - 1] = double.IsPositiveInfinity(accel)
                ? options.DefaultMaxAccelerationRadiansPerSecondSquared
                : accel;
        }

        if (s[^1] < 1e-12) return geometric;

        var vertexVelocitySquared = new double[count];
        for (var i = 0; i < count; i++)
        {
            if (i == 0 || i == count - 1)
            {
                vertexVelocitySquared[i] = 0;
                continue;
            }

            vertexVelocitySquared[i] = Math.Min(segmentVelocitySquared[i - 1], segmentVelocitySquared[i]);
        }

        var controllable = (double[])vertexVelocitySquared.Clone();
        controllable[^1] = 0;
        for (var i = count - 2; i >= 0; i--)
        {
            var ds = s[i + 1] - s[i];
            if (ds < 1e-12) continue;
            var accel = PositiveLimit(segmentAccel[i], options.DefaultMaxAccelerationRadiansPerSecondSquared, $"segment {i} acceleration");
            controllable[i] = Math.Min(controllable[i], controllable[i + 1] + 2.0 * accel * ds);
        }

        var x = new double[count];
        x[0] = 0;
        for (var i = 1; i < count; i++)
        {
            var ds = s[i] - s[i - 1];
            if (ds < 1e-12)
            {
                x[i] = Math.Min(controllable[i], vertexVelocitySquared[i]);
                continue;
            }

            var accel = PositiveLimit(segmentAccel[i - 1], options.DefaultMaxAccelerationRadiansPerSecondSquared, $"segment {i - 1} acceleration");
            var reachable = x[i - 1] + 2.0 * accel * ds;
            x[i] = Math.Min(Math.Min(vertexVelocitySquared[i], controllable[i]), reachable);
        }

        var retimed = new List<TrajectoryPoint>(count) { CopyWithTime(points[0], 0) };
        var t = 0.0;
        for (var i = 1; i < count; i++)
        {
            var ds = s[i] - s[i - 1];
            if (ds >= 1e-12)
            {
                var v0 = Math.Sqrt(Math.Max(0, x[i - 1]));
                var v1 = Math.Sqrt(Math.Max(0, x[i]));
                if (v0 + v1 > 1e-9)
                {
                    t += 2.0 * ds / (v0 + v1);
                }
                else
                {
                    var accel = PositiveLimit(segmentAccel[i - 1], options.DefaultMaxAccelerationRadiansPerSecondSquared, $"segment {i - 1} acceleration");
                    t += 2.0 * Math.Sqrt(ds / accel);
                }
            }

            if (!double.IsFinite(t))
                throw new InvalidOperationException("TOTG retiming produced a non-finite timestamp.");
            retimed.Add(CopyWithTime(points[i], t));
        }

        return new Trajectory(geometric.Robot, retimed);
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

        var retimed = new List<TrajectoryPoint>(count) { CopyWithTime(points[0], 0) };
        var t = 0.0;
        for (var i = 1; i < count; i++)
        {
            var ds = s[i] - s[i - 1];
            var vAvg = Math.Max(1e-3, (v[i - 1] + v[i]) * 0.5);
            t += ds / vAvg;
            retimed.Add(CopyWithTime(points[i], t));
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
        ValidateGeometricTrajectory(geometric);

        var retimed = new List<TrajectoryPoint>(points.Count) { CopyWithTime(points[0], 0) };
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
            retimed.Add(CopyWithTime(points[i], t));

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

    private static void ValidateGeometricTrajectory(Trajectory trajectory)
    {
        var limits = trajectory.Robot.Preset.JointLimits;
        for (var i = 0; i < trajectory.Points.Count; i++)
        {
            var time = trajectory.Points[i].TimeSeconds;
            if (!double.IsFinite(time) || time < 0 || (i > 0 && time < trajectory.Points[i - 1].TimeSeconds))
                throw new InvalidOperationException($"Trajectory point {i} has an invalid timestamp.");
            var q = trajectory.Points[i].JointState.Positions;
            if (q.Length != limits.Count)
                throw new InvalidOperationException(
                    $"Trajectory point {i} has {q.Length} axes; robot preset has {limits.Count} limits.");
            for (var j = 0; j < q.Length; j++)
            {
                if (!double.IsFinite(q[j]))
                    throw new InvalidOperationException($"Trajectory point {i}, joint {j + 1} is NaN/Inf.");
            }
        }
    }

    private static double PositiveLimit(double? configured, double fallback, string name)
    {
        var value = configured ?? fallback;
        if (!double.IsFinite(value) || value <= 0)
            throw new InvalidOperationException($"{name} limit must be finite and positive.");
        return value;
    }

    private static TrajectoryPoint CopyWithTime(TrajectoryPoint source, double timeSeconds) =>
        new(
            timeSeconds,
            source.JointState,
            source.MotionType,
            source.SegmentIndex,
            source.BlendRadiusMeters,
            source.ToolState,
            source.BaseFrameOverride);
}
