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
    /// Default velocity limit in rad/s for radian-unit joints when a <see cref="JointLimit"/>
    /// omits <see cref="JointLimit.MaxVelocity"/>.
    /// </summary>
    public double DefaultMaxVelocityRadiansPerSecond { get; init; } = 1.5;
    /// <summary>
    /// Default acceleration limit in rad/s² for radian-unit joints when a
    /// <see cref="JointLimit"/> omits <see cref="JointLimit.MaxAcceleration"/>.
    /// </summary>
    public double DefaultMaxAccelerationRadiansPerSecondSquared { get; init; } = 3.0;
    /// <summary>
    /// Radian-unit jerk setting (rad/s³) for the segment spacing heuristic. This does not
    /// construct or guarantee a continuous jerk-limited profile.
    /// </summary>
    public double DefaultMaxJerkRadiansPerSecondCubed { get; init; } = 15.0;
    /// <summary>Default velocity for meter-unit joints (m/s) when unset.</summary>
    public double DefaultMaxVelocityMetersPerSecond { get; init; } = 0.5;
    /// <summary>Default acceleration for meter-unit joints (m/s²) when unset.</summary>
    public double DefaultMaxAccelerationMetersPerSecondSquared { get; init; } = 1.0;
    /// <summary>Meter-unit jerk setting (m/s³) for the same segment spacing heuristic.</summary>
    public double DefaultMaxJerkMetersPerSecondCubed { get; init; } = 5.0;
}

/// <summary>Joint-space retiming: segment trapezoid, path-wide bottleneck, or managed TOPP-RA-style TOTG.</summary>
public static class TrajectoryRetimer
{
    public static Trajectory Retime(Trajectory geometric, TrajectoryRetimerOptions? options = null)
    {
        options ??= new TrajectoryRetimerOptions();
        ValidateGeometricTrajectory(geometric);
        ValidateDynamicLimits(geometric.Robot.Preset.JointLimits, options);
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

    private static Trajectory RetimeTotg(Trajectory geometric, TrajectoryRetimerOptions options) =>
        RetimePath(geometric, options, usePathWideBottleneck: false);

    private static Trajectory RetimeBottleneck(Trajectory geometric, TrajectoryRetimerOptions options) =>
        RetimePath(geometric, options, usePathWideBottleneck: true);

    private static Trajectory RetimePath(
        Trajectory geometric, TrajectoryRetimerOptions options, bool usePathWideBottleneck)
    {
        var points = geometric.Points;
        if (points.Count <= 1) return geometric;

        var limits = geometric.Robot.Preset.JointLimits;
        var n = limits.Count;
        var count = points.Count;
        var s = new double[count];
        var segmentAccel = new double[count - 1];
        var segmentVelocitySquared = new double[count - 1];

        for (var i = 1; i < count; i++)
        {
            // s is an arbitrary path parameter, including for mixed m/rad axes. Project each
            // axis's limits through dq/ds rather than treating the norm as meters or radians.
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
                    limits[j].MaxVelocity,
                    DefaultVelocity(limits[j], options),
                    $"joint {j + 1} velocity");
                var amax = PositiveLimit(
                    limits[j].MaxAcceleration,
                    DefaultAcceleration(limits[j], options),
                    $"joint {j + 1} acceleration");

                velocitySq = Math.Min(velocitySq, (vmax / dqds) * (vmax / dqds));
                accel = Math.Min(accel, amax / dqds);
            }

            segmentVelocitySquared[i - 1] = double.IsPositiveInfinity(velocitySq)
                ? DefaultVelocity(limits[0], options) * DefaultVelocity(limits[0], options)
                : velocitySq;
            segmentAccel[i - 1] = double.IsPositiveInfinity(accel)
                ? DefaultAcceleration(limits[0], options)
                : accel;
        }

        if (s[^1] < 1e-12) return geometric;

        if (usePathWideBottleneck)
        {
            var velocitySq = segmentVelocitySquared.Min();
            var accel = segmentAccel.Min();
            Array.Fill(segmentVelocitySquared, velocitySq);
            Array.Fill(segmentAccel, accel);
        }

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
            if (ds < 1e-12)
            {
                // Repeated waypoints cannot reset the speed or bypass the terminal stop.
                controllable[i] = Math.Min(controllable[i], controllable[i + 1]);
                continue;
            }
            var accel = PositiveLimit(segmentAccel[i], DefaultAcceleration(limits[0], options), $"segment {i} acceleration");
            controllable[i] = Math.Min(controllable[i], controllable[i + 1] + 2.0 * accel * ds);
        }

        var x = new double[count];
        x[0] = 0;
        for (var i = 1; i < count; i++)
        {
            var ds = s[i] - s[i - 1];
            if (ds < 1e-12)
            {
                x[i] = Math.Min(x[i - 1], Math.Min(controllable[i], vertexVelocitySquared[i]));
                continue;
            }

            var accel = PositiveLimit(segmentAccel[i - 1], DefaultAcceleration(limits[0], options), $"segment {i - 1} acceleration");
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
                    var accel = PositiveLimit(segmentAccel[i - 1], DefaultAcceleration(limits[0], options), $"segment {i - 1} acceleration");
                    var vmax = Math.Sqrt(segmentVelocitySquared[i - 1]);
                    // With rest at both sampled ends, insert the implicit peak/cruise phase.
                    // A triangular profile is valid only if its peak stays under vmax.
                    t += RestToRestDuration(ds, vmax, accel);
                }
            }

            if (!double.IsFinite(t))
                throw new InvalidOperationException("Path retiming produced a non-finite timestamp.");
            retimed.Add(CopyWithTime(points[i], t));
        }

        return new Trajectory(geometric.Robot, retimed);
    }

    private static Trajectory RetimeSegmentTrapezoid(Trajectory geometric, TrajectoryRetimerOptions options)
    {
        var points = geometric.Points;
        if (points.Count <= 1) return geometric;

        var limits = geometric.Robot.Preset.JointLimits;
        var n = limits.Count;
        // Aerial body poses have their own planning clock and no joint coordinates to retime.
        if (n == 0) return geometric;
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
            if (!double.IsFinite(t))
                throw new InvalidOperationException("Segment trapezoid retiming produced a non-finite timestamp.");
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

            var vmax = PositiveLimit(limits[j].MaxVelocity, DefaultVelocity(limits[j], options), $"joint {j + 1} velocity");
            var amax = PositiveLimit(limits[j].MaxAcceleration, DefaultAcceleration(limits[j], options), $"joint {j + 1} acceleration");
            maxDuration = Math.Max(maxDuration, RestToRestDuration(dq, vmax, amax));
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
            var jmax = PositiveLimit(null, DefaultJerk(limits[j], options), $"joint {j + 1} jerk");
            // Average-velocity change / dt² estimates jerk, so spacing scales as sqrt(Δv/j).
            // It is a finite-difference heuristic, not a continuous jerk-limited profile.
            var minDt = Math.Sqrt(Math.Abs(vel - prevVel[j]) / jmax);
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

    private static void ValidateDynamicLimits(IReadOnlyList<JointLimit> limits, TrajectoryRetimerOptions options)
    {
        for (var j = 0; j < limits.Count; j++)
        {
            PositiveLimit(limits[j].MaxVelocity, DefaultVelocity(limits[j], options), $"joint {j + 1} velocity");
            PositiveLimit(limits[j].MaxAcceleration, DefaultAcceleration(limits[j], options), $"joint {j + 1} acceleration");
            if (options.Algorithm == RetimerAlgorithm.SegmentTrapezoid)
                PositiveLimit(null, DefaultJerk(limits[j], options), $"joint {j + 1} jerk");
        }
    }

    private static double RestToRestDuration(double distance, double vmax, double amax) =>
        distance <= vmax * vmax / amax
            ? 2.0 * Math.Sqrt(distance / amax)
            : distance / vmax + vmax / amax;

    private static double DefaultVelocity(JointLimit limit, TrajectoryRetimerOptions options) =>
        limit.Unit == JointCoordinateUnit.Meters
            ? options.DefaultMaxVelocityMetersPerSecond
            : options.DefaultMaxVelocityRadiansPerSecond;

    private static double DefaultAcceleration(JointLimit limit, TrajectoryRetimerOptions options) =>
        limit.Unit == JointCoordinateUnit.Meters
            ? options.DefaultMaxAccelerationMetersPerSecondSquared
            : options.DefaultMaxAccelerationRadiansPerSecondSquared;

    private static double DefaultJerk(JointLimit limit, TrajectoryRetimerOptions options) =>
        limit.Unit == JointCoordinateUnit.Meters
            ? options.DefaultMaxJerkMetersPerSecondCubed
            : options.DefaultMaxJerkRadiansPerSecondCubed;

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
