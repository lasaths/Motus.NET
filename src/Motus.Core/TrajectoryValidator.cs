namespace Motus.Core;

public sealed class TrajectoryValidationOptions
{
    public ICollisionChecker? CollisionChecker { get; init; }
    public CollisionScene? CollisionScene { get; init; }
    public bool CheckAcceleration { get; init; } = true;
}

public sealed class TrajectoryValidator : ITrajectoryValidator
{
    public ValidationResult Validate(Trajectory trajectory) =>
        Validate(trajectory, null);

    public ValidationResult Validate(Trajectory trajectory, TrajectoryValidationOptions? options)
    {
        var errors = new List<string>();
        var warnings = new List<string>();
        var limits = trajectory.Robot.Preset.JointLimits;
        var collision = options?.CollisionChecker;
        var scene = options?.CollisionScene ?? new CollisionScene();
        var checkAccel = options?.CheckAcceleration ?? true;

        if (trajectory.Points.Count == 0)
        {
            errors.Add("Trajectory has no points.");
            return ValidationResult.Fail(errors);
        }

        var prevTime = -1.0;
        TrajectoryPoint? prevPt = null;
        double[]? prevVel = null;
        foreach (var pt in trajectory.Points)
        {
            if (pt.TimeSeconds < prevTime)
                errors.Add($"Non-monotonic time at t={pt.TimeSeconds:F4}s.");
            prevTime = pt.TimeSeconds;

            var val = pt.JointState.Validate(limits);
            if (!val.IsValid) errors.AddRange(val.Errors);

            if (collision is not null && scene.Objects.Count > 0)
            {
                if (!collision.IsCollisionFree(pt.JointState, scene))
                    errors.Add($"Collision at t={pt.TimeSeconds:F4}s.");
                if (prevPt is not null && !collision.SegmentCollisionFree(prevPt.JointState, pt.JointState, scene, 0.05))
                    errors.Add($"Collision between t={prevPt.TimeSeconds:F4}s and t={pt.TimeSeconds:F4}s.");
            }

            if (prevPt is not null)
            {
                var dt = pt.TimeSeconds - prevPt.TimeSeconds;
                if (dt > 0)
                {
                    var n = Math.Min(pt.JointState.AxisCount, limits.Count);
                    var vel = new double[n];
                    for (var j = 0; j < n; j++)
                    {
                        vel[j] = (pt.JointState.Positions[j] - prevPt.JointState.Positions[j]) / dt;
                        var maxVel = limits[j].MaxVelocityRadiansPerSecond;
                        if (maxVel is null or <= 0) continue;
                        if (Math.Abs(vel[j]) > maxVel.Value + 1e-9)
                            errors.Add($"Joint {j + 1} velocity {Math.Abs(vel[j]):F4} rad/s exceeds limit {maxVel.Value:F4} at t={pt.TimeSeconds:F4}s.");
                    }

                    if (checkAccel && prevVel is not null)
                    {
                        for (var j = 0; j < n; j++)
                        {
                            var maxAcc = limits[j].MaxAccelerationRadiansPerSecondSquared;
                            if (maxAcc is null or <= 0) continue;
                            var acc = (vel[j] - prevVel[j]) / dt;
                            if (Math.Abs(acc) > maxAcc.Value + 1e-9)
                                errors.Add($"Joint {j + 1} acceleration {Math.Abs(acc):F4} rad/s² exceeds limit {maxAcc.Value:F4} at t={pt.TimeSeconds:F4}s.");
                        }
                    }
                    prevVel = vel;
                }
            }
            prevPt = pt;
        }

        if (collision is null)
            warnings.Add("TrajectoryValidator: no collision checker supplied.");
        return errors.Count == 0 ? ValidationResult.Ok(warnings) : ValidationResult.Fail(errors, warnings);
    }
}
