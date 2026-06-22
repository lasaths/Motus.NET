namespace Motus.Core;

public sealed class TrajectoryValidator : ITrajectoryValidator
{
    public ValidationResult Validate(Trajectory trajectory)
    {
        var errors = new List<string>();
        var warnings = new List<string>();
        var limits = trajectory.Robot.Preset.JointLimits;

        if (trajectory.Points.Count == 0)
        {
            errors.Add("Trajectory has no points.");
            return ValidationResult.Fail(errors);
        }

        var prevTime = -1.0;
        TrajectoryPoint? prevPt = null;
        foreach (var pt in trajectory.Points)
        {
            if (pt.TimeSeconds < prevTime)
                errors.Add($"Non-monotonic time at t={pt.TimeSeconds:F4}s.");
            prevTime = pt.TimeSeconds;

            var val = pt.JointState.Validate(limits);
            if (!val.IsValid) errors.AddRange(val.Errors);

            if (prevPt is not null)
            {
                var dt = pt.TimeSeconds - prevPt.TimeSeconds;
                if (dt > 0)
                {
                    var n = Math.Min(pt.JointState.AxisCount, limits.Count);
                    for (var j = 0; j < n; j++)
                    {
                        var maxVel = limits[j].MaxVelocityRadiansPerSecond;
                        if (maxVel is null or <= 0) continue;
                        var vel = Math.Abs(pt.JointState.Positions[j] - prevPt.JointState.Positions[j]) / dt;
                        if (vel > maxVel.Value + 1e-9)
                            errors.Add($"Joint {j + 1} velocity {vel:F4} rad/s exceeds limit {maxVel.Value:F4} at t={pt.TimeSeconds:F4}s.");
                    }
                }
            }
            prevPt = pt;
        }

        warnings.Add("TrajectoryValidator: timing and joint limits only; no collision check.");
        return errors.Count == 0 ? ValidationResult.Ok(warnings) : ValidationResult.Fail(errors, warnings);
    }
}
