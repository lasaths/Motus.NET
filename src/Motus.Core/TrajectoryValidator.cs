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
        foreach (var pt in trajectory.Points)
        {
            if (pt.TimeSeconds < prevTime)
                errors.Add($"Non-monotonic time at t={pt.TimeSeconds:F4}s.");
            prevTime = pt.TimeSeconds;

            var val = pt.JointState.Validate(limits);
            if (!val.IsValid) errors.AddRange(val.Errors);
        }

        warnings.Add("TrajectoryValidator: timing and joint limits only; no collision check.");
        return errors.Count == 0 ? ValidationResult.Ok(warnings) : ValidationResult.Fail(errors, warnings);
    }
}
