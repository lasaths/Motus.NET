namespace Motus.Core;

/// <summary>Deterministic joint-space linear interpolation planner. No collision checking.</summary>
public sealed class JointLinearPlanner : IPlanner
{
    public PlanningResult Plan(PlanningRequest request)
    {
        var warnings = new List<string>();
        var errors = new List<string>();
        var robot = request.Robot;
        var opts = request.Options;

        if (request.Start.AxisCount != robot.Preset.AxisCount)
            errors.Add($"Start state has {request.Start.AxisCount} joints; robot expects {robot.Preset.AxisCount}.");
        if (request.Goal.AxisCount != robot.Preset.AxisCount)
            errors.Add($"Goal state has {request.Goal.AxisCount} joints; robot expects {robot.Preset.AxisCount}.");
        if (errors.Count > 0) return PlanningResult.Failed(errors);

        var startVal = request.Start.Validate(robot.Preset.JointLimits);
        var goalVal = request.Goal.Validate(robot.Preset.JointLimits);
        if (!startVal.IsValid) errors.AddRange(startVal.Errors.Select(e => $"Start: {e}"));
        if (!goalVal.IsValid) errors.AddRange(goalVal.Errors.Select(e => $"Goal: {e}"));
        if (errors.Count > 0) return PlanningResult.Failed(errors);

        if (opts.MaxJointStepRadians <= 0)
            return PlanningResult.Failed(new[] { "MaxJointStepRadians must be positive." });
        if (opts.TimeStepSeconds <= 0)
            return PlanningResult.Failed(new[] { "TimeStepSeconds must be positive." });

        var n = robot.Preset.AxisCount;
        var deltas = new double[n];
        var maxSteps = 1;
        for (var i = 0; i < n; i++)
        {
            deltas[i] = request.Goal.Positions[i] - request.Start.Positions[i];
            var steps = (int)Math.Ceiling(Math.Abs(deltas[i]) / opts.MaxJointStepRadians);
            if (steps > maxSteps) maxSteps = steps;
        }

        var points = new List<TrajectoryPoint>(maxSteps + 1);
        var t = 0.0;
        for (var s = 0; s <= maxSteps; s++)
        {
            var alpha = maxSteps == 0 ? 1.0 : (double)s / maxSteps;
            var pos = new double[n];
            for (var i = 0; i < n; i++)
                pos[i] = request.Start.Positions[i] + alpha * deltas[i];

            var state = new JointState(pos);
            var val = state.Validate(robot.Preset.JointLimits);
            if (!val.IsValid)
            {
                errors.Add($"Interpolated point at step {s} violates joint limits.");
                return PlanningResult.Failed(errors);
            }
            points.Add(new TrajectoryPoint(t, state));
            t += opts.TimeStepSeconds;
        }

        warnings.Add("JointLinearPlanner: no collision checking.");
        return PlanningResult.Succeeded(new Trajectory(robot, points), warnings);
    }
}
