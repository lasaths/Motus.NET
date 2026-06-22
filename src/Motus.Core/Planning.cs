namespace Motus.Core;

public sealed class PlanningOptions
{
    public double MaxJointStepRadians { get; init; } = 0.05;
    public double TimeStepSeconds { get; init; } = 0.04;
    public double MaxJointVelocityRadiansPerSecond { get; init; } = 1.5;
}

public sealed class PlanningRequest
{
    public RobotModel Robot { get; }
    public JointState Start { get; }
    public JointState Goal { get; }
    public PlanningOptions Options { get; }

    public PlanningRequest(RobotModel robot, JointState start, JointState goal, PlanningOptions? options = null)
    {
        Robot = robot;
        Start = start;
        Goal = goal;
        Options = options ?? new PlanningOptions();
    }
}

public sealed class PlanningResult
{
    public bool Success { get; }
    public Trajectory? Trajectory { get; }
    public IReadOnlyList<string> Errors { get; }
    public IReadOnlyList<string> Warnings { get; }

    private PlanningResult(bool success, Trajectory? trajectory, IReadOnlyList<string> errors, IReadOnlyList<string> warnings)
    {
        Success = success;
        Trajectory = trajectory;
        Errors = errors;
        Warnings = warnings;
    }

    public static PlanningResult Succeeded(Trajectory trajectory, IReadOnlyList<string>? warnings = null) =>
        new(true, trajectory, Array.Empty<string>(), warnings ?? Array.Empty<string>());

    public static PlanningResult Failed(IEnumerable<string> errors, IEnumerable<string>? warnings = null) =>
        new(false, null, errors.ToList(), warnings?.ToList() ?? new List<string>());
}
