namespace Motus.Core;

public enum PlanningMessageSeverity
{
    Info,
    Warning,
    Error
}

public sealed class PlanningMessage
{
    public string Code { get; }
    public string Message { get; }
    public PlanningMessageSeverity Severity { get; }

    public PlanningMessage(string code, string message, PlanningMessageSeverity severity)
    {
        Code = code;
        Message = message;
        Severity = severity;
    }
}

public static class PlanningMessageCodes
{
    public const string EndpointCollision = "planning.endpoint_collision";
    public const string PathCollision = "planning.path_collision";
    public const string InvalidOptions = "planning.invalid_options";
    public const string InvalidStart = "planning.invalid_start";
    public const string InvalidGoal = "planning.invalid_goal";
    public const string PlannerUnavailable = "planning.planner_unavailable";
    public const string PlannerFallback = "planning.planner_fallback";
    public const string PlannerWarning = "planning.warning";
}

public sealed class PlanningOptions
{
    public double MaxJointStepRadians { get; init; } = 0.05;
    public double TimeStepSeconds { get; init; } = 0.04;
    public double MaxJointVelocityRadiansPerSecond { get; init; } = 1.5;
    public CollisionScene? CollisionScene { get; init; }
    public ICollisionChecker? CollisionChecker { get; init; }
    public IReadOnlyList<AttachedBody>? AttachedBodies { get; init; }
    /// <summary>When set, planners vary only mapped joints; others stay at <see cref="PlanningRequest.Start"/>.</summary>
    public JointIndexMap? GroupMap { get; init; }
    public bool RetimeTrajectory { get; init; }
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
    public IReadOnlyList<PlanningMessage> Messages { get; }

    private PlanningResult(
        bool success,
        Trajectory? trajectory,
        IReadOnlyList<string> errors,
        IReadOnlyList<string> warnings,
        IReadOnlyList<PlanningMessage> messages)
    {
        Success = success;
        Trajectory = trajectory;
        Errors = errors;
        Warnings = warnings;
        Messages = messages;
    }

    public static PlanningResult Succeeded(Trajectory trajectory, IReadOnlyList<string>? warnings = null) =>
        new(
            true,
            trajectory,
            Array.Empty<string>(),
            warnings ?? Array.Empty<string>(),
            BuildSuccessMessages(warnings));

    public static PlanningResult Succeeded(Trajectory trajectory, IReadOnlyList<PlanningMessage> messages) =>
        new(
            true,
            trajectory,
            Array.Empty<string>(),
            messages.Where(m => m.Severity == PlanningMessageSeverity.Warning).Select(m => m.Message).ToList(),
            messages);

    public static PlanningResult Failed(IEnumerable<string> errors, IEnumerable<string>? warnings = null) =>
        new(
            false,
            null,
            errors.ToList(),
            warnings?.ToList() ?? new List<string>(),
            BuildFailureMessages(errors, warnings));

    public static PlanningResult Failed(IEnumerable<PlanningMessage> messages) =>
        new(
            false,
            null,
            messages.Where(m => m.Severity == PlanningMessageSeverity.Error).Select(m => m.Message).ToList(),
            messages.Where(m => m.Severity == PlanningMessageSeverity.Warning).Select(m => m.Message).ToList(),
            messages.ToList());

    private static IReadOnlyList<PlanningMessage> BuildSuccessMessages(IReadOnlyList<string>? warnings)
    {
        if (warnings is null || warnings.Count == 0) return Array.Empty<PlanningMessage>();
        return warnings
            .Select(w => new PlanningMessage(PlanningMessageCodes.PlannerWarning, w, PlanningMessageSeverity.Warning))
            .ToList();
    }

    private static IReadOnlyList<PlanningMessage> BuildFailureMessages(IEnumerable<string> errors, IEnumerable<string>? warnings)
    {
        var messages = errors
            .Select(e => new PlanningMessage(PlanningMessageCodes.PlannerUnavailable, e, PlanningMessageSeverity.Error))
            .ToList();
        if (warnings is not null)
        {
            messages.AddRange(
                warnings.Select(w => new PlanningMessage(PlanningMessageCodes.PlannerWarning, w, PlanningMessageSeverity.Warning)));
        }
        return messages;
    }
}
