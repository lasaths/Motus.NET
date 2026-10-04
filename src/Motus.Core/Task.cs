using System.Collections.ObjectModel;

namespace Motus.Core;

/// <summary>
/// A task instance with a reusable string identity, an ordered set of named frames,
/// and metadata. Multiple instances can share the same identity while carrying different
/// frames and metadata.
/// </summary>
public sealed class TaskInstance
{
    /// <summary>Task identity (e.g., "pick", "place"). Reusable across many instances.</summary>
    public string Identity { get; }

    /// <summary>Named geometric frames (position and orientation in robot world).</summary>
    public IReadOnlyDictionary<string, Frame> Frames { get; }

    /// <summary>Named metadata values (e.g., object name, jaw opening).</summary>
    public IReadOnlyDictionary<string, object> Metadata { get; }

    public TaskInstance(string identity, IDictionary<string, Frame> frames, IDictionary<string, object>? metadata = null)
    {
        if (string.IsNullOrWhiteSpace(identity))
            throw new ArgumentException("Identity cannot be null or whitespace.", nameof(identity));

        Identity = identity;
        Frames = new ReadOnlyDictionary<string, Frame>(new Dictionary<string, Frame>(frames ?? new Dictionary<string, Frame>()));
        Metadata = new ReadOnlyDictionary<string, object>(metadata != null
            ? new Dictionary<string, object>(metadata)
            : new Dictionary<string, object>());
    }

    /// <summary>
    /// Create a new task instance with the same identity but updated frames.
    /// </summary>
    public TaskInstance WithFrames(IDictionary<string, Frame> frames)
    {
        return new TaskInstance(Identity, frames, Metadata.ToDictionary(kv => kv.Key, kv => kv.Value));
    }

    /// <summary>
    /// Create a new task instance with the same identity but updated metadata.
    /// </summary>
    public TaskInstance WithMetadata(IDictionary<string, object> metadata)
    {
        return new TaskInstance(Identity, Frames.ToDictionary(kv => kv.Key, kv => kv.Value), metadata);
    }

    /// <summary>
    /// Create a new task instance with the same identity, existing frames/metadata, and one updated frame.
    /// </summary>
    public TaskInstance WithFrame(string name, Frame frame)
    {
        var updatedFrames = Frames.ToDictionary(kv => kv.Key, kv => kv.Value);
        updatedFrames[name] = frame;
        return new TaskInstance(Identity, updatedFrames, Metadata.ToDictionary(kv => kv.Key, kv => kv.Value));
    }

    /// <summary>
    /// Create a new task instance with the same identity, existing frames/metadata, and one updated metadata value.
    /// </summary>
    public TaskInstance WithMetadataValue(string name, object value)
    {
        var updatedMetadata = Metadata.ToDictionary(kv => kv.Key, kv => kv.Value);
        updatedMetadata[name] = value;
        return new TaskInstance(Identity, Frames.ToDictionary(kv => kv.Key, kv => kv.Value), updatedMetadata);
    }

    public override string ToString() => $"Task[{Identity}] ({Frames.Count} frames, {Metadata.Count} metadata)";
}

/// <summary>
/// An ordered list of task instances representing a robot task sequence.
/// </summary>
public sealed class TaskPath
{
    public IReadOnlyList<TaskInstance> Tasks { get; }

    public TaskPath(IEnumerable<TaskInstance> tasks)
    {
        Tasks = new List<TaskInstance>(tasks).AsReadOnly();
    }

    public TaskPath(params TaskInstance[] tasks)
    {
        Tasks = new List<TaskInstance>(tasks).AsReadOnly();
    }

    public override string ToString() => $"TaskPath[{Tasks.Count} tasks]";
}

/// <summary>
/// Validates task paths for collision-free reachability.
/// </summary>
public static class TaskPathValidator
{
    /// <summary>
    /// Check if all task frames in a path are reachable and collision-free.
    /// </summary>
    public static TaskPathValidationResult Validate(
        TaskPath path,
        RobotModel robot,
        IInverseKinematics ik,
        ICollisionChecker checker,
        CollisionScene scene,
        BaseFrame? baseFrame = null,
        ToolFrame? toolFrame = null,
        JointState? seed = null)
    {
        baseFrame ??= robot.Preset.BaseFrame;
        toolFrame ??= robot.Preset.ToolFrame;
        seed ??= new JointState(new double[robot.Preset.AxisCount]);

        var errors = new List<string>();
        var reachableStates = new List<JointState>();

        foreach (var task in path.Tasks)
        {
            foreach (var (frameName, frame) in task.Frames)
            {
                var targetPose = new CartesianPose(frame);
                
                if (!ik.TrySolve(targetPose, seed, out var solution))
                {
                    errors.Add($"Task '{task.Identity}' frame '{frameName}' unreachable: IK failed at {frame}");
                    continue;
                }

                if (!solution.Validate(robot.Preset.JointLimits).IsValid)
                {
                    errors.Add($"Task '{task.Identity}' frame '{frameName}' violates joint limits");
                    continue;
                }

                if (!checker.IsCollisionFree(solution, scene))
                {
                    errors.Add($"Task '{task.Identity}' frame '{frameName}' collides at {frame}");
                    continue;
                }

                reachableStates.Add(solution);
                seed = solution;
            }
        }

        return new TaskPathValidationResult(errors.Count == 0, errors, reachableStates);
    }

    /// <summary>
    /// Check if segments between consecutive task frames are collision-free.
    /// </summary>
    public static TaskPathValidationResult ValidateWithMotion(
        TaskPath path,
        RobotModel robot,
        IInverseKinematics ik,
        ICollisionChecker checker,
        CollisionScene scene,
        double configurationStep = 0.05,
        BaseFrame? baseFrame = null,
        ToolFrame? toolFrame = null,
        JointState? seed = null)
    {
        var frameCheck = Validate(path, robot, ik, checker, scene, baseFrame, toolFrame, seed);
        if (!frameCheck.IsValid)
            return frameCheck;

        var errors = new List<string>(frameCheck.Errors);
        var states = frameCheck.ReachableStates;

        for (var i = 1; i < states.Count; i++)
        {
            if (!checker.SegmentCollisionFree(states[i - 1], states[i], scene, configurationStep))
            {
                errors.Add($"Segment collision between frame {i - 1} and {i}");
            }
        }

        return new TaskPathValidationResult(errors.Count == 0, errors, states);
    }
}

/// <summary>
/// Result of task path validation.
/// </summary>
public sealed class TaskPathValidationResult
{
    public bool IsValid { get; }
    public IReadOnlyList<string> Errors { get; }
    public IReadOnlyList<JointState> ReachableStates { get; }

    public TaskPathValidationResult(bool isValid, IReadOnlyList<string> errors, IReadOnlyList<JointState> reachableStates)
    {
        IsValid = isValid;
        Errors = errors;
        ReachableStates = reachableStates;
    }
}
