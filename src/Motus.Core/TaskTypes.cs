namespace Motus.Core;

/// <summary>
/// Core task types on top of the string-identity <see cref="TaskInstance"/> model: pick, place, walk (node / land modes,
/// gait) and park (relay). Generic over robot and scene — objects, clamp targets and connectors are plain names, frames
/// are optional; a robot/scene binding (e.g. the bamboo-truss planner) interprets them.
/// </summary>
public static class TaskIdentities
{
    public const string Pick = "pick";
    public const string Place = "place";
    public const string Walk = "walk";
    public const string Park = "park";

    public static readonly IReadOnlyList<string> All = new[] { Pick, Place, Walk, Park };
}

/// <summary>Metadata keys used by the core task types.</summary>
public static class TaskMetadataKeys
{
    /// <summary>Object handled (pick / place / park) or walked onto (walk: target clamp object).</summary>
    public const string Object = "object";
    /// <summary>Gripper jaw opening at the end of the task, mm.</summary>
    public const string JawMm = "jawMm";
    /// <summary>Walk mode, see <see cref="WalkMode"/>.</summary>
    public const string Mode = "mode";
    /// <summary>Walk gait name (robot-specific; null = default).</summary>
    public const string Gait = "gait";
    /// <summary>Park: connector (node) the object is parked in.</summary>
    public const string Connector = "connector";
    /// <summary>Park: direction the parked object points away from the connector (robot/scene-specific token).</summary>
    public const string Direction = "direction";
}

/// <summary>How a walk block chooses where it lands.</summary>
public enum WalkMode
{
    /// <summary>Walk to a given clamp target (object + point) by a graph search over steps (a gait).</summary>
    Node,
    /// <summary>One targeted step to a landing chosen for the next task's object (point + roll from the reach map).</summary>
    Land
}

/// <summary>Typed view of a pick task.</summary>
public sealed record PickTask(string Object, double JawMm, IReadOnlyDictionary<string, Frame> Frames);
/// <summary>Typed view of a place task.</summary>
public sealed record PlaceTask(string Object, double JawMm, IReadOnlyDictionary<string, Frame> Frames);
/// <summary>Typed view of a walk task. <c>Target</c>: clamp object (node mode) or the object to land for (land mode).</summary>
public sealed record WalkTask(string Target, WalkMode Mode, string? Gait, IReadOnlyDictionary<string, Frame> Frames);
/// <summary>Typed view of a park (relay) task: the held object is set into <c>Connector</c>, pointing <c>Direction</c>.</summary>
public sealed record ParkTask(string Object, string Connector, string Direction, double JawMm, IReadOnlyDictionary<string, Frame> Frames);

/// <summary>Factories and typed views for the core task types.</summary>
public static class CoreTasks
{
    static Dictionary<string, Frame> F(IDictionary<string, Frame>? frames) => frames is null ? new() : new(frames);

    public static TaskInstance Pick(string obj, double jawMm, IDictionary<string, Frame>? frames = null) =>
        new(TaskIdentities.Pick, F(frames), new Dictionary<string, object> { [TaskMetadataKeys.Object] = obj, [TaskMetadataKeys.JawMm] = jawMm });

    public static TaskInstance Place(string obj, double jawMm, IDictionary<string, Frame>? frames = null) =>
        new(TaskIdentities.Place, F(frames), new Dictionary<string, object> { [TaskMetadataKeys.Object] = obj, [TaskMetadataKeys.JawMm] = jawMm });

    public static TaskInstance Walk(string target, WalkMode mode = WalkMode.Node, string? gait = null, IDictionary<string, Frame>? frames = null)
    {
        var meta = new Dictionary<string, object> { [TaskMetadataKeys.Object] = target, [TaskMetadataKeys.Mode] = mode.ToString().ToLowerInvariant() };
        if (gait is not null) meta[TaskMetadataKeys.Gait] = gait;
        return new TaskInstance(TaskIdentities.Walk, F(frames), meta);
    }

    public static TaskInstance Park(string obj, string connector, string direction, double jawMm, IDictionary<string, Frame>? frames = null) =>
        new(TaskIdentities.Park, F(frames), new Dictionary<string, object>
        {
            [TaskMetadataKeys.Object] = obj, [TaskMetadataKeys.Connector] = connector,
            [TaskMetadataKeys.Direction] = direction, [TaskMetadataKeys.JawMm] = jawMm
        });

    public static string ObjectOf(TaskInstance t) => Str(t, TaskMetadataKeys.Object) ?? "";

    public static PickTask? AsPick(TaskInstance t) => t.Identity == TaskIdentities.Pick ? new(ObjectOf(t), Num(t, TaskMetadataKeys.JawMm), t.Frames) : null;
    public static PlaceTask? AsPlace(TaskInstance t) => t.Identity == TaskIdentities.Place ? new(ObjectOf(t), Num(t, TaskMetadataKeys.JawMm), t.Frames) : null;

    public static WalkTask? AsWalk(TaskInstance t)
    {
        if (t.Identity != TaskIdentities.Walk) return null;
        var mode = Enum.TryParse<WalkMode>(Str(t, TaskMetadataKeys.Mode), ignoreCase: true, out var m) ? m : WalkMode.Node;
        return new WalkTask(ObjectOf(t), mode, Str(t, TaskMetadataKeys.Gait), t.Frames);
    }

    public static ParkTask? AsPark(TaskInstance t) => t.Identity == TaskIdentities.Park
        ? new(ObjectOf(t), Str(t, TaskMetadataKeys.Connector) ?? "", Str(t, TaskMetadataKeys.Direction) ?? "", Num(t, TaskMetadataKeys.JawMm), t.Frames)
        : null;

    static string? Str(TaskInstance t, string key) => t.Metadata.TryGetValue(key, out var v) ? v?.ToString() : null;
    static double Num(TaskInstance t, string key) => t.Metadata.TryGetValue(key, out var v) && v is IConvertible c ? c.ToDouble(System.Globalization.CultureInfo.InvariantCulture) : 0;
}

/// <summary>
/// Robot-independent sequence rules for the core task types: one object in hand at a time; pick is followed by place or
/// park of the same object; never walk while carrying; a parked object stays parked until it is picked again (from the
/// connector) and must be placed in the end.
/// </summary>
public static class TaskSequenceRules
{
    public sealed record Problem(int Index, string Message);

    public static List<Problem> Validate(IEnumerable<TaskInstance> tasks)
    {
        var problems = new List<Problem>();
        string? held = null;
        var parked = new HashSet<string>();
        var placed = new HashSet<string>();
        var i = -1;
        foreach (var t in tasks)
        {
            i++;
            var obj = CoreTasks.ObjectOf(t);
            switch (t.Identity)
            {
                case TaskIdentities.Pick:
                    if (held is not null) problems.Add(new(i, $"pick {obj} while still holding {held}"));
                    if (placed.Contains(obj)) problems.Add(new(i, $"pick {obj}: already placed"));
                    held = obj;
                    parked.Remove(obj);
                    break;
                case TaskIdentities.Place:
                    if (held != obj) problems.Add(new(i, held is null ? $"place {obj} without picking it" : $"place {obj} while holding {held}"));
                    held = null;
                    placed.Add(obj);
                    break;
                case TaskIdentities.Park:
                    if (held != obj) problems.Add(new(i, held is null ? $"park {obj} without picking it" : $"park {obj} while holding {held}"));
                    var p = CoreTasks.AsPark(t)!;
                    if (string.IsNullOrEmpty(p.Connector)) problems.Add(new(i, $"park {obj}: no connector"));
                    held = null;
                    parked.Add(obj);
                    break;
                case TaskIdentities.Walk:
                    if (held is not null) problems.Add(new(i, $"walk while carrying {held}"));
                    break;
            }
        }
        if (held is not null) problems.Add(new(i, $"sequence ends holding {held}"));
        foreach (var o in parked) problems.Add(new(i, $"{o} is still parked at the end"));
        return problems;
    }
}
