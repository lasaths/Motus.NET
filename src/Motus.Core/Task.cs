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
        Frames = new ReadOnlyDictionary<string, Frame>(new Dictionary<string, Frame>(frames));
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
