namespace Motus.Core;

/// <summary>Time window when attached bodies ride the TCP (for preview / hosts).</summary>
public sealed class AttachTimeSpan
{
    public double StartSeconds { get; }
    public double EndSeconds { get; }
    public IReadOnlyList<AttachedBody> Bodies { get; }
    /// <summary>World pose of the workpiece after Detach (preview should use this, not TCP at EndSeconds).</summary>
    public Frame? ReleaseWorldPose { get; }

    public AttachTimeSpan(
        double startSeconds,
        double endSeconds,
        IReadOnlyList<AttachedBody> bodies,
        Frame? releaseWorldPose = null)
    {
        StartSeconds = startSeconds;
        EndSeconds = Math.Max(endSeconds, startSeconds);
        Bodies = bodies ?? Array.Empty<AttachedBody>();
        ReleaseWorldPose = releaseWorldPose;
    }
}
