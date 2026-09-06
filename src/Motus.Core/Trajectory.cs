namespace Motus.Core;

public sealed class TrajectoryPoint
{
    public double TimeSeconds { get; }
    public JointState JointState { get; }
    /// <summary>
    /// Optional world base pose for mobile-base planning samples. Null means use
    /// <see cref="RobotPreset.BaseFrame"/> from the trajectory robot.
    /// </summary>
    public BaseFrame? BaseFrameOverride { get; }
    public MotionPrimitiveType? MotionType { get; }
    public int? SegmentIndex { get; }
    public double? BlendRadiusMeters { get; }
    public EndEffectorState? ToolState { get; }

    public TrajectoryPoint(
        double timeSeconds,
        JointState jointState,
        MotionPrimitiveType? motionType = null,
        int? segmentIndex = null,
        double? blendRadiusMeters = null,
        EndEffectorState? toolState = null,
        BaseFrame? baseFrameOverride = null)
    {
        TimeSeconds = timeSeconds;
        JointState = jointState;
        BaseFrameOverride = baseFrameOverride;
        MotionType = motionType;
        SegmentIndex = segmentIndex;
        BlendRadiusMeters = blendRadiusMeters;
        ToolState = toolState;
    }
}

public sealed class Trajectory
{
    public RobotModel Robot { get; }
    public IReadOnlyList<TrajectoryPoint> Points { get; }
    /// <summary>Attachment timeline on the same clock as Points; preserved by retiming and export.</summary>
    public IReadOnlyList<AttachTimeSpan> AttachSpans { get; }
    public double DurationSeconds => Points.Count == 0 ? 0 : Points[^1].TimeSeconds;

    public Trajectory(RobotModel robot, IReadOnlyList<TrajectoryPoint> points,
        IReadOnlyList<AttachTimeSpan>? attachSpans = null)
    {
        Robot = robot;
        Points = points;
        AttachSpans = attachSpans ?? Array.Empty<AttachTimeSpan>();
    }
}
