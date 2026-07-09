namespace Motus.Core;

public sealed class TrajectoryPoint
{
    public double TimeSeconds { get; }
    public JointState JointState { get; }
    public MotionPrimitiveType? MotionType { get; }
    public int? SegmentIndex { get; }
    public double? BlendRadiusMeters { get; }

    public TrajectoryPoint(
        double timeSeconds,
        JointState jointState,
        MotionPrimitiveType? motionType = null,
        int? segmentIndex = null,
        double? blendRadiusMeters = null)
    {
        TimeSeconds = timeSeconds;
        JointState = jointState;
        MotionType = motionType;
        SegmentIndex = segmentIndex;
        BlendRadiusMeters = blendRadiusMeters;
    }
}

public sealed class Trajectory
{
    public RobotModel Robot { get; }
    public IReadOnlyList<TrajectoryPoint> Points { get; }
    public double DurationSeconds => Points.Count == 0 ? 0 : Points[^1].TimeSeconds;

    public Trajectory(RobotModel robot, IReadOnlyList<TrajectoryPoint> points)
    {
        Robot = robot;
        Points = points;
    }
}
