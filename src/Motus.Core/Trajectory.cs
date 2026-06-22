namespace Motus.Core;

public sealed class TrajectoryPoint
{
    public double TimeSeconds { get; }
    public JointState JointState { get; }

    public TrajectoryPoint(double timeSeconds, JointState jointState)
    {
        TimeSeconds = timeSeconds;
        JointState = jointState;
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
