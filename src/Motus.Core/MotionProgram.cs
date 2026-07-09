namespace Motus.Core;

public enum MotionPrimitiveType
{
    Ptp,
    Lin,
    Circ
}

public abstract class MotionSegment
{
    public MotionPrimitiveType Type { get; }
    public double BlendRadiusMeters { get; }

    protected MotionSegment(MotionPrimitiveType type, double blendRadiusMeters)
    {
        Type = type;
        BlendRadiusMeters = Math.Max(0, blendRadiusMeters);
    }
}

public sealed class PtpSegment : MotionSegment
{
    public JointState Goal { get; }

    public PtpSegment(JointState goal, double blendRadiusMeters = 0)
        : base(MotionPrimitiveType.Ptp, blendRadiusMeters)
    {
        Goal = goal;
    }
}

public sealed class LinSegment : MotionSegment
{
    public CartesianPose Goal { get; }
    public double StepMeters { get; }

    public LinSegment(CartesianPose goal, double stepMeters = 0.005, double blendRadiusMeters = 0)
        : base(MotionPrimitiveType.Lin, blendRadiusMeters)
    {
        Goal = goal;
        StepMeters = stepMeters;
    }
}

public sealed class CircSegment : MotionSegment
{
    public CartesianPose Via { get; }
    public CartesianPose Goal { get; }
    public int ArcSamples { get; }

    public CircSegment(CartesianPose via, CartesianPose goal, int arcSamples = 16, double blendRadiusMeters = 0)
        : base(MotionPrimitiveType.Circ, blendRadiusMeters)
    {
        Via = via;
        Goal = goal;
        ArcSamples = arcSamples;
    }
}

public sealed class MotionProgramRequest
{
    public RobotModel Robot { get; }
    public JointState Start { get; }
    public IReadOnlyList<MotionSegment> Segments { get; }
    public PlanningOptions Options { get; }

    public MotionProgramRequest(
        RobotModel robot,
        JointState start,
        IReadOnlyList<MotionSegment> segments,
        PlanningOptions? options = null)
    {
        Robot = robot;
        Start = start;
        Segments = segments;
        Options = options ?? new PlanningOptions();
    }
}
