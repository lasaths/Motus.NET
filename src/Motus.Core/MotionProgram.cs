namespace Motus.Core;

public enum MotionPrimitiveType
{
    Ptp,
    Lin,
    Circ,
    Set,
    Wait,
    Attach,
    Detach
}

public abstract class MotionSegment
{
    public MotionPrimitiveType Type { get; }
    public double BlendRadiusMeters { get; }
    /// <summary>Target tool state at segment end (Ramp) or start (Instant).</summary>
    public EndEffectorState? TargetState { get; init; }
    public ToolStateMode ToolStateMode { get; init; } = ToolStateMode.Hold;

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

/// <summary>Hold arm pose and change tool state (optional ramp duration).</summary>
public sealed class SetToolStateSegment : MotionSegment
{
    public EndEffectorState State { get; }
    public double DurationSeconds { get; }

    public SetToolStateSegment(EndEffectorState state, double durationSeconds = 0)
        : base(MotionPrimitiveType.Set, 0)
    {
        State = state;
        DurationSeconds = Math.Max(0, durationSeconds);
    }
}

/// <summary>Dwell at current arm and tool state.</summary>
public sealed class WaitSegment : MotionSegment
{
    public double DurationSeconds { get; }

    public WaitSegment(double durationSeconds)
        : base(MotionPrimitiveType.Wait, 0)
    {
        DurationSeconds = Math.Max(0, durationSeconds);
    }
}

/// <summary>Zero-time: attach geometry to TCP (hides matching scene name when present).</summary>
public sealed class AttachSegment : MotionSegment
{
    public string Name { get; }
    public Frame TcpLocal { get; }
    /// <summary>World- or identity-posed collision volume; planner stores TCP-local identity copy.</summary>
    public CollisionObject Geometry { get; }

    public AttachSegment(string name, Frame tcpLocal, CollisionObject geometry)
        : base(MotionPrimitiveType.Attach, 0)
    {
        if (string.IsNullOrWhiteSpace(name))
            throw new ArgumentException("Attach name is required.", nameof(name));
        Name = name;
        TcpLocal = tcpLocal;
        Geometry = geometry ?? throw new ArgumentNullException(nameof(geometry));
    }
}

/// <summary>Zero-time: detach named body and place its geometry at <see cref="WorldPose"/>.</summary>
public sealed class DetachSegment : MotionSegment
{
    public string Name { get; }
    public Frame WorldPose { get; }

    public DetachSegment(string name, Frame worldPose)
        : base(MotionPrimitiveType.Detach, 0)
    {
        if (string.IsNullOrWhiteSpace(name))
            throw new ArgumentException("Detach name is required.", nameof(name));
        Name = name;
        WorldPose = worldPose;
    }
}

public sealed class MotionProgramRequest
{
    public RobotModel Robot { get; }
    public JointState Start { get; }
    public IReadOnlyList<MotionSegment> Segments { get; }
    public PlanningOptions Options { get; }
    public EndEffectorState? InitialToolState { get; init; }
    public ToolCapabilities? ToolCapabilities { get; init; }
    public ToolDefinition? SessionTool { get; init; }

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
