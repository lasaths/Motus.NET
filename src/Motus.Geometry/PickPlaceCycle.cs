using Motus.Core;

namespace Motus.Geometry;

public sealed class PickPlaceOptions
{
    /// <summary>Use TransferSegment for travel to pickup hover and between pickup/place hovers.</summary>
    public bool UseSamplingTransfers { get; init; }
    /// <summary>Explicit gripper collision body names permitted to touch the current workpiece during approach/release.</summary>
    public IReadOnlyList<string> TouchBodies { get; init; } = Array.Empty<string>();
}

/// <summary>Expands one pick→place brick cycle into Motus Move segments (LIN/SET/Attach/Detach).</summary>
public static class PickPlaceCycle
{
    /// <summary>
    /// Hover → grasp → close → attach → lift → place hover → place → open → detach → retract.
    /// Hover/lift/retract offset world +Z by <paramref name="approachMeters"/>.
    /// </summary>
    public static IReadOnlyList<MotionSegment> Expand(
        CartesianPose grasp,
        CartesianPose place,
        double approachMeters,
        EndEffectorState open,
        EndEffectorState close,
        CollisionObject obj,
        double linStepMeters = 0.005,
        double setDurationSeconds = 0.1,
        PickPlaceOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(grasp);
        ArgumentNullException.ThrowIfNull(place);
        ArgumentNullException.ThrowIfNull(open);
        ArgumentNullException.ThrowIfNull(close);
        ArgumentNullException.ThrowIfNull(obj);
        if (!double.IsFinite(approachMeters) || approachMeters < 0)
            throw new ArgumentOutOfRangeException(nameof(approachMeters), "Approach height must be >= 0.");
        if (!double.IsFinite(linStepMeters) || linStepMeters <= 0)
            throw new ArgumentOutOfRangeException(nameof(linStepMeters), "LIN step must be > 0.");
        if (!double.IsFinite(setDurationSeconds) || setDurationSeconds < 0)
            throw new ArgumentOutOfRangeException(nameof(setDurationSeconds));

        options ??= new PickPlaceOptions();
        if (options.TouchBodies.Any(string.IsNullOrWhiteSpace))
            throw new ArgumentException("Touch body names must not be empty.", nameof(options));
        var contacts = options.TouchBodies.Select(name => (name, obj.Name)).ToArray();

        var graspHover = OffsetWorldZ(grasp, approachMeters);
        var placeHover = OffsetWorldZ(place, approachMeters);
        var tcpLocal = TcpLocalFromGrasp(grasp.Tcp, obj.Pose);
        // Detach pose = brick center at place (same tcpLocal relative to place TCP).
        var placeWorld = Transforms.ToFrame(Transforms.Multiply(
            Transforms.FromFrame(place.Tcp), Transforms.FromFrame(tcpLocal)));

        // Touch pairs must stay on every segment while the workpiece is attached (or in contact);
        // Robotiq closed-envelope hull intersects the brick without them → Tr null at lift.
        return
        [
            options.UseSamplingTransfers ? new TransferSegment(graspHover) : new LinSegment(graspHover, linStepMeters),
            new LinSegment(grasp, linStepMeters) { AllowedCollisionPairs = contacts },
            new SetToolStateSegment(close, setDurationSeconds) { AllowedCollisionPairs = contacts },
            new AttachSegment(obj.Name, tcpLocal, obj) { AllowedCollisionPairs = contacts },
            new LinSegment(graspHover, linStepMeters) { AllowedCollisionPairs = contacts },
            options.UseSamplingTransfers
                ? new TransferSegment(placeHover) { AllowedCollisionPairs = contacts }
                : new LinSegment(placeHover, linStepMeters) { AllowedCollisionPairs = contacts },
            new LinSegment(place, linStepMeters) { AllowedCollisionPairs = contacts },
            new SetToolStateSegment(open, setDurationSeconds) { AllowedCollisionPairs = contacts },
            new DetachSegment(obj.Name, placeWorld) { AllowedCollisionPairs = contacts },
            new LinSegment(placeHover, linStepMeters) { AllowedCollisionPairs = contacts }
        ];
    }

    /// <summary>Expand N grasp/place/object triples in visit order.</summary>
    public static IReadOnlyList<MotionSegment> ExpandMany(
        IReadOnlyList<CartesianPose> grasps,
        IReadOnlyList<CartesianPose> places,
        IReadOnlyList<CollisionObject> objects,
        double approachMeters,
        EndEffectorState open,
        EndEffectorState close,
        double linStepMeters = 0.005,
        double setDurationSeconds = 0.1,
        PickPlaceOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(grasps);
        ArgumentNullException.ThrowIfNull(places);
        ArgumentNullException.ThrowIfNull(objects);
        if (grasps.Count != places.Count || grasps.Count != objects.Count)
            throw new ArgumentException("Grasp, place, and object lists must have the same count.");
        if (objects.Select(o => o.Name).Distinct(StringComparer.OrdinalIgnoreCase).Count() != objects.Count)
            throw new ArgumentException("Each workpiece must have a unique name.", nameof(objects));

        var segments = new List<MotionSegment>(grasps.Count * 10);
        for (var i = 0; i < grasps.Count; i++)
            segments.AddRange(Expand(grasps[i], places[i], approachMeters, open, close, objects[i], linStepMeters, setDurationSeconds, options));
        return segments;
    }

    internal static Frame TcpLocalFromGrasp(Frame tcpAtGrasp, Frame boxWorld)
    {
        var invTcp = Transforms.Inverse(Transforms.FromFrame(tcpAtGrasp));
        return Transforms.ToFrame(Transforms.Multiply(invTcp, Transforms.FromFrame(boxWorld)));
    }

    private static CartesianPose OffsetWorldZ(CartesianPose pose, double dz) =>
        new(new Frame(
            pose.Tcp.X, pose.Tcp.Y, pose.Tcp.Z + dz,
            pose.Tcp.Qw, pose.Tcp.Qx, pose.Tcp.Qy, pose.Tcp.Qz));
}
