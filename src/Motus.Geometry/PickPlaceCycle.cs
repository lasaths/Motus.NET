using Motus.Core;

namespace Motus.Geometry;

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
        double setDurationSeconds = 0.1)
    {
        ArgumentNullException.ThrowIfNull(grasp);
        ArgumentNullException.ThrowIfNull(place);
        ArgumentNullException.ThrowIfNull(open);
        ArgumentNullException.ThrowIfNull(close);
        ArgumentNullException.ThrowIfNull(obj);
        if (approachMeters < 0)
            throw new ArgumentOutOfRangeException(nameof(approachMeters), "Approach height must be >= 0.");
        if (linStepMeters <= 0)
            throw new ArgumentOutOfRangeException(nameof(linStepMeters), "LIN step must be > 0.");

        var graspHover = OffsetWorldZ(grasp, approachMeters);
        var placeHover = OffsetWorldZ(place, approachMeters);
        var tcpLocal = TcpLocalFromGrasp(grasp.Tcp, obj.Pose);
        // Detach pose = brick center at place (same tcpLocal relative to place TCP).
        var placeWorld = obj.Shape switch
        {
            _ => Transforms.ToFrame(Transforms.Multiply(
                Transforms.FromFrame(place.Tcp),
                Transforms.FromFrame(tcpLocal)))
        };

        return
        [
            new LinSegment(graspHover, linStepMeters),
            new LinSegment(grasp, linStepMeters),
            new SetToolStateSegment(close, setDurationSeconds),
            new AttachSegment(obj.Name, tcpLocal, obj),
            new LinSegment(graspHover, linStepMeters),
            new LinSegment(placeHover, linStepMeters),
            new LinSegment(place, linStepMeters),
            new SetToolStateSegment(open, setDurationSeconds),
            // Lift off before Detach so TCP is clear of the restored scene obstacle.
            new LinSegment(placeHover, linStepMeters),
            new DetachSegment(obj.Name, placeWorld)
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
        double setDurationSeconds = 0.1)
    {
        ArgumentNullException.ThrowIfNull(grasps);
        ArgumentNullException.ThrowIfNull(places);
        ArgumentNullException.ThrowIfNull(objects);
        if (grasps.Count != places.Count || grasps.Count != objects.Count)
            throw new ArgumentException("Grasp, place, and object lists must have the same count.");

        var segments = new List<MotionSegment>(grasps.Count * 10);
        for (var i = 0; i < grasps.Count; i++)
            segments.AddRange(Expand(grasps[i], places[i], approachMeters, open, close, objects[i], linStepMeters, setDurationSeconds));
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
