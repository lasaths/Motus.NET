using Motus.Core;
using Motus.Geometry;

namespace Motus.Geometry;

/// <summary>Sphere-envelope collision checker using link origins.</summary>
public sealed class SphereCollisionChecker : ICollisionChecker
{
    private readonly IFkSolver _fk;
    private readonly BaseFrame _base;

    public SphereCollisionChecker(RobotPreset preset)
        : this(KinematicsResolver.CreateFkSolver(preset), preset.BaseFrame) { }

    public SphereCollisionChecker(RobotPreset preset, SerialJointChain serialChain)
        : this(KinematicsResolver.CreateFkSolver(preset, serialChain), preset.BaseFrame) { }

    public SphereCollisionChecker(IFkSolver fk, BaseFrame baseFrame)
    {
        _fk = fk;
        _base = baseFrame;
    }

    public bool IsCollisionFree(JointState state, CollisionScene scene)
    {
        if (!SelfCollisionFree(state)) return false;
        var origins = _fk.ComputeLinkOrigins(state.Positions, _base.Frame);
        var radii = _fk.LinkRadiiMeters;
        return LinkEnvelopeCollision.SceneObstacleFree(origins, radii, scene, Intersects);
    }

    public bool SegmentCollisionFree(JointState from, JointState to, CollisionScene scene, double stepRadians)
    {
        var n = from.AxisCount;
        var maxDelta = 0.0;
        for (var i = 0; i < n; i++)
            maxDelta = Math.Max(maxDelta, Math.Abs(to.Positions[i] - from.Positions[i]));
        var steps = Math.Max(1, (int)Math.Ceiling(maxDelta / stepRadians));
        for (var s = 0; s <= steps; s++)
        {
            var alpha = (double)s / steps;
            var q = new double[n];
            for (var i = 0; i < n; i++)
                q[i] = from.Positions[i] + alpha * (to.Positions[i] - from.Positions[i]);
            if (!IsCollisionFree(new JointState(q), scene))
                return false;
        }
        return true;
    }

    private bool SelfCollisionFree(JointState state)
    {
        var origins = _fk.ComputeLinkOrigins(state.Positions, _base.Frame);
        var radii = _fk.LinkRadiiMeters;
        for (var i = 0; i < origins.Count; i++)
        {
            for (var j = i + 2; j < origins.Count; j++)
            {
                if (SphereSphereOverlap(origins[i], radii[i], origins[j], radii[j]))
                    return false;
            }
        }
        return true;
    }

    private static bool Intersects(Frame link, double linkRadius, CollisionObject obj) =>
        obj.Shape switch
        {
            CollisionShape.Sphere => SphereSphereOverlap(link, linkRadius, obj.Pose, obj.ExtentX),
            CollisionShape.Box => SphereBoxOverlap(link, linkRadius, obj),
            _ => false
        };

    private static bool SphereSphereOverlap(Frame a, double ra, Frame b, double rb)
    {
        var dx = a.X - b.X; var dy = a.Y - b.Y; var dz = a.Z - b.Z;
        var dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        return dist < ra + rb;
    }

    private static bool SphereBoxOverlap(Frame center, double radius, CollisionObject box)
    {
        var local = WorldToBoxLocal(center, box.Pose);
        var hx = box.ExtentX; var hy = box.ExtentY; var hz = box.ExtentZ;
        var cx = Math.Clamp(local.X, -hx, hx);
        var cy = Math.Clamp(local.Y, -hy, hy);
        var cz = Math.Clamp(local.Z, -hz, hz);
        var dx = local.X - cx; var dy = local.Y - cy; var dz = local.Z - cz;
        return dx * dx + dy * dy + dz * dz < radius * radius;
    }

    private static Frame WorldToBoxLocal(Frame point, Frame boxPose)
    {
        var inv = Transforms.Inverse(Transforms.FromFrame(boxPose));
        var p = Transforms.Multiply(inv, Transforms.FromFrame(point));
        return Transforms.ToFrame(p);
    }
}
