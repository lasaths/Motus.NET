using Motus.Core;

namespace Motus.Geometry;

/// <summary>Sphere-envelope collision checker using link origins.</summary>
public sealed class SphereCollisionChecker : ICollisionChecker
{
    private readonly IFkSolver _fk;
    private readonly BaseFrame _base;
    private readonly KinematicsChain? _dhChain;
    private readonly double[]? _baseM;
    private readonly double[]? _linkXyz;
    private readonly double[]? _matA;
    private readonly double[]? _matB;
    private readonly double[]? _matC;
    private readonly double[]? _qBuffer;
    private readonly double[] _radii;

    public SphereCollisionChecker(RobotPreset preset)
        : this(KinematicsResolver.CreateFkSolver(preset), preset.BaseFrame, preset) { }

    public SphereCollisionChecker(RobotPreset preset, SerialJointChain serialChain)
        : this(KinematicsResolver.CreateFkSolver(preset, serialChain), preset.BaseFrame, preset) { }

    public SphereCollisionChecker(IFkSolver fk, BaseFrame baseFrame)
        : this(fk, baseFrame, null) { }

    private SphereCollisionChecker(IFkSolver fk, BaseFrame baseFrame, RobotPreset? preset)
    {
        _fk = fk;
        _base = baseFrame;
        _radii = _fk.LinkRadiiMeters;
        if (preset is not null && KinematicsProfiles.TryGet(preset, out var chain))
        {
            _dhChain = chain;
            _baseM = Transforms.FromFrame(baseFrame.Frame);
            _linkXyz = new double[_radii.Length * 3];
            _matA = new double[16];
            _matB = new double[16];
            _matC = new double[16];
            _qBuffer = new double[_radii.Length];
        }
    }

    public bool IsCollisionFree(JointState state, CollisionScene scene) =>
        IsCollisionFree(state.Positions, scene);

    public bool IsCollisionFree(IReadOnlyList<double> positions, CollisionScene scene)
    {
        if (_dhChain is not null && positions is double[] q)
            return IsCollisionFreeFast(q, scene);
        if (_dhChain is not null)
            return IsCollisionFreeFast(positions.ToArray(), scene);
        var origins = _fk.ComputeLinkOrigins(positions, _base.Frame);
        if (!SelfCollisionFree(origins, _radii)) return false;
        return LinkEnvelopeCollision.SceneObstacleFree(origins, _radii, scene, Intersects);
    }

    public bool SegmentCollisionFree(IReadOnlyList<double> from, IReadOnlyList<double> to, CollisionScene scene, double stepRadians)
    {
        if (stepRadians <= 0) stepRadians = 1e-3;
        var n = from.Count;
        var maxDelta = 0.0;
        for (var i = 0; i < n; i++)
            maxDelta = Math.Max(maxDelta, Math.Abs(to[i] - from[i]));
        var steps = Math.Max(1, (int)Math.Ceiling(maxDelta / stepRadians));
        if (_dhChain is not null)
        {
            var q = _qBuffer!;
            for (var s = 0; s <= steps; s++)
            {
                var alpha = (double)s / steps;
                for (var i = 0; i < n; i++)
                    q[i] = from[i] + alpha * (to[i] - from[i]);
                if (!IsCollisionFreeFast(q, scene))
                    return false;
            }
            return true;
        }

        var qSlow = new double[n];
        for (var s = 0; s <= steps; s++)
        {
            var alpha = (double)s / steps;
            for (var i = 0; i < n; i++)
                qSlow[i] = from[i] + alpha * (to[i] - from[i]);
            if (!IsCollisionFree(qSlow, scene))
                return false;
        }
        return true;
    }

    public bool SegmentCollisionFree(JointState from, JointState to, CollisionScene scene, double stepRadians) =>
        SegmentCollisionFree(from.Positions, to.Positions, scene, stepRadians);

    private bool IsCollisionFreeFast(double[] positions, CollisionScene scene)
    {
        FastDhFk.ComputeLinkWorldPositions(_dhChain!, positions, _baseM!, _linkXyz!, _matA!, _matB!, _matC!);
        if (!SelfCollisionFreeXyz(_linkXyz!)) return false;
        return scene.Objects.Count == 0 || LinkEnvelopeCollision.SceneObstacleFreeXyz(_linkXyz!, _radii, scene);
    }

    private bool SelfCollisionFreeXyz(ReadOnlySpan<double> xyz)
    {
        var linkCount = _radii.Length;
        for (var i = 0; i < linkCount; i++)
        {
            var a = i * 3;
            for (var j = i + 2; j < linkCount; j++)
            {
                var b = j * 3;
                if (CoincidentXyz(xyz[a], xyz[a + 1], xyz[a + 2], xyz[b], xyz[b + 1], xyz[b + 2])) continue;
                var dx = xyz[a] - xyz[b];
                var dy = xyz[a + 1] - xyz[b + 1];
                var dz = xyz[a + 2] - xyz[b + 2];
                var limit = _radii[i] + _radii[j];
                if (dx * dx + dy * dy + dz * dz < limit * limit)
                    return false;
            }
        }
        return true;
    }

    private static bool SelfCollisionFree(IReadOnlyList<Frame> origins, IReadOnlyList<double> radii)
    {
        for (var i = 0; i < origins.Count; i++)
        {
            for (var j = i + 2; j < origins.Count; j++)
            {
                if (CoincidentLinkOrigins(origins[i], origins[j])) continue;
                if (SphereSphereOverlap(origins[i], radii[i], origins[j], radii[j]))
                    return false;
            }
        }
        return true;
    }

    private static bool CoincidentXyz(double ax, double ay, double az, double bx, double by, double bz)
    {
        const double eps = 1e-6;
        return Math.Abs(ax - bx) < eps && Math.Abs(ay - by) < eps && Math.Abs(az - bz) < eps;
    }

    private static bool CoincidentLinkOrigins(Frame a, Frame b)
    {
        const double eps = 1e-6;
        return Math.Abs(a.X - b.X) < eps && Math.Abs(a.Y - b.Y) < eps && Math.Abs(a.Z - b.Z) < eps;
    }

    private static bool Intersects(Frame link, double linkRadius, CollisionObject obj) =>
        obj.Shape switch
        {
            CollisionShape.Sphere => SphereSphereOverlap(link, linkRadius, obj.Pose, obj.ExtentX),
            CollisionShape.Box => SphereBoxOverlap(link, linkRadius, obj),
            CollisionShape.Plane => SpherePlaneOverlap(link, linkRadius, obj),
            _ => false
        };

    /// <summary>Half-space: Motus local +X is free. Collide when signed distance &lt; radius.</summary>
    private static bool SpherePlaneOverlap(Frame center, double radius, CollisionObject plane)
    {
        var m = Transforms.FromFrame(plane.Pose);
        var nx = m[0]; var ny = m[4]; var nz = m[8];
        var signed = (center.X - plane.Pose.X) * nx + (center.Y - plane.Pose.Y) * ny + (center.Z - plane.Pose.Z) * nz;
        return signed < radius;
    }

    private static bool SphereSphereOverlap(Frame a, double ra, Frame b, double rb)
    {
        var dx = a.X - b.X; var dy = a.Y - b.Y; var dz = a.Z - b.Z;
        var limit = ra + rb;
        return dx * dx + dy * dy + dz * dz < limit * limit;
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
