using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Managed Stewart/Gough collision checker. JointState positions are leg lengths in meters; FK reconstructs
/// the platform pose and validates TCP/platform anchors plus six leg segments against scene obstacles.
/// </summary>
public sealed class StewartCollisionChecker : ICollisionChecker
{
    private const double LegRadiusMeters = 0.012;
    private const double PlatformPointRadiusMeters = 0.035;

    private readonly StewartPlatform _platform;
    private readonly StewartForwardKinematics _fk;

    public StewartCollisionChecker(StewartPlatform platform)
    {
        _platform = platform ?? throw new ArgumentNullException(nameof(platform));
        _fk = new StewartForwardKinematics(platform);
    }

    public bool IsCollisionFree(JointState state, CollisionScene scene)
    {
        var solve = _fk.TrySolve(state);
        if (!solve.Success || solve.Pose is null)
            return false;

        var pose = solve.Pose.Tcp;
        if (!PointObstacleFree(pose, PlatformPointRadiusMeters, scene, "platform_tcp"))
            return false;

        var poseM = Transforms.FromFrame(pose);
        for (var i = 0; i < StewartPlatform.LegCount; i++)
        {
            var baseAnchor = _platform.BaseAnchors[i];
            var platformAnchor = _platform.PlatformAnchors[i];
            Transforms.TransformPointInto(
                poseM,
                platformAnchor.X,
                platformAnchor.Y,
                platformAnchor.Z,
                out var px,
                out var py,
                out var pz);

            var a = new Frame(baseAnchor.X, baseAnchor.Y, baseAnchor.Z);
            var b = new Frame(px, py, pz);
            if (!PointObstacleFree(b, PlatformPointRadiusMeters, scene, $"platform_anchor_{i + 1}"))
                return false;
            if (!SegmentObstacleFree(a, b, LegRadiusMeters, scene, $"leg_{i + 1}"))
                return false;
        }

        return true;
    }

    public bool SegmentCollisionFree(JointState from, JointState to, CollisionScene scene, double configurationStep)
    {
        if (configurationStep <= 0) configurationStep = 1e-3;
        var maxDelta = 0.0;
        for (var i = 0; i < StewartPlatform.LegCount; i++)
            maxDelta = Math.Max(maxDelta, Math.Abs(to.Positions[i] - from.Positions[i]));
        var steps = Math.Max(1, (int)Math.Ceiling(maxDelta / configurationStep));
        var q = new double[StewartPlatform.LegCount];
        for (var s = 0; s <= steps; s++)
        {
            var alpha = (double)s / steps;
            for (var i = 0; i < q.Length; i++)
                q[i] = from.Positions[i] + alpha * (to.Positions[i] - from.Positions[i]);
            if (!IsCollisionFree(new JointState(q), scene))
                return false;
        }
        return true;
    }

    private static bool PointObstacleFree(Frame point, double radius, CollisionScene scene, string body)
    {
        foreach (var obj in scene.Objects)
        {
            if (scene.IsPairAllowed(body, obj.Name)) continue;
            if (PointIntersects(point, radius, obj))
                return false;
        }
        return true;
    }

    private static bool SegmentObstacleFree(Frame a, Frame b, double radius, CollisionScene scene, string body)
    {
        foreach (var obj in scene.Objects)
        {
            if (scene.IsPairAllowed(body, obj.Name)) continue;
            if (SegmentIntersects(a, b, radius, obj))
                return false;
        }
        return true;
    }

    private static bool PointIntersects(Frame point, double radius, CollisionObject obj) =>
        obj.Shape switch
        {
            CollisionShape.Sphere => DistanceSquared(point, obj.Pose) < (radius + obj.ExtentX) * (radius + obj.ExtentX),
            CollisionShape.Box => PointBoxDistanceSquared(point, obj) < radius * radius,
            CollisionShape.Plane => PlaneSignedDistance(point, obj) < radius,
            _ => false
        };

    private static bool SegmentIntersects(Frame a, Frame b, double radius, CollisionObject obj) =>
        obj.Shape switch
        {
            CollisionShape.Sphere => SegmentPointDistanceSquared(a, b, obj.Pose) < (radius + obj.ExtentX) * (radius + obj.ExtentX),
            CollisionShape.Box => SegmentBoxIntersects(a, b, radius, obj),
            CollisionShape.Plane => Math.Min(PlaneSignedDistance(a, obj), PlaneSignedDistance(b, obj)) < radius,
            _ => false
        };

    private static bool SegmentBoxIntersects(Frame a, Frame b, double radius, CollisionObject box)
    {
        const int samples = 12;
        for (var i = 0; i <= samples; i++)
        {
            var u = (double)i / samples;
            var p = new Frame(
                a.X + u * (b.X - a.X),
                a.Y + u * (b.Y - a.Y),
                a.Z + u * (b.Z - a.Z));
            if (PointBoxDistanceSquared(p, box) < radius * radius)
                return true;
        }
        return false;
    }

    private static double SegmentPointDistanceSquared(Frame a, Frame b, Frame p)
    {
        var vx = b.X - a.X; var vy = b.Y - a.Y; var vz = b.Z - a.Z;
        var wx = p.X - a.X; var wy = p.Y - a.Y; var wz = p.Z - a.Z;
        var vv = vx * vx + vy * vy + vz * vz;
        var t = vv < 1e-16 ? 0 : Math.Clamp((wx * vx + wy * vy + wz * vz) / vv, 0, 1);
        var cx = a.X + t * vx; var cy = a.Y + t * vy; var cz = a.Z + t * vz;
        var dx = p.X - cx; var dy = p.Y - cy; var dz = p.Z - cz;
        return dx * dx + dy * dy + dz * dz;
    }

    private static double PointBoxDistanceSquared(Frame point, CollisionObject box)
    {
        var inv = Transforms.Inverse(Transforms.FromFrame(box.Pose));
        var local = Transforms.ToFrame(Transforms.Multiply(inv, Transforms.FromFrame(point)));
        var cx = Math.Clamp(local.X, -box.ExtentX, box.ExtentX);
        var cy = Math.Clamp(local.Y, -box.ExtentY, box.ExtentY);
        var cz = Math.Clamp(local.Z, -box.ExtentZ, box.ExtentZ);
        var dx = local.X - cx; var dy = local.Y - cy; var dz = local.Z - cz;
        return dx * dx + dy * dy + dz * dz;
    }

    private static double PlaneSignedDistance(Frame point, CollisionObject plane)
    {
        var m = Transforms.FromFrame(plane.Pose);
        var nx = m[0]; var ny = m[4]; var nz = m[8];
        return (point.X - plane.Pose.X) * nx + (point.Y - plane.Pose.Y) * ny + (point.Z - plane.Pose.Z) * nz;
    }

    private static double DistanceSquared(Frame a, Frame b)
    {
        var dx = a.X - b.X; var dy = a.Y - b.Y; var dz = a.Z - b.Z;
        return dx * dx + dy * dy + dz * dz;
    }
}
