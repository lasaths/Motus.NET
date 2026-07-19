using Motus.Core;

namespace Motus.Geometry;

internal static class LinkEnvelopeCollision
{
    public static bool SceneObstacleFreeXyz(
        ReadOnlySpan<double> xyz,
        ReadOnlySpan<double> radii,
        CollisionScene scene)
    {
        var linkCount = radii.Length;
        foreach (var obj in scene.Objects)
        {
            for (var i = 0; i < linkCount; i++)
            {
                if (scene.IsPairAllowed(CollisionBodies.RobotLink(i), obj.Name)) continue;
                var o = i * 3;
                if (IntersectsXyz(xyz[o], xyz[o + 1], xyz[o + 2], radii[i], obj))
                    return false;
            }

            for (var i = 0; i < linkCount - 1; i++)
            {
                if (scene.IsPairAllowed(CollisionBodies.RobotLink(i), obj.Name)
                    && scene.IsPairAllowed(CollisionBodies.RobotLink(i + 1), obj.Name))
                    continue;
                var a = i * 3;
                var b = (i + 1) * 3;
                if (!CapsuleObjectFreeXyz(
                    xyz[a], xyz[a + 1], xyz[a + 2], radii[i],
                    xyz[b], xyz[b + 1], xyz[b + 2], radii[i + 1],
                    obj))
                    return false;
            }
        }
        return true;
    }

    public static bool SceneObstacleFree(
        IReadOnlyList<Frame> origins,
        IReadOnlyList<double> radii,
        CollisionScene scene,
        Func<Frame, double, CollisionObject, bool> intersects,
        double maxJointDeltaRadians = 0.0)
    {
        var sampleScale = maxJointDeltaRadians > 0
            ? Math.Clamp((int)Math.Ceiling(maxJointDeltaRadians / 0.05), 1, 12)
            : 1;

        foreach (var obj in scene.Objects)
        {
            for (var i = 0; i < origins.Count; i++)
            {
                if (scene.IsPairAllowed(CollisionBodies.RobotLink(i), obj.Name)) continue;
                if (intersects(origins[i], radii[i], obj))
                    return false;
            }

            for (var i = 0; i < origins.Count - 1; i++)
            {
                if (scene.IsPairAllowed(CollisionBodies.RobotLink(i), obj.Name)
                    && scene.IsPairAllowed(CollisionBodies.RobotLink(i + 1), obj.Name))
                    continue;
                if (!CapsuleObjectFree(origins[i], radii[i], origins[i + 1], radii[i + 1], obj, intersects, sampleScale))
                    return false;
            }
        }
        return true;
    }

    private static bool CapsuleObjectFree(
        Frame a, double ra, Frame b, double rb, CollisionObject obj,
        Func<Frame, double, CollisionObject, bool> intersects, int sampleScale)
    {
        var samples = Math.Max(4, 4 * sampleScale);
        for (var s = 0; s <= samples; s++)
        {
            var t = (double)s / samples;
            var cx = a.X + t * (b.X - a.X);
            var cy = a.Y + t * (b.Y - a.Y);
            var cz = a.Z + t * (b.Z - a.Z);
            var r = ra + t * (rb - ra);
            if (intersects(new Frame(cx, cy, cz), r, obj))
                return false;
        }
        return true;
    }

    private static bool CapsuleObjectFreeXyz(
        double ax, double ay, double az, double ra,
        double bx, double by, double bz, double rb,
        CollisionObject obj)
    {
        const int samples = 4;
        for (var s = 0; s <= samples; s++)
        {
            var t = (double)s / samples;
            var cx = ax + t * (bx - ax);
            var cy = ay + t * (by - ay);
            var cz = az + t * (bz - az);
            var r = ra + t * (rb - ra);
            if (IntersectsXyz(cx, cy, cz, r, obj))
                return false;
        }
        return true;
    }

    private static bool IntersectsXyz(double x, double y, double z, double radius, CollisionObject obj) =>
        obj.Shape switch
        {
            CollisionShape.Sphere => SphereSphereOverlapXyz(
                x, y, z, radius, obj.Pose.X, obj.Pose.Y, obj.Pose.Z, obj.ExtentX),
            CollisionShape.Box => SphereBoxOverlapXyz(x, y, z, radius, obj),
            CollisionShape.Plane => SpherePlaneOverlapXyz(x, y, z, radius, obj),
            _ => false
        };

    /// <summary>Half-space: Motus local +X is free. Collide when signed distance &lt; radius.</summary>
    private static bool SpherePlaneOverlapXyz(double x, double y, double z, double radius, CollisionObject plane)
    {
        var m = Transforms.FromFrame(plane.Pose);
        // Local +X axis in world = first column
        var nx = m[0]; var ny = m[4]; var nz = m[8];
        var signed = (x - plane.Pose.X) * nx + (y - plane.Pose.Y) * ny + (z - plane.Pose.Z) * nz;
        return signed < radius;
    }

    private static bool SphereSphereOverlapXyz(
        double ax, double ay, double az, double ra,
        double bx, double by, double bz, double rb)
    {
        var dx = ax - bx; var dy = ay - by; var dz = az - bz;
        var limit = ra + rb;
        return dx * dx + dy * dy + dz * dz < limit * limit;
    }

    private static bool SphereBoxOverlapXyz(double x, double y, double z, double radius, CollisionObject box)
    {
        var local = WorldToBoxLocalXyz(x, y, z, box.Pose);
        var hx = box.ExtentX; var hy = box.ExtentY; var hz = box.ExtentZ;
        var cx = Math.Clamp(local.x, -hx, hx);
        var cy = Math.Clamp(local.y, -hy, hy);
        var cz = Math.Clamp(local.z, -hz, hz);
        var dx = local.x - cx; var dy = local.y - cy; var dz = local.z - cz;
        return dx * dx + dy * dy + dz * dz < radius * radius;
    }

    private static (double x, double y, double z) WorldToBoxLocalXyz(double px, double py, double pz, Frame boxPose)
    {
        var dx = px - boxPose.X;
        var dy = py - boxPose.Y;
        var dz = pz - boxPose.Z;
        var q = NormalizeQuat(boxPose.Qw, boxPose.Qx, boxPose.Qy, boxPose.Qz);
        return RotateVectorByQuatInverse(dx, dy, dz, q.w, q.x, q.y, q.z);
    }

    private static (double w, double x, double y, double z) NormalizeQuat(double w, double x, double y, double z)
    {
        var n = Math.Sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-12) return (1, 0, 0, 0);
        return (w / n, x / n, y / n, z / n);
    }

    private static (double x, double y, double z) RotateVectorByQuatInverse(
        double vx, double vy, double vz, double w, double x, double y, double z)
    {
        var cx = y * vz - z * vy;
        var cy = z * vx - x * vz;
        var cz = x * vy - y * vx;
        var dot = x * vx + y * vy + z * vz;
        return (
            vx + 2 * (w * cx + y * cz - z * cy),
            vy + 2 * (w * cy + z * cx - x * cz),
            vz + 2 * (w * cz + x * cy - y * cx));
    }
}

