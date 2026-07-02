using Motus.Core;

namespace Motus.Geometry;

internal static class LinkEnvelopeCollision
{
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
}

