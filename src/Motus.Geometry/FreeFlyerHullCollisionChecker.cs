using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Rigid free-flyer / aerial hull as a single sphere envelope at the world base pose.
/// Implements <see cref="IBaseFrameCollisionChecker"/> so HolonomicSE3 sampling can override
/// the base frame per configuration. Motus-honest offline collision only — not PX4 / SITL.
/// </summary>
public sealed class FreeFlyerHullCollisionChecker : IBaseFrameCollisionChecker
{
    private readonly double _radiusMeters;
    private readonly BaseFrame _defaultBase;
    private readonly double[] _radii;

    /// <summary>
    /// Circumscribed sphere radius for <c>tests/fixtures/aerial/free_flyer_box.urdf</c>
    /// (box size 0.2 × 0.2 × 0.08 m → half-extents 0.1 / 0.1 / 0.04).
    /// </summary>
    public static double FreeFlyerBoxCircumscribedRadiusMeters { get; } =
        Math.Sqrt(0.1 * 0.1 + 0.1 * 0.1 + 0.04 * 0.04);

    public FreeFlyerHullCollisionChecker(double radiusMeters, BaseFrame? defaultBase = null)
    {
        if (!double.IsFinite(radiusMeters) || radiusMeters <= 0)
            throw new ArgumentOutOfRangeException(nameof(radiusMeters), "Hull radius must be finite and > 0.");
        _radiusMeters = radiusMeters;
        _defaultBase = defaultBase ?? BaseFrame.Identity;
        _radii = [_radiusMeters];
    }

    /// <summary>Hull matched to the meshless free-flyer box fixture.</summary>
    public static FreeFlyerHullCollisionChecker ForFreeFlyerBox(BaseFrame? defaultBase = null) =>
        new(FreeFlyerBoxCircumscribedRadiusMeters, defaultBase);

    public double RadiusMeters => _radiusMeters;

    public bool IsCollisionFree(JointState state, CollisionScene scene) =>
        IsCollisionFree(state, scene, _defaultBase);

    public bool IsCollisionFree(JointState state, CollisionScene scene, BaseFrame baseFrame)
    {
        // Free-flyer: actuated joints (if any) are ignored; hull rides the base frame.
        _ = state;
        if (scene.Objects.Count == 0)
            return true;
        Frame[] origins = [baseFrame.Frame];
        return LinkEnvelopeCollision.SceneObstacleFree(origins, _radii, scene, Intersects);
    }

    public bool SegmentCollisionFree(JointState from, JointState to, CollisionScene scene, double configurationStep)
    {
        // Without an explicit base path, only endpoints at the default base are meaningful.
        _ = configurationStep;
        return IsCollisionFree(from, scene) && IsCollisionFree(to, scene);
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
        var nx = m[0];
        var ny = m[4];
        var nz = m[8];
        var signed = (center.X - plane.Pose.X) * nx + (center.Y - plane.Pose.Y) * ny + (center.Z - plane.Pose.Z) * nz;
        return signed < radius;
    }

    private static bool SphereSphereOverlap(Frame a, double ra, Frame b, double rb)
    {
        var dx = a.X - b.X;
        var dy = a.Y - b.Y;
        var dz = a.Z - b.Z;
        var limit = ra + rb;
        return dx * dx + dy * dy + dz * dz < limit * limit;
    }

    private static bool SphereBoxOverlap(Frame center, double radius, CollisionObject box)
    {
        var local = WorldToBoxLocal(center, box.Pose);
        var hx = box.ExtentX;
        var hy = box.ExtentY;
        var hz = box.ExtentZ;
        var cx = Math.Clamp(local.X, -hx, hx);
        var cy = Math.Clamp(local.Y, -hy, hy);
        var cz = Math.Clamp(local.Z, -hz, hz);
        var dx = local.X - cx;
        var dy = local.Y - cy;
        var dz = local.Z - cz;
        return dx * dx + dy * dy + dz * dz < radius * radius;
    }

    private static Frame WorldToBoxLocal(Frame point, Frame boxPose)
    {
        var inv = Transforms.Inverse(Transforms.FromFrame(boxPose));
        var p = Transforms.Multiply(inv, Transforms.FromFrame(point));
        return Transforms.ToFrame(p);
    }
}
