using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Wraps an <see cref="IBaseFrameCollisionChecker"/> so attached payloads ride the mobile
/// <see cref="BaseFrame"/> (HolonomicSE3 / free-flyer). <see cref="AttachedBody.TcpLocalPose"/>
/// is interpreted as <b>base-local</b> (same field name as serial TCP-local for API compat).
/// Motus 2.1 ADR 0002 Phase B.
/// </summary>
public sealed class BaseFrameAttachCollisionChecker : IBaseFrameCollisionChecker
{
    private readonly IBaseFrameCollisionChecker _inner;
    private readonly IReadOnlyList<AttachedBody> _attached;
    private readonly BaseFrame _defaultBase;
    private readonly Dictionary<int, BvhNode> _meshBvhCache = new();
    private readonly CollisionQueryScratch _scratch = new();

    public BaseFrameAttachCollisionChecker(
        IBaseFrameCollisionChecker inner,
        IReadOnlyList<AttachedBody> attached,
        BaseFrame? defaultBase = null)
    {
        _inner = inner ?? throw new ArgumentNullException(nameof(inner));
        _attached = attached ?? Array.Empty<AttachedBody>();
        _defaultBase = defaultBase ?? BaseFrame.Identity;
    }

    public bool IsCollisionFree(JointState state, CollisionScene scene) =>
        _inner.IsCollisionFree(state, scene) && AttachedCollisionFree(_defaultBase, scene);

    public bool IsCollisionFree(JointState state, CollisionScene scene, BaseFrame baseFrame) =>
        _inner.IsCollisionFree(state, scene, baseFrame) && AttachedCollisionFree(baseFrame, scene);

    public bool SegmentCollisionFree(JointState from, JointState to, CollisionScene scene, double configurationStep)
    {
        if (!_inner.SegmentCollisionFree(from, to, scene, configurationStep)) return false;
        if (_attached.Count == 0) return true;
        // Mobility sampling uses per-sample BaseFrameOverride via IsCollisionFree(..., base).
        return AttachedCollisionFree(_defaultBase, scene);
    }

    private bool AttachedCollisionFree(BaseFrame baseFrame, CollisionScene scene)
    {
        if (_attached.Count == 0) return true;
        var baseM = Transforms.FromFrame(baseFrame.Frame);
        foreach (var body in _attached)
        {
            var localM = Transforms.Multiply(baseM, Transforms.FromFrame(body.TcpLocalPose));
            var worldM = CollisionGeometry.ComposeWorldMatrix(localM, body.Geometry.Pose);
            foreach (var obj in scene.Objects)
            {
                if (body.SourceSceneObjectName is { } src &&
                    string.Equals(obj.Name, src, StringComparison.OrdinalIgnoreCase))
                    continue;
                if (scene.IsPairAllowed(body.Geometry.Name, obj.Name)) continue;
                if (scene.IsPairAllowed(CollisionBodies.Attached(body.Name), obj.Name)) continue;
                if (CollisionGeometry.IntersectsAtPose(body.Geometry, worldM, obj, _meshBvhCache, _scratch))
                    return false;
            }
        }
        return true;
    }
}
