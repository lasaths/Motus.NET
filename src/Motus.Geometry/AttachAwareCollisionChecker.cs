using Motus.Core;

namespace Motus.Geometry;

/// <summary>Wraps any checker with TCP-attached body collision.</summary>
public sealed class AttachAwareCollisionChecker : ICollisionChecker
{
    private readonly ICollisionChecker _inner;
    private readonly IFkSolver _fk;
    private readonly BaseFrame _base;
    private readonly ToolFrame _tool;
    private readonly IReadOnlyList<AttachedBody> _attached;
    private readonly Dictionary<int, BvhNode> _meshBvhCache = new();
    private readonly CollisionQueryScratch _scratch = new();
    private double[]? _segmentQ;
    private JointState? _segmentState;
    private int _sceneFingerprint;
    private bool _sceneFingerprintValid;

    public AttachAwareCollisionChecker(
        ICollisionChecker inner,
        IFkSolver fk,
        BaseFrame baseFrame,
        ToolFrame toolFrame,
        IReadOnlyList<AttachedBody> attached)
    {
        _inner = inner;
        _fk = fk;
        _base = baseFrame;
        _tool = toolFrame;
        _attached = attached;
    }

    public bool IsCollisionFree(JointState state, CollisionScene scene)
    {
        if (!_inner.IsCollisionFree(state, scene)) return false;
        return AttachedCollisionFree(state, scene);
    }

    public bool SegmentCollisionFree(JointState from, JointState to, CollisionScene scene, double stepRadians)
    {
        if (stepRadians <= 0) stepRadians = 1e-3;
        if (!_inner.SegmentCollisionFree(from, to, scene, stepRadians)) return false;
        var n = from.AxisCount;
        var maxDelta = 0.0;
        for (var i = 0; i < n; i++)
            maxDelta = Math.Max(maxDelta, Math.Abs(to.Positions[i] - from.Positions[i]));
        var steps = Math.Max(1, (int)Math.Ceiling(maxDelta / stepRadians));

        if (_segmentQ is null || _segmentQ.Length != n)
        {
            _segmentQ = new double[n];
            _segmentState = JointState.Wrap(_segmentQ);
        }

        var q = _segmentQ;
        var state = _segmentState!;
        for (var s = 0; s <= steps; s++)
        {
            var alpha = (double)s / steps;
            for (var i = 0; i < n; i++)
                q[i] = from.Positions[i] + alpha * (to.Positions[i] - from.Positions[i]);
            if (!AttachedCollisionFree(state, scene)) return false;
        }
        return true;
    }

    private bool AttachedCollisionFree(JointState state, CollisionScene scene)
    {
        if (_attached.Count == 0) return true;
        EnsureBvhCache(scene);
        var tcpM = _fk.ComputeTcpTransform(state.Positions, _base.Frame, _tool.Frame);
        foreach (var body in _attached)
        {
            var localM = Transforms.Multiply(tcpM, Transforms.FromFrame(body.TcpLocalPose));
            var worldM = CollisionGeometry.ComposeWorldMatrix(localM, body.Geometry.Pose);
            foreach (var obj in scene.Objects)
            {
                if (scene.IsPairAllowed(body.Geometry.Name, obj.Name)) continue;
                if (scene.IsPairAllowed(CollisionBodies.Attached(body.Name), obj.Name)) continue;
                if (CollisionGeometry.IntersectsAtPose(body.Geometry, worldM, obj, _meshBvhCache, _scratch))
                    return false;
            }
        }
        return true;
    }

    private void EnsureBvhCache(CollisionScene scene)
    {
        var hash = new HashCode();
        hash.Add(scene.Objects.Count);
        for (var i = 0; i < scene.Objects.Count; i++)
        {
            var o = scene.Objects[i];
            hash.Add(o.ContentHash);
            hash.Add(o.Pose.X);
            hash.Add(o.Pose.Y);
            hash.Add(o.Pose.Z);
        }
        var fp = hash.ToHashCode();
        if (_sceneFingerprintValid && fp == _sceneFingerprint)
            return;

        _meshBvhCache.Clear();
        for (var i = 0; i < scene.Objects.Count; i++)
        {
            var meshObj = scene.Objects[i];
            if (meshObj.Shape != CollisionShape.Mesh) continue;
            if (meshObj.MeshVertices is null || meshObj.MeshIndices is null) continue;
            var key = CollisionMeshCache.GeometryFingerprint(meshObj);
            _meshBvhCache[key] = CollisionMeshCache.GetOrBuild(meshObj);
        }
        foreach (var body in _attached)
        {
            if (body.Geometry.Shape == CollisionShape.Mesh &&
                body.Geometry.MeshVertices is not null &&
                body.Geometry.MeshIndices is not null)
            {
                var key = CollisionMeshCache.GeometryFingerprint(body.Geometry);
                _meshBvhCache[key] = CollisionMeshCache.GetOrBuild(body.Geometry);
            }
        }
        _sceneFingerprint = fp;
        _sceneFingerprintValid = true;
    }
}
