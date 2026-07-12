using System.Linq;
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
    private readonly Dictionary<string, BvhNode> _meshBvhCache = new();

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
        for (var s = 0; s <= steps; s++)
        {
            var alpha = (double)s / steps;
            var q = new double[n];
            for (var i = 0; i < n; i++)
                q[i] = from.Positions[i] + alpha * (to.Positions[i] - from.Positions[i]);
            if (!AttachedCollisionFree(new JointState(q), scene)) return false;
        }
        return true;
    }

    private bool AttachedCollisionFree(JointState state, CollisionScene scene)
    {
        if (_attached.Count == 0) return true;
        BuildBvhCache(scene);
        var tcpM = _fk.ComputeTcpTransform(state.Positions, _base.Frame, _tool.Frame);
        foreach (var body in _attached)
        {
            var localM = Transforms.Multiply(tcpM, Transforms.FromFrame(body.TcpLocalPose));
            var att = CollisionGeometry.Transform(body.Geometry, localM);
            foreach (var obj in scene.Objects)
            {
                if (scene.IsPairAllowed(att.Name, obj.Name)) continue;
                if (scene.IsPairAllowed(CollisionBodies.Attached(body.Name), obj.Name)) continue;
                if (CollisionGeometry.Intersects(att, obj, _meshBvhCache)) return false;
            }
        }
        return true;
    }

    private void BuildBvhCache(CollisionScene scene)
    {
        foreach (var meshObj in scene.Objects.Where(o => o.Shape == CollisionShape.Mesh))
        {
            if (meshObj.MeshVertices is not null && meshObj.MeshIndices is not null)
                _meshBvhCache[meshObj.Name] = CollisionMeshCache.GetOrBuild(meshObj);
        }
        foreach (var body in _attached)
        {
            if (body.Geometry.Shape == CollisionShape.Mesh &&
                body.Geometry.MeshVertices is not null &&
                body.Geometry.MeshIndices is not null)
                _meshBvhCache[body.Name] = CollisionMeshCache.GetOrBuild(body.Geometry);
        }
    }
}
