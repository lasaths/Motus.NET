using System.Linq;
using Motus.Core;

namespace Motus.Geometry;

/// <summary>Collision checker using per-link robot geometry when available; sphere fallback otherwise.</summary>
public sealed class RobotMeshCollisionChecker : ICollisionChecker
{
    private readonly IFkSolver _fk;
    private readonly BaseFrame _base;
    private readonly RobotCollisionModel? _robotCollision;
    private readonly SphereCollisionChecker _fallback;
    private readonly Dictionary<string, BvhNode> _meshBvhCache = new();

    public RobotMeshCollisionChecker(RobotModel robot, SerialJointChain? chain = null)
    {
        _fk = KinematicsResolver.CreateFkSolver(robot.Preset, chain);
        _base = robot.Preset.BaseFrame;
        _robotCollision = robot.CollisionModel;
        _fallback = new SphereCollisionChecker(_fk, _base);
    }

    public bool IsCollisionFree(JointState state, CollisionScene scene)
    {
        if (_robotCollision is null || _robotCollision.Links.Count == 0)
            return _fallback.IsCollisionFree(state, scene);

        BuildBvhCache(scene);
        if (!SelfCollisionFree(state)) return false;

        var linkMats = _fk.ComputeLinkTransforms(state.Positions);
        var baseM = Transforms.FromFrame(_base.Frame);
        foreach (var obj in scene.Objects)
        {
            foreach (var link in _robotCollision.Links)
            {
                if (link.LinkIndex < 0 || link.LinkIndex >= linkMats.Count) continue;
                if (scene.IsPairAllowed(CollisionBodies.RobotLink(link.LinkIndex), obj.Name))
                    continue;
                var worldM = Transforms.Multiply(baseM, linkMats[link.LinkIndex]);
                var worldGeom = CollisionGeometry.Transform(link.LocalGeometry, worldM);
                if (CollisionGeometry.Intersects(worldGeom, obj, _meshBvhCache))
                    return false;
            }
        }
        return true;
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

    private void BuildBvhCache(CollisionScene scene)
    {
        foreach (var meshObj in scene.Objects.Where(o => o.Shape == CollisionShape.Mesh))
        {
            if (!_meshBvhCache.ContainsKey(meshObj.Name) && meshObj.MeshVertices is not null && meshObj.MeshIndices is not null)
                _meshBvhCache[meshObj.Name] = BvhBuilder.Build(meshObj);
        }
    }

    private bool SelfCollisionFree(JointState state)
    {
        if (_robotCollision is null) return _fallback.IsCollisionFree(state, new CollisionScene());
        var linkMats = _fk.ComputeLinkTransforms(state.Positions);
        var baseM = Transforms.FromFrame(_base.Frame);
        var worldLinks = new List<(int index, CollisionObject geom)>();
        foreach (var link in _robotCollision.Links)
        {
            if (link.LinkIndex < 0 || link.LinkIndex >= linkMats.Count) continue;
            var worldM = Transforms.Multiply(baseM, linkMats[link.LinkIndex]);
            worldLinks.Add((link.LinkIndex, CollisionGeometry.Transform(link.LocalGeometry, worldM)));
        }
        for (var i = 0; i < worldLinks.Count; i++)
        {
            for (var j = i + 2; j < worldLinks.Count; j++)
            {
                if (Math.Abs(worldLinks[i].index - worldLinks[j].index) <= 2) continue;
                if (CollisionGeometry.Intersects(worldLinks[i].geom, worldLinks[j].geom, _meshBvhCache))
                    return false;
            }
        }
        return true;
    }
}
