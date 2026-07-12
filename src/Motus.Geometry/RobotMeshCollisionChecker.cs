using System.Linq;
using Motus.Core;

namespace Motus.Geometry;

/// <summary>Collision checker using per-link robot geometry when available; sphere fallback otherwise.</summary>
public sealed class RobotMeshCollisionChecker : ICollisionChecker
{
    private readonly IFkSolver _fk;
    private readonly BaseFrame _base;
    private readonly ToolFrame _tool;
    private readonly RobotCollisionModel? _robotCollision;
    private readonly SphereCollisionChecker _fallback;
    private readonly IReadOnlyList<AttachedBody> _attached;
    private readonly Dictionary<string, BvhNode> _meshBvhCache = new();

    public RobotMeshCollisionChecker(RobotModel robot, SerialJointChain? chain = null, IReadOnlyList<AttachedBody>? attached = null)
    {
        _fk = KinematicsResolver.CreateFkSolver(robot.Preset, chain);
        _base = robot.Preset.BaseFrame;
        _tool = robot.Preset.ToolFrame;
        _robotCollision = robot.CollisionModel;
        _attached = attached ?? Array.Empty<AttachedBody>();
        _fallback = new SphereCollisionChecker(_fk, _base);
    }

    public bool IsCollisionFree(JointState state, CollisionScene scene)
    {
        if (_robotCollision is null || _robotCollision.Links.Count == 0)
            return _fallback.IsCollisionFree(state, scene);

        BuildBvhCache(scene);
        if (!SelfCollisionFree(state)) return false;
        if (!ToolSceneCollisionFree(state, scene)) return false;
        if (_attached.Count > 0 && !AttachedBodiesCollisionFree(state, scene)) return false;

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
        if (stepRadians <= 0) stepRadians = 1e-3;
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

    private bool ToolSceneCollisionFree(JointState state, CollisionScene scene)
    {
        if (_robotCollision?.ToolGeometry is not { } toolGeom || scene.Objects.Count == 0)
            return true;

        var tcpM = _fk.ComputeTcpTransform(state.Positions, _base.Frame, _tool.Frame);
        var toolWorld = CollisionGeometry.Transform(toolGeom, tcpM);
        foreach (var obj in scene.Objects)
        {
            if (scene.IsPairAllowed(toolWorld.Name, obj.Name)) continue;
            if (CollisionGeometry.Intersects(toolWorld, obj, _meshBvhCache)) return false;
        }
        return true;
    }

    private bool AttachedBodiesCollisionFree(JointState state, CollisionScene scene)
    {
        if (_attached.Count == 0) return true;

        var tcpM = _fk.ComputeTcpTransform(state.Positions, _base.Frame, _tool.Frame);
        var linkMats = _fk.ComputeLinkTransforms(state.Positions);
        var baseM = Transforms.FromFrame(_base.Frame);

        foreach (var body in _attached)
        {
            var localM = Transforms.Multiply(tcpM, Transforms.FromFrame(body.TcpLocalPose));
            var att = CollisionGeometry.Transform(body.Geometry, localM);

            foreach (var obj in scene.Objects)
            {
                if (scene.IsPairAllowed(att.Name, obj.Name)) continue;
                if (CollisionGeometry.Intersects(att, obj, _meshBvhCache)) return false;
            }

            if (_robotCollision is null) continue;
            foreach (var link in _robotCollision.Links)
            {
                if (link.LinkIndex < 0 || link.LinkIndex >= linkMats.Count) continue;
                if (scene.IsPairAllowed(att.Name, CollisionBodies.RobotLink(link.LinkIndex))) continue;
                var worldM = Transforms.Multiply(baseM, linkMats[link.LinkIndex]);
                var linkGeom = CollisionGeometry.Transform(link.LocalGeometry, worldM);
                if (CollisionGeometry.Intersects(att, linkGeom, _meshBvhCache)) return false;
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
        if (_robotCollision.ToolGeometry is { } tool)
        {
            var tcpM = _fk.ComputeTcpTransform(state.Positions, _base.Frame, _tool.Frame);
            worldLinks.Add((linkMats.Count - 1, CollisionGeometry.Transform(tool, tcpM)));
        }
        for (var i = 0; i < worldLinks.Count; i++)
        {
            for (var j = i + 2; j < worldLinks.Count; j++)
            {
                if (Math.Abs(worldLinks[i].index - worldLinks[j].index) <= 3) continue;
                if (CollisionGeometry.Intersects(worldLinks[i].geom, worldLinks[j].geom, _meshBvhCache))
                    return false;
            }
        }
        return true;
    }
}
