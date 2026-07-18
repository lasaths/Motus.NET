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
    private readonly Dictionary<int, BvhNode> _meshBvhCache = new();
    private readonly CollisionQueryScratch _scratch = new();
    private readonly List<LinkCollisionEntry> _links = new();
    private readonly LinkCollisionEntry? _toolEntry;
    private readonly List<(AttachedBody body, LinkCollisionEntry entry)> _attachEntries = new();
    private double[]? _segmentQ;
    private JointState? _segmentState;

    private sealed class LinkCollisionEntry
    {
        public required CollisionObject Geometry { get; init; }
        public int LinkIndex { get; init; }
        public BvhNode? LocalBvh { get; init; }
        public double EnvelopeRadius { get; init; }
    }

    public RobotMeshCollisionChecker(RobotModel robot, SerialJointChain? chain = null, IReadOnlyList<AttachedBody>? attached = null)
    {
        _fk = KinematicsResolver.CreateFkSolver(robot.Preset, chain);
        _base = robot.Preset.BaseFrame;
        _tool = robot.Preset.ToolFrame;
        _robotCollision = robot.CollisionModel;
        _attached = attached ?? Array.Empty<AttachedBody>();
        _fallback = new SphereCollisionChecker(_fk, _base);

        if (_robotCollision is not null)
        {
            foreach (var link in _robotCollision.Links)
                _links.Add(BuildEntry(link.LocalGeometry, link.LinkIndex));

            if (_robotCollision.ToolGeometry is { } tool)
                _toolEntry = BuildEntry(tool, -1);
        }

        foreach (var body in _attached)
            _attachEntries.Add((body, BuildEntry(body.Geometry, -1)));
    }

    private static LinkCollisionEntry BuildEntry(CollisionObject geom, int linkIndex)
    {
        BvhNode? bvh = null;
        var envelope = 0.01;
        if (geom.Shape == CollisionShape.Mesh &&
            geom.MeshVertices is not null &&
            geom.MeshIndices is not null &&
            geom.MeshIndices.Count >= 3)
        {
            bvh = CollisionMeshCache.GetOrBuild(geom);
            envelope = CollisionGeometry.MeshEnvelopeRadius(geom);
        }
        else
        {
            envelope = geom.Shape switch
            {
                CollisionShape.Sphere => geom.ExtentX,
                CollisionShape.Capsule => geom.ExtentX + geom.ExtentY,
                CollisionShape.Box => Math.Sqrt(
                    geom.ExtentX * geom.ExtentX + geom.ExtentY * geom.ExtentY + geom.ExtentZ * geom.ExtentZ),
                _ => 0.01
            };
        }

        return new LinkCollisionEntry
        {
            Geometry = geom,
            LinkIndex = linkIndex,
            LocalBvh = bvh,
            EnvelopeRadius = envelope
        };
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
            foreach (var entry in _links)
            {
                if (entry.LinkIndex < 0 || entry.LinkIndex >= linkMats.Count) continue;
                if (scene.IsPairAllowed(CollisionBodies.RobotLink(entry.LinkIndex), obj.Name))
                    continue;
                var worldM = CollisionGeometry.ComposeWorldMatrix(
                    Transforms.Multiply(baseM, linkMats[entry.LinkIndex]), entry.Geometry.Pose);
                if (!CollisionGeometry.EnvelopeMayHit(entry.Geometry, worldM, entry.EnvelopeRadius, obj, _meshBvhCache, _scratch))
                    continue;
                if (CollisionGeometry.IntersectsAtPose(entry.Geometry, worldM, obj, _meshBvhCache, _scratch))
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
            if (!IsCollisionFree(state, scene))
                return false;
        }
        return true;
    }

    private bool ToolSceneCollisionFree(JointState state, CollisionScene scene)
    {
        if (_toolEntry is null || scene.Objects.Count == 0 || _robotCollision is null)
            return true;

        var toolM = ToolCollisionPlacement.WorldMatrix(
            _fk, state.Positions, _base, _tool, _toolEntry.Geometry,
            _robotCollision.ToolGeometryInFlangeFrame,
            _robotCollision.ToolGeometryAttachOffset);
        var worldM = CollisionGeometry.ComposeWorldMatrix(toolM, _toolEntry.Geometry.Pose);
        foreach (var obj in scene.Objects)
        {
            if (scene.IsPairAllowed(_toolEntry.Geometry.Name, obj.Name)) continue;
            if (!CollisionGeometry.EnvelopeMayHit(_toolEntry.Geometry, worldM, _toolEntry.EnvelopeRadius, obj, _meshBvhCache, _scratch))
                continue;
            if (CollisionGeometry.IntersectsAtPose(_toolEntry.Geometry, worldM, obj, _meshBvhCache, _scratch))
                return false;
        }
        return true;
    }

    private bool AttachedBodiesCollisionFree(JointState state, CollisionScene scene)
    {
        if (_attachEntries.Count == 0) return true;

        var tcpM = _fk.ComputeTcpTransform(state.Positions, _base.Frame, _tool.Frame);
        var linkMats = _fk.ComputeLinkTransforms(state.Positions);
        var baseM = Transforms.FromFrame(_base.Frame);

        foreach (var (body, entry) in _attachEntries)
        {
            var localM = Transforms.Multiply(tcpM, Transforms.FromFrame(body.TcpLocalPose));
            var attWorldM = CollisionGeometry.ComposeWorldMatrix(localM, entry.Geometry.Pose);

            foreach (var obj in scene.Objects)
            {
                if (scene.IsPairAllowed(attWorldName(entry), obj.Name)) continue;
                if (!CollisionGeometry.EnvelopeMayHit(entry.Geometry, attWorldM, entry.EnvelopeRadius, obj, _meshBvhCache, _scratch))
                    continue;
                if (CollisionGeometry.IntersectsAtPose(entry.Geometry, attWorldM, obj, _meshBvhCache, _scratch))
                    return false;
            }

            foreach (var link in _links)
            {
                if (link.LinkIndex < 0 || link.LinkIndex >= linkMats.Count) continue;
                if (scene.IsPairAllowed(entry.Geometry.Name, CollisionBodies.RobotLink(link.LinkIndex))) continue;
                var linkMat = Transforms.Multiply(baseM, linkMats[link.LinkIndex]);
                var linkWorldM = CollisionGeometry.ComposeWorldMatrix(linkMat, link.Geometry.Pose);

                CollisionGeometry.TransformLocalAabbToWorld(entry.Geometry, attWorldM, _scratch.WorldAabbMin, _scratch.WorldAabbMax);
                CollisionGeometry.TransformLocalAabbToWorld(link.Geometry, linkWorldM, _scratch.WorldAabbMinB, _scratch.WorldAabbMaxB);
                if (!CollisionGeometry.AabbAabbOverlap(
                        _scratch.WorldAabbMin, _scratch.WorldAabbMax,
                        _scratch.WorldAabbMinB, _scratch.WorldAabbMaxB))
                    continue;

                if (entry.Geometry.Shape == CollisionShape.Mesh && link.Geometry.Shape == CollisionShape.Mesh)
                {
                    if (CollisionGeometry.IntersectsMeshesAtPoses(
                            entry.Geometry, attWorldM, link.Geometry, linkWorldM, link.LocalBvh, _scratch))
                        return false;
                }
                else if (CollisionGeometry.IntersectsAtPose(
                             entry.Geometry, attWorldM,
                             CollisionGeometry.Transform(link.Geometry, linkMat),
                             _meshBvhCache, _scratch))
                {
                    return false;
                }
            }
        }
        return true;

        static string attWorldName(LinkCollisionEntry e) => e.Geometry.Name;
    }

    private void BuildBvhCache(CollisionScene scene)
    {
        foreach (var meshObj in scene.Objects.Where(o => o.Shape == CollisionShape.Mesh))
        {
            if (meshObj.MeshVertices is not null && meshObj.MeshIndices is not null)
            {
                var key = CollisionMeshCache.GeometryFingerprint(meshObj);
                _meshBvhCache[key] = CollisionMeshCache.GetOrBuild(meshObj);
            }
        }
    }

    private bool SelfCollisionFree(JointState state)
    {
        if (_robotCollision is null) return _fallback.IsCollisionFree(state, new CollisionScene());

        var linkMats = _fk.ComputeLinkTransforms(state.Positions);
        var baseM = Transforms.FromFrame(_base.Frame);
        var posed = new List<(int index, LinkCollisionEntry entry, double[] linkMat, double[] worldM)>(_links.Count + 1);

        foreach (var entry in _links)
        {
            if (entry.LinkIndex < 0 || entry.LinkIndex >= linkMats.Count) continue;
            var linkMat = Transforms.Multiply(baseM, linkMats[entry.LinkIndex]);
            var worldM = CollisionGeometry.ComposeWorldMatrix(linkMat, entry.Geometry.Pose);
            posed.Add((entry.LinkIndex, entry, linkMat, worldM));
        }

        if (_toolEntry is not null)
        {
            var toolM = ToolCollisionPlacement.WorldMatrix(
                _fk, state.Positions, _base, _tool, _toolEntry.Geometry,
                _robotCollision.ToolGeometryInFlangeFrame,
                _robotCollision.ToolGeometryAttachOffset);
            var worldM = CollisionGeometry.ComposeWorldMatrix(toolM, _toolEntry.Geometry.Pose);
            posed.Add((linkMats.Count - 1, _toolEntry, toolM, worldM));
        }

        for (var i = 0; i < posed.Count; i++)
        {
            for (var j = i + 2; j < posed.Count; j++)
            {
                if (Math.Abs(posed[i].index - posed[j].index) <= 3) continue;

                var a = posed[i];
                var b = posed[j];

                if (a.entry.Geometry.Shape == CollisionShape.Mesh && b.entry.Geometry.Shape == CollisionShape.Mesh)
                {
                    if (CollisionGeometry.IntersectsMeshesAtPoses(
                            a.entry.Geometry, a.worldM, b.entry.Geometry, b.worldM, b.entry.LocalBvh, _scratch))
                        return false;
                    continue;
                }

                if (CollisionGeometry.IntersectsAtPose(
                        a.entry.Geometry, a.worldM,
                        CollisionGeometry.Transform(b.entry.Geometry, b.linkMat),
                        _meshBvhCache, _scratch))
                    return false;
            }
        }
        return true;
    }
}
