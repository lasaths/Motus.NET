using Motus.Core;

namespace Motus.Geometry;

/// <summary>Collision checker using per-link robot geometry when available; sphere fallback otherwise.</summary>
public sealed class RobotMeshCollisionChecker : ICollisionChecker, ICollisionContactReporter
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
    private double[][]? _linkMats;
    private readonly double[] _baseM;
    private int _sceneFingerprint;
    private bool _sceneFingerprintValid;
    private readonly List<(int index, LinkCollisionEntry entry, double[] linkMat, double[] worldM)> _posedScratch = new();
    private readonly List<double[]> _linkMatPool = new();
    private readonly List<double[]> _worldMatPool = new();
    private readonly double[] _linkWorldScratch = new double[16];
    private readonly double[] _worldScratch = new double[16];
    private readonly double[] _poseScratch = new double[16];
    private readonly int _selfMinLinkGap;
    private readonly IReadOnlyList<string> _linkNames;

    /// <summary>Default self-collision link-index gap (pairs closer than this in the chain are skipped).</summary>
    public const int DefaultSelfCollisionMinLinkGap = 4;

    private sealed class LinkCollisionEntry
    {
        public required CollisionObject Geometry { get; init; }
        public int LinkIndex { get; init; }
        public BvhNode? LocalBvh { get; init; }
        public double EnvelopeRadius { get; init; }
    }

    public RobotMeshCollisionChecker(RobotModel robot, SerialJointChain? chain = null, IReadOnlyList<AttachedBody>? attached = null)
        : this(robot, chain, attached, DefaultSelfCollisionMinLinkGap)
    {
    }

    /// <param name="selfCollisionMinLinkGap">
    /// Link pairs whose FK indices differ by less than this are not self-checked (adjacent links share a joint).
    /// Default 4 keeps the historical industrial-arm behaviour; compact arms with explicit per-part
    /// geometry (e.g. the ICD bamboo arm) use 2 so only directly-jointed links are skipped.
    /// </param>
    public RobotMeshCollisionChecker(
        RobotModel robot,
        SerialJointChain? chain,
        IReadOnlyList<AttachedBody>? attached,
        int selfCollisionMinLinkGap)
    {
        _selfMinLinkGap = Math.Max(1, selfCollisionMinLinkGap);
        _fk = KinematicsResolver.CreateFkSolver(robot.Preset, chain);
        _base = robot.Preset.BaseFrame;
        _tool = robot.Preset.ToolFrame;
        _robotCollision = robot.CollisionModel;
        _attached = attached ?? Array.Empty<AttachedBody>();
        _fallback = new SphereCollisionChecker(_fk, _base);
        _baseM = Transforms.FromFrame(_base.Frame);

        if (_robotCollision is not null)
        {
            foreach (var link in _robotCollision.Links)
                _links.Add(BuildEntry(link.LocalGeometry, link.LinkIndex));

            if (_robotCollision.ToolGeometry is { } tool)
                _toolEntry = BuildEntry(tool, -1);
        }

        foreach (var body in _attached)
            _attachEntries.Add((body, BuildEntry(body.Geometry, -1)));

        var names = new List<string>();
        if (_robotCollision is not null)
        {
            foreach (var link in _robotCollision.Links)
            {
                while (names.Count <= link.LinkIndex) names.Add(CollisionBodies.RobotLink(names.Count));
                names[link.LinkIndex] = link.LinkName;
            }
        }
        _linkNames = names;
    }

    private static LinkCollisionEntry BuildEntry(CollisionObject geom, int linkIndex)
    {
        BvhNode? bvh = null;
        double envelope;
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

    public bool IsCollisionFree(JointState state, CollisionScene scene) => Evaluate(state, scene, null, 0);

    /// <summary>
    /// Every colliding body pair at <paramref name="state"/> (self, robot–scene, tool, attached), named by
    /// collision geometry (URDF <c>&lt;collision name=...&gt;</c> or <c>{link}_col{i}</c>) and scene object name.
    /// Empty when collision-free. Same narrow phase and allowed-pair filtering as <see cref="IsCollisionFree"/>.
    /// </summary>
    public IReadOnlyList<CollisionContact> FindContacts(JointState state, CollisionScene scene, int maxContacts = 64)
    {
        var sink = new List<CollisionContact>();
        Evaluate(state, scene, sink, Math.Max(1, maxContacts));
        return sink;
    }

    private bool Evaluate(JointState state, CollisionScene scene, List<CollisionContact>? sink, int max)
    {
        if (_robotCollision is null || _robotCollision.Links.Count == 0)
        {
            var free = _fallback.IsCollisionFree(state, scene);
            if (!free && sink is not null) sink.Add(new CollisionContact("robot", "scene", CollisionContactKind.Scene));
            return free;
        }

        EnsureBvhCache(scene);
        var linkMats = EnsureLinkMats(state.Positions.Length);
        _fk.ComputeLinkTransformsInto(state.Positions, linkMats);

        var clear = true;
        if (!SelfCollisionFree(state, scene, linkMats, sink, max)) { clear = false; if (Full(sink, max)) return false; }
        if (!ToolSceneCollisionFree(state, scene, sink, max)) { clear = false; if (Full(sink, max)) return false; }
        if (_attached.Count > 0 && !AttachedBodiesCollisionFree(state, scene, linkMats, sink, max)) { clear = false; if (Full(sink, max)) return false; }

        foreach (var obj in scene.Objects)
        {
            foreach (var entry in _links)
            {
                if (entry.LinkIndex < 0 || entry.LinkIndex >= linkMats.Length) continue;
                if (scene.IsPairAllowed(CollisionBodies.RobotLink(entry.LinkIndex), obj.Name))
                    continue;
                if (scene.IsPairAllowed(entry.Geometry.Name, obj.Name))
                    continue;
                ComposeWorldInto(_worldScratch, _baseM, linkMats[entry.LinkIndex], entry.Geometry.Pose);
                if (!CollisionGeometry.EnvelopeMayHit(entry.Geometry, _worldScratch, entry.EnvelopeRadius, obj, _meshBvhCache, _scratch))
                    continue;
                if (CollisionGeometry.IntersectsAtPose(entry.Geometry, _worldScratch, obj, _meshBvhCache, _scratch))
                {
                    if (sink is null) return false;
                    clear = false;
                    sink.Add(new CollisionContact(entry.Geometry.Name, obj.Name, CollisionContactKind.Scene, LinkName(entry.LinkIndex), null));
                    if (sink.Count >= max) return false;
                }
            }
        }
        return clear;
    }

    private static bool Full(List<CollisionContact>? sink, int max) => sink is null || sink.Count >= max;

    private string? LinkName(int index) =>
        index >= 0 && index < _linkNames.Count ? _linkNames[index] : null;

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

    private bool ToolSceneCollisionFree(JointState state, CollisionScene scene, List<CollisionContact>? sink, int max)
    {
        if (_toolEntry is null || scene.Objects.Count == 0 || _robotCollision is null)
            return true;

        var toolM = ToolCollisionPlacement.WorldMatrix(
            _fk, state.Positions, _base, _tool, _toolEntry.Geometry,
            _robotCollision.ToolGeometryInFlangeFrame,
            _robotCollision.ToolGeometryAttachOffset);
        var worldM = CollisionGeometry.ComposeWorldMatrix(toolM, _toolEntry.Geometry.Pose);
        var clear = true;
        foreach (var obj in scene.Objects)
        {
            if (scene.IsPairAllowed(_toolEntry.Geometry.Name, obj.Name)) continue;
            if (!CollisionGeometry.EnvelopeMayHit(_toolEntry.Geometry, worldM, _toolEntry.EnvelopeRadius, obj, _meshBvhCache, _scratch))
                continue;
            if (CollisionGeometry.IntersectsAtPose(_toolEntry.Geometry, worldM, obj, _meshBvhCache, _scratch))
            {
                if (sink is null) return false;
                clear = false;
                sink.Add(new CollisionContact(_toolEntry.Geometry.Name, obj.Name, CollisionContactKind.Tool));
                if (sink.Count >= max) return false;
            }
        }
        return clear;
    }

    private bool AttachedBodiesCollisionFree(
        JointState state, CollisionScene scene, double[][] linkMats, List<CollisionContact>? sink, int max)
    {
        if (_attachEntries.Count == 0) return true;
        var clear = true;

        var tcpM = _fk.ComputeTcpTransform(state.Positions, _base.Frame, _tool.Frame);

        foreach (var (body, entry) in _attachEntries)
        {
            var localM = Transforms.Multiply(tcpM, Transforms.FromFrame(body.TcpLocalPose));
            var attWorldM = CollisionGeometry.ComposeWorldMatrix(localM, entry.Geometry.Pose);

            foreach (var obj in scene.Objects)
            {
                if (scene.IsPairAllowed(entry.Geometry.Name, obj.Name)) continue;
                if (!CollisionGeometry.EnvelopeMayHit(entry.Geometry, attWorldM, entry.EnvelopeRadius, obj, _meshBvhCache, _scratch))
                    continue;
                if (CollisionGeometry.IntersectsAtPose(entry.Geometry, attWorldM, obj, _meshBvhCache, _scratch))
                {
                    if (sink is null) return false;
                    clear = false;
                    sink.Add(new CollisionContact(entry.Geometry.Name, obj.Name, CollisionContactKind.Attached));
                    if (sink.Count >= max) return false;
                }
            }

            foreach (var link in _links)
            {
                if (link.LinkIndex < 0 || link.LinkIndex >= linkMats.Length) continue;
                if (scene.IsPairAllowed(entry.Geometry.Name, CollisionBodies.RobotLink(link.LinkIndex))) continue;
                if (scene.IsPairAllowed(entry.Geometry.Name, link.Geometry.Name)) continue;
                var linkMat = Transforms.Multiply(_baseM, linkMats[link.LinkIndex]);
                var linkWorldM = CollisionGeometry.ComposeWorldMatrix(linkMat, link.Geometry.Pose);

                CollisionGeometry.TransformLocalAabbToWorld(entry.Geometry, attWorldM, _scratch.WorldAabbMin, _scratch.WorldAabbMax);
                CollisionGeometry.TransformLocalAabbToWorld(link.Geometry, linkWorldM, _scratch.WorldAabbMinB, _scratch.WorldAabbMaxB);
                if (!CollisionGeometry.AabbAabbOverlap(
                        _scratch.WorldAabbMin, _scratch.WorldAabbMax,
                        _scratch.WorldAabbMinB, _scratch.WorldAabbMaxB))
                    continue;

                bool hit;
                if (entry.Geometry.Shape == CollisionShape.Mesh && link.Geometry.Shape == CollisionShape.Mesh)
                    hit = CollisionGeometry.IntersectsMeshesAtPoses(
                        entry.Geometry, attWorldM, link.Geometry, linkWorldM, link.LocalBvh, _scratch);
                else
                    hit = CollisionGeometry.IntersectsAtPose(
                        entry.Geometry, attWorldM,
                        CollisionGeometry.Transform(link.Geometry, linkMat),
                        _meshBvhCache, _scratch);
                if (hit)
                {
                    if (sink is null) return false;
                    clear = false;
                    sink.Add(new CollisionContact(entry.Geometry.Name, link.Geometry.Name, CollisionContactKind.Attached, null, LinkName(link.LinkIndex)));
                    if (sink.Count >= max) return false;
                }
            }
        }
        return clear;
    }

    private void EnsureBvhCache(CollisionScene scene)
    {
        var fp = SceneFingerprint(scene);
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
        _sceneFingerprint = fp;
        _sceneFingerprintValid = true;
    }

    private static int SceneFingerprint(CollisionScene scene)
    {
        var hash = new HashCode();
        hash.Add(scene.Objects.Count);
        hash.Add(scene.AllowedPairs.Count);
        for (var i = 0; i < scene.Objects.Count; i++)
        {
            var o = scene.Objects[i];
            hash.Add(o.ContentHash);
            hash.Add(o.Pose.X);
            hash.Add(o.Pose.Y);
            hash.Add(o.Pose.Z);
            hash.Add(o.Pose.Qw);
            hash.Add(o.Pose.Qx);
            hash.Add(o.Pose.Qy);
            hash.Add(o.Pose.Qz);
        }
        return hash.ToHashCode();
    }

    private double[][] EnsureLinkMats(int n)
    {
        if (_linkMats is not null && _linkMats.Length == n)
            return _linkMats;
        _linkMats = new double[n][];
        for (var i = 0; i < n; i++)
            _linkMats[i] = new double[16];
        return _linkMats;
    }

    private bool SelfCollisionFree(
        JointState state, CollisionScene scene, double[][] linkMats, List<CollisionContact>? sink, int max)
    {
        _posedScratch.Clear();
        var posedIdx = 0;

        foreach (var entry in _links)
        {
            if (entry.LinkIndex < 0 || entry.LinkIndex >= linkMats.Length) continue;
            var linkMat = RentMat(_linkMatPool, posedIdx);
            Transforms.MultiplyInto(linkMat, _baseM, linkMats[entry.LinkIndex]);
            var worldM = RentMat(_worldMatPool, posedIdx);
            ComposeWorldInto(worldM, linkMat, entry.Geometry.Pose);
            _posedScratch.Add((entry.LinkIndex, entry, linkMat, worldM));
            posedIdx++;
        }

        if (_toolEntry is not null && _robotCollision is not null)
        {
            var toolM = ToolCollisionPlacement.WorldMatrix(
                _fk, state.Positions, _base, _tool, _toolEntry.Geometry,
                _robotCollision.ToolGeometryInFlangeFrame,
                _robotCollision.ToolGeometryAttachOffset);
            var worldM = RentMat(_worldMatPool, posedIdx);
            ComposeWorldInto(worldM, toolM, _toolEntry.Geometry.Pose);
            var linkMat = RentMat(_linkMatPool, posedIdx);
            Array.Copy(toolM, linkMat, 16);
            _posedScratch.Add((linkMats.Length - 1, _toolEntry, linkMat, worldM));
        }

        var posed = _posedScratch;
        var clear = true;
        for (var i = 0; i < posed.Count; i++)
        {
            for (var j = i + 1; j < posed.Count; j++)
            {
                if (Math.Abs(posed[i].index - posed[j].index) < _selfMinLinkGap) continue;

                var a = posed[i];
                var b = posed[j];
                // Conservative bounding-sphere cull before the exact narrow phase.
                EnvelopeCenter(a.entry, a.worldM, out var acx, out var acy, out var acz);
                EnvelopeCenter(b.entry, b.worldM, out var bcx, out var bcy, out var bcz);
                var rr = a.entry.EnvelopeRadius + b.entry.EnvelopeRadius;
                var ddx = acx - bcx; var ddy = acy - bcy; var ddz = acz - bcz;
                if (ddx * ddx + ddy * ddy + ddz * ddz > rr * rr) continue;
                if (scene.IsPairAllowed(a.entry.Geometry.Name, b.entry.Geometry.Name)) continue;
                if (a.entry.LinkIndex >= 0 && b.entry.LinkIndex >= 0 &&
                    scene.IsPairAllowed(CollisionBodies.RobotLink(a.entry.LinkIndex), CollisionBodies.RobotLink(b.entry.LinkIndex)))
                    continue;

                bool hit;
                if (a.entry.Geometry.Shape == CollisionShape.Mesh && b.entry.Geometry.Shape == CollisionShape.Mesh)
                    hit = CollisionGeometry.IntersectsMeshesAtPoses(
                        a.entry.Geometry, a.worldM, b.entry.Geometry, b.worldM, b.entry.LocalBvh, _scratch);
                else
                    hit = CollisionGeometry.IntersectsAtPose(
                        a.entry.Geometry, a.worldM,
                        CollisionGeometry.Transform(b.entry.Geometry, b.linkMat),
                        _meshBvhCache, _scratch);
                if (!hit) continue;
                if (sink is null) return false;
                clear = false;
                sink.Add(new CollisionContact(a.entry.Geometry.Name, b.entry.Geometry.Name, CollisionContactKind.Self,
                    LinkName(a.entry.LinkIndex), LinkName(b.entry.LinkIndex)));
                if (sink.Count >= max) return false;
            }
        }
        return clear;
    }

    private static void EnvelopeCenter(LinkCollisionEntry entry, double[] worldM, out double x, out double y, out double z)
    {
        if (entry.Geometry.Shape == CollisionShape.Mesh)
        {
            CollisionGeometry.MeshEnvelopeCenterLocal(entry.Geometry, out var lx, out var ly, out var lz);
            Transforms.TransformPointInto(worldM, lx, ly, lz, out x, out y, out z);
            return;
        }
        x = worldM[3]; y = worldM[7]; z = worldM[11];
    }

    private void ComposeWorldInto(double[] dest, double[] linkWorldMatrix, Frame localPose)
    {
        Transforms.FromFrameInto(_poseScratch, localPose);
        Transforms.MultiplyInto(dest, linkWorldMatrix, _poseScratch);
    }

    private void ComposeWorldInto(double[] dest, double[] baseM, double[] linkMat, Frame localPose)
    {
        Transforms.MultiplyInto(_linkWorldScratch, baseM, linkMat);
        ComposeWorldInto(dest, _linkWorldScratch, localPose);
    }

    private static double[] RentMat(List<double[]> pool, int index)
    {
        while (pool.Count <= index)
            pool.Add(new double[16]);
        return pool[index];
    }
}
