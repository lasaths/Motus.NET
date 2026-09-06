using Motus.Core;
using Motus.Native;

namespace Motus.Geometry;

/// <summary>FCL-backed collision when motus_native is built with FCL; falls back to mesh checker.</summary>
public sealed class FclCollisionChecker : ICollisionChecker, IDisposable
{
    private const uint LinkIdBase = 0x01000000;
    private const uint AttachedIdBase = 0x02000000;
    private const uint ObstacleIdBase = 0x03000000;
    private const uint ToolId = 0x0100FF00;

    private readonly RobotModel _robot;
    private readonly IFkSolver _fk;
    private readonly BaseFrame _base;
    private readonly ToolFrame _tool;
    private readonly IReadOnlyList<AttachedBody> _attached;
    private readonly ICollisionChecker _fallback;
    private readonly bool _useFcl;
    private IntPtr _world;
    private readonly HashSet<uint> _obstacleIds = new();
    private readonly Dictionary<string, uint> _nameToId = new(StringComparer.OrdinalIgnoreCase);
    private int _sceneHash;

    public FclCollisionChecker(RobotModel robot, SerialJointChain? chain = null, IReadOnlyList<AttachedBody>? attached = null)
    {
        _robot = robot;
        _fk = KinematicsResolver.CreateFkSolver(robot.Preset, chain);
        _base = robot.Preset.BaseFrame;
        _tool = robot.Preset.ToolFrame;
        _attached = attached ?? Array.Empty<AttachedBody>();
        _fallback = CreateFallback(robot, chain, attached);
        _useFcl = IsAvailable && SupportsFcl(robot, _attached);
        if (_useFcl)
        {
            lock (NativeSync.Gate)
            {
                _world = NativeBindings.motus_fcl_world_create();
                RegisterSelfAllowedPairs();
            }
        }
        else
        {
            _world = IntPtr.Zero;
        }
    }

    public static bool IsAvailable => NativeBindings.FclIsAvailable();

    public static bool SupportsFcl(RobotModel robot, IReadOnlyList<AttachedBody>? attached = null)
    {
        if (!IsAvailable) return false;
        if (robot.CollisionModel is null || robot.CollisionModel.Links.Count == 0) return false;
        if (robot.CollisionModel.Links.Any(l => l.LocalGeometry.Shape == CollisionShape.Mesh)) return false;
        if (robot.CollisionModel.ToolGeometry?.Shape == CollisionShape.Mesh) return false;
        if (attached is not null)
        {
            foreach (var body in attached)
                if (body.Geometry.Shape == CollisionShape.Mesh) return false;
        }
        return true;
    }

    private static ICollisionChecker CreateFallback(
        RobotModel robot, SerialJointChain? chain, IReadOnlyList<AttachedBody>? attached)
    {
        if (robot.CollisionModel is not null && (robot.CollisionModel.Links.Count > 0 || robot.CollisionModel.ToolGeometry is not null))
            return new RobotMeshCollisionChecker(robot, chain, attached);
        return chain is null
            ? new SphereCollisionChecker(robot.Preset)
            : new SphereCollisionChecker(robot.Preset, chain);
    }

    public bool IsCollisionFree(JointState state, CollisionScene scene)
    {
        if (!_useFcl || HasMeshObstacle(scene))
            return _fallback.IsCollisionFree(state, scene);

        lock (NativeSync.Gate)
        {
            ApplyScene(scene);
            UpdateRobot(state);
            UpdateAttached(state);
            return NativeBindings.motus_fcl_check(_world, out _, out _) == NativeBindings.Ok;
        }
    }

    public bool SegmentCollisionFree(JointState from, JointState to, CollisionScene scene, double stepRadians)
    {
        if (!_useFcl || HasMeshObstacle(scene))
            return _fallback.SegmentCollisionFree(from, to, scene, stepRadians);

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
            if (!IsCollisionFree(new JointState(q), scene)) return false;
        }
        return true;
    }

    ~FclCollisionChecker() => Dispose();

    public void Dispose()
    {
        lock (NativeSync.Gate)
        {
            if (_world != IntPtr.Zero)
            {
                NativeBindings.motus_fcl_world_destroy(_world);
                _world = IntPtr.Zero;
            }
        }
        GC.SuppressFinalize(this);
    }

    // Plane has no FCL upsert yet — fall back so planes are not silently ignored.
    private static bool HasMeshObstacle(CollisionScene scene) =>
        scene.Objects.Any(o => o.Shape is CollisionShape.Mesh or CollisionShape.Plane);

    private void ApplyScene(CollisionScene scene)
    {
        var hash = HashScene(scene);
        if (hash == _sceneHash) return;
        _sceneHash = hash;

        if (_world != IntPtr.Zero)
            NativeBindings.motus_fcl_world_destroy(_world);
        _world = NativeBindings.motus_fcl_world_create();
        _obstacleIds.Clear();
        _nameToId.Clear();

        foreach (var link in _robot.CollisionModel!.Links)
            _nameToId[CollisionBodies.RobotLink(link.LinkIndex)] = LinkIdBase | (uint)link.LinkIndex;
        if (_robot.CollisionModel.ToolGeometry is not null)
            _nameToId["tool"] = ToolId;

        foreach (var body in _attached)
            _nameToId[CollisionBodies.Attached(body.Name)] = AttachedIdBase ^ FnvHash(body.Name);

        foreach (var obj in scene.Objects)
        {
            var id = ObstacleIdBase ^ FnvHash(obj.Name);
            _obstacleIds.Add(id);
            _nameToId[obj.Name] = id;
            UpsertGeometry(_world, id, obj);
        }

        foreach (var (a, b) in scene.AllowedPairs)
        {
            if (_nameToId.TryGetValue(a, out var idA) && _nameToId.TryGetValue(b, out var idB))
                NativeBindings.motus_fcl_set_allowed_pair(_world, idA, idB);
        }

        RegisterSelfAllowedPairs();
    }

    private void RegisterSelfAllowedPairs()
    {
        var links = _robot.CollisionModel!.Links;
        for (var i = 0; i < links.Count; i++)
        {
            var idA = LinkIdBase | (uint)links[i].LinkIndex;
            for (var j = i + 1; j < links.Count; j++)
            {
                if (Math.Abs(links[i].LinkIndex - links[j].LinkIndex) > 3) continue;
                var idB = LinkIdBase | (uint)links[j].LinkIndex;
                NativeBindings.motus_fcl_set_allowed_pair(_world, idA, idB);
            }
        }
        if (_robot.CollisionModel.ToolGeometry is not null && links.Count > 0)
        {
            var lastIndex = links[^1].LinkIndex;
            foreach (var link in links)
            {
                if (Math.Abs(link.LinkIndex - lastIndex) <= 3)
                    NativeBindings.motus_fcl_set_allowed_pair(_world, LinkIdBase | (uint)link.LinkIndex, ToolId);
            }
        }
    }

    private void UpdateRobot(JointState state)
    {
        var model = _robot.CollisionModel!;
        var linkMats = _fk.ComputeLinkTransforms(state.Positions);
        var baseM = Transforms.FromFrame(_base.Frame);
        foreach (var link in model.Links)
        {
            if (link.LinkIndex < 0 || link.LinkIndex >= linkMats.Count) continue;
            var worldM = Transforms.Multiply(baseM, linkMats[link.LinkIndex]);
            var worldGeom = CollisionGeometry.Transform(link.LocalGeometry, worldM);
            UpsertGeometry(_world, LinkIdBase | (uint)link.LinkIndex, worldGeom);
        }

        if (model.ToolGeometry is { } tool)
        {
            var toolM = ToolCollisionPlacement.WorldMatrix(
                _fk, state.Positions, _base, _tool, tool,
                model.ToolGeometryInFlangeFrame,
                model.ToolGeometryAttachOffset);
            var worldTool = CollisionGeometry.Transform(tool, toolM);
            UpsertGeometry(_world, ToolId, worldTool);
        }
    }

    private void UpdateAttached(JointState state)
    {
        if (_attached.Count == 0) return;
        var tcpM = _fk.ComputeTcpTransform(state.Positions, _base.Frame, _tool.Frame);
        foreach (var body in _attached)
        {
            var localM = Transforms.Multiply(tcpM, Transforms.FromFrame(body.TcpLocalPose));
            var worldGeom = CollisionGeometry.Transform(body.Geometry, localM);
            var id = AttachedIdBase ^ FnvHash(body.Name);
            UpsertGeometry(_world, id, worldGeom);
        }
    }

    private static void UpsertGeometry(IntPtr world, uint id, CollisionObject geom)
    {
        var pose = NativeBindings.MotusTransform.FromMatrix(Transforms.FromFrame(geom.Pose));
        switch (geom.Shape)
        {
            case CollisionShape.Sphere:
                NativeBindings.motus_fcl_upsert_sphere(world, id, ref pose, geom.ExtentX);
                break;
            case CollisionShape.Box:
                NativeBindings.motus_fcl_upsert_box(world, id, ref pose, geom.ExtentX, geom.ExtentY, geom.ExtentZ);
                break;
            case CollisionShape.Capsule:
                NativeBindings.motus_fcl_upsert_capsule(world, id, ref pose, geom.ExtentX, geom.ExtentY);
                break;
        }
    }

    private static int HashScene(CollisionScene scene)
    {
        var hash = new HashCode();
        foreach (var o in scene.Objects)
        {
            hash.Add(o.Name);
            hash.Add(o.Shape);
            hash.Add(o.Pose.X);
            hash.Add(o.Pose.Y);
            hash.Add(o.Pose.Z);
            hash.Add(o.Pose.Qw);
            hash.Add(o.Pose.Qx);
            hash.Add(o.Pose.Qy);
            hash.Add(o.Pose.Qz);
            hash.Add(o.ExtentX);
            hash.Add(o.ExtentY);
            hash.Add(o.ExtentZ);
        }
        foreach (var p in scene.AllowedPairs)
        {
            hash.Add(p.A);
            hash.Add(p.B);
        }
        return hash.ToHashCode();
    }

    private static uint FnvHash(string s)
    {
        uint h = 2166136261;
        foreach (var c in s)
            h = (h ^ c) * 16777619;
        return h;
    }
}
