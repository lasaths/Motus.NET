using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Scene collision via TreeFK — poses every named collision link (tip path + side branches).
/// Plan joint state may be tip-only or tip+branches (see constructor planJointNames).
/// Unmatched tree drivers use treeDriverHome (open gripper, etc.).
/// Opt-in self-collision and a clearance margin are off by default so existing callers are unchanged.
/// </summary>
public sealed class TreeFkCollisionChecker : ICollisionChecker
{
    private readonly TreeForwardKinematics _treeFk;
    private readonly KinematicTree _tree;
    private readonly IFkSolver? _tipFk;
    private readonly int _tipCount;
    private readonly BaseFrame _base;
    private readonly ToolFrame _tool;
    private readonly RobotCollisionModel _collision;
    private readonly IReadOnlyList<string>? _planJointNames;
    private readonly double[]? _treeDriverHome;
    private readonly IReadOnlyList<AttachedBody> _attached;
    private readonly List<LinkEntry> _links = new();
    private readonly LinkEntry? _toolEntry;
    private readonly Dictionary<int, BvhNode> _meshBvhCache = new();
    private readonly CollisionQueryScratch _scratch = new();
    private readonly double[] _baseM;
    private readonly double[] _driverQ;
    private readonly double[][] _treeMats;
    private readonly int[] _treeLinkOfEntry;
    private readonly double[] _worldScratch = new double[16];
    private readonly double[] _linkScratch = new double[16];
    private readonly bool _checkSelfCollision;
    private readonly double _clearanceMarginM;
    private readonly CollisionObject[]? _grown;
    private readonly double[]? _grownRadius;
    private readonly double[][]? _linkWorld;
    private double[]? _segmentQ;
    private JointState? _segmentState;

    private sealed class LinkEntry
    {
        public required CollisionObject Geometry { get; init; }
        public required string LinkName { get; init; }
        public double EnvelopeRadius { get; init; }
    }

    /// <summary>Last colliding pair name, or a driver-fill error, after <see cref="IsCollisionFree"/> returns false.</summary>
    public string? LastHit { get; private set; }

    public TreeFkCollisionChecker(
        RobotModel robot,
        KinematicTree tree,
        SerialJointChain? tipChain = null,
        IReadOnlyList<string>? planJointNames = null,
        IReadOnlyList<double>? treeDriverHome = null,
        IReadOnlyList<AttachedBody>? attached = null,
        bool checkSelfCollision = false,
        double clearanceMarginM = 0)
    {
        if (robot.CollisionModel is null || robot.CollisionModel.Links.Count == 0)
            throw new ArgumentException("TreeFkCollisionChecker requires a CollisionModel with links.", nameof(robot));
        if (!double.IsFinite(clearanceMarginM) || clearanceMarginM < 0)
            throw new ArgumentException("Clearance must be a finite value >= 0.", nameof(clearanceMarginM));

        _tree = tree;
        _treeFk = new TreeForwardKinematics(tree);
        _planJointNames = planJointNames ?? robot.JointNames;
        _treeDriverHome = treeDriverHome?.ToArray();
        _attached = attached ?? Array.Empty<AttachedBody>();
        _collision = robot.CollisionModel;
        _base = robot.Preset.BaseFrame;
        _tool = robot.Preset.ToolFrame;
        _baseM = Transforms.FromFrame(_base.Frame);
        _driverQ = new double[tree.DriverCount];
        _treeMats = new double[tree.Links.Count][];
        for (var i = 0; i < _treeMats.Length; i++)
            _treeMats[i] = new double[16];
        _checkSelfCollision = checkSelfCollision;
        _clearanceMarginM = clearanceMarginM;

        _tipCount = tipChain?.Joints.Length ?? 0;
        _tipFk = tipChain is not null
            ? KinematicsResolver.CreateFkSolver(robot.Preset, tipChain)
            : null;

        foreach (var link in _collision.Links)
        {
            if (string.IsNullOrWhiteSpace(link.LinkName))
                continue;
            try { _ = tree.IndexOfLink(link.LinkName); }
            catch { continue; }
            _links.Add(BuildEntry(link.LocalGeometry, link.LinkName));
        }

        _treeLinkOfEntry = new int[_links.Count];
        for (var i = 0; i < _links.Count; i++)
            _treeLinkOfEntry[i] = tree.IndexOfLink(_links[i].LinkName);

        if (_collision.ToolGeometry is { } tool)
            _toolEntry = BuildEntry(tool, tool.Name);

        if (_checkSelfCollision)
        {
            _linkWorld = new double[_links.Count][];
            for (var i = 0; i < _links.Count; i++)
                _linkWorld[i] = new double[16];
        }

        if (_clearanceMarginM > 0)
        {
            _grown = new CollisionObject[_links.Count];
            _grownRadius = new double[_links.Count];
            for (var i = 0; i < _links.Count; i++)
            {
                var g = CollisionGeometry.Grow(_links[i].Geometry, _clearanceMarginM);
                _grown[i] = g;
                _grownRadius[i] = g.Shape == CollisionShape.Mesh
                    ? CollisionGeometry.MeshEnvelopeRadius(g)
                    : _links[i].EnvelopeRadius + _clearanceMarginM;
                if (g.Shape == CollisionShape.Mesh && g.MeshVertices is not null && g.MeshIndices is { Count: >= 3 })
                    CollisionMeshCache.GetOrBuild(g);
            }
        }
    }

    private static LinkEntry BuildEntry(CollisionObject geom, string linkName)
    {
        var envelope = geom.Shape switch
        {
            CollisionShape.Sphere => geom.ExtentX,
            CollisionShape.Capsule => geom.ExtentX + geom.ExtentY,
            CollisionShape.Box => Math.Sqrt(
                geom.ExtentX * geom.ExtentX + geom.ExtentY * geom.ExtentY + geom.ExtentZ * geom.ExtentZ),
            CollisionShape.Mesh => CollisionGeometry.MeshEnvelopeRadius(geom),
            _ => 0.01
        };
        if (geom.Shape == CollisionShape.Mesh &&
            geom.MeshVertices is not null &&
            geom.MeshIndices is { Count: >= 3 })
            CollisionMeshCache.GetOrBuild(geom);

        return new LinkEntry
        {
            Geometry = geom,
            LinkName = linkName,
            EnvelopeRadius = envelope
        };
    }

    public bool IsCollisionFree(JointState state, CollisionScene scene)
    {
        LastHit = null;
        if (FillDriverQ(state.Positions) is { } err)
        {
            LastHit = err;
            return false;
        }

        _treeFk.ComputeLinkTransformsInto(_driverQ, _treeMats);

        if (_checkSelfCollision && !SelfCollisionFree(scene))
            return false;

        foreach (var obj in scene.Objects)
        {
            var obstacle = _clearanceMarginM > 0 ? CollisionGeometry.Grow(obj, _clearanceMarginM) : obj;
            for (var i = 0; i < _links.Count; i++)
            {
                var entry = _links[i];
                var li = _treeLinkOfEntry[i];
                if (li < 0 || li >= _treeMats.Length) continue;
                if (scene.IsPairAllowed(CollisionBodies.RobotLink(li), obj.Name) ||
                    scene.IsPairAllowed(entry.LinkName, obj.Name))
                    continue;
                ComposeWorldInto(_worldScratch, _baseM, _treeMats[li], entry.Geometry.Pose);
                if (!CollisionGeometry.EnvelopeMayHit(entry.Geometry, _worldScratch, entry.EnvelopeRadius + _clearanceMarginM, obstacle, _meshBvhCache, _scratch))
                    continue;
                if (CollisionGeometry.IntersectsAtPose(entry.Geometry, _worldScratch, obstacle, _meshBvhCache, _scratch))
                {
                    LastHit = $"{entry.LinkName} vs {obj.Name}";
                    return false;
                }
            }
        }

        if (!ToolSceneCollisionFree(state, scene))
            return false;

        if (_attached.Count > 0 && _tipFk is not null && _tipCount > 0)
        {
            var tipQ = TipSlice(state.Positions, _tipCount);
            var tcpM = _tipFk.ComputeTcpTransform(tipQ, _base.Frame, _tool.Frame);
            foreach (var body in _attached)
            {
                var localM = Transforms.Multiply(tcpM, Transforms.FromFrame(body.TcpLocalPose));
                var attWorldM = CollisionGeometry.ComposeWorldMatrix(localM, body.Geometry.Pose);
                foreach (var obj in scene.Objects)
                {
                    if (scene.IsPairAllowed(body.Geometry.Name, obj.Name)) continue;
                    var obstacle = _clearanceMarginM > 0 ? CollisionGeometry.Grow(obj, _clearanceMarginM) : obj;
                    if (CollisionGeometry.IntersectsAtPose(body.Geometry, attWorldM, obstacle, _meshBvhCache, _scratch))
                    {
                        LastHit = $"{body.Geometry.Name} vs {obj.Name}";
                        return false;
                    }
                }
            }
        }

        return true;
    }

    public bool SegmentCollisionFree(JointState from, JointState to, CollisionScene scene, double configurationStep)
    {
        if (configurationStep <= 0) configurationStep = 1e-3;
        var n = from.AxisCount;
        var maxDelta = 0.0;
        for (var i = 0; i < n; i++)
            maxDelta = Math.Max(maxDelta, Math.Abs(to.Positions[i] - from.Positions[i]));
        var steps = Math.Max(1, (int)Math.Ceiling(maxDelta / configurationStep));

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

    private bool SelfCollisionFree(CollisionScene scene)
    {
        var worlds = _linkWorld!;
        var n = _links.Count;
        for (var i = 0; i < n; i++)
        {
            var li = _treeLinkOfEntry[i];
            if (li < 0 || li >= _treeMats.Length) continue;
            ComposeWorldInto(worlds[i], _baseM, _treeMats[li], _links[i].Geometry.Pose);
        }

        for (var i = 0; i < n; i++)
        {
            var li = _treeLinkOfEntry[i];
            if (li < 0 || li >= _treeMats.Length) continue;
            var a = _links[i];
            for (var j = i + 1; j < n; j++)
            {
                var lj = _treeLinkOfEntry[j];
                if (lj < 0 || lj >= _treeMats.Length) continue;
                var b = _links[j];
                if (scene.IsPairAllowed(a.LinkName, b.LinkName) ||
                    scene.IsPairAllowed(CollisionBodies.RobotLink(li), CollisionBodies.RobotLink(lj)))
                    continue;

                var rB = _grownRadius is null ? b.EnvelopeRadius : _grownRadius[j];
                if (!CentersMayHit(a.Geometry, worlds[i], a.EnvelopeRadius, b.Geometry, worlds[j], rB))
                    continue;

                var other = _grown is null ? b.Geometry : _grown[j];
                var hit = a.Geometry.Shape == CollisionShape.Mesh && other.Shape == CollisionShape.Mesh
                    ? CollisionGeometry.IntersectsMeshesAtPoses(
                        a.Geometry, worlds[i], other, worlds[j],
                        CollisionMeshCache.GetOrBuild(other), _scratch)
                    : CollisionGeometry.IntersectsAtPose(
                        a.Geometry, worlds[i], AsWorldObstacle(other, worlds[j]), _meshBvhCache, _scratch);
                if (hit)
                {
                    LastHit = $"{a.LinkName} vs {b.LinkName}";
                    return false;
                }
            }
        }
        return true;
    }

    private static bool CentersMayHit(
        CollisionObject a, double[] worldA, double rA,
        CollisionObject b, double[] worldB, double rB)
    {
        if (a.Shape != CollisionShape.Mesh || b.Shape != CollisionShape.Mesh)
            return true;
        CollisionGeometry.MeshEnvelopeCenterLocal(a, out var lax, out var lay, out var laz);
        CollisionGeometry.MeshEnvelopeCenterLocal(b, out var lbx, out var lby, out var lbz);
        Transforms.TransformPointInto(worldA, lax, lay, laz, out var ax, out var ay, out var az);
        Transforms.TransformPointInto(worldB, lbx, lby, lbz, out var bx, out var by, out var bz);
        var dx = ax - bx; var dy = ay - by; var dz = az - bz;
        var rr = rA + rB;
        return dx * dx + dy * dy + dz * dz <= rr * rr;
    }

    private static CollisionObject AsWorldObstacle(CollisionObject local, double[] worldM)
    {
        var f = Transforms.ToFrame(worldM);
        return local.Shape switch
        {
            CollisionShape.Sphere => CollisionObject.Sphere(local.Name, f, local.ExtentX),
            CollisionShape.Box => CollisionObject.Box(local.Name, f, local.ExtentX, local.ExtentY, local.ExtentZ),
            CollisionShape.Capsule => CollisionObject.Capsule(local.Name, f, local.ExtentX, local.ExtentY),
            CollisionShape.Mesh when local.MeshVertices is not null && local.MeshIndices is not null =>
                CollisionObject.Mesh(local.Name, f, local.MeshVertices, local.MeshIndices),
            _ => local
        };
    }

    private bool ToolSceneCollisionFree(JointState state, CollisionScene scene)
    {
        if (_toolEntry is null || scene.Objects.Count == 0 || _tipFk is null || _tipCount <= 0)
            return true;

        var tipQ = TipSlice(state.Positions, _tipCount);
        var toolM = ToolCollisionPlacement.WorldMatrix(
            _tipFk, tipQ, _base, _tool, _toolEntry.Geometry,
            _collision.ToolGeometryInFlangeFrame,
            _collision.ToolGeometryAttachOffset);
        var worldM = CollisionGeometry.ComposeWorldMatrix(toolM, _toolEntry.Geometry.Pose);
        foreach (var obj in scene.Objects)
        {
            if (scene.IsPairAllowed(_toolEntry.Geometry.Name, obj.Name)) continue;
            var obstacle = _clearanceMarginM > 0 ? CollisionGeometry.Grow(obj, _clearanceMarginM) : obj;
            if (!CollisionGeometry.EnvelopeMayHit(_toolEntry.Geometry, worldM, _toolEntry.EnvelopeRadius + _clearanceMarginM, obstacle, _meshBvhCache, _scratch))
                continue;
            if (CollisionGeometry.IntersectsAtPose(_toolEntry.Geometry, worldM, obstacle, _meshBvhCache, _scratch))
            {
                LastHit = $"{_toolEntry.Geometry.Name} vs {obj.Name}";
                return false;
            }
        }
        return true;
    }

    private string? FillDriverQ(IReadOnlyList<double> planQ)
    {
        var homeComplete = _treeDriverHome is not null && _treeDriverHome.Length == _tree.DriverCount;
        for (var di = 0; di < _tree.DriverCount; di++)
        {
            var j = _tree.Joints[_tree.DriverJointIndices[di]];
            var ai = -1;
            if (_planJointNames is not null)
            {
                for (var k = 0; k < _planJointNames.Count; k++)
                {
                    if (string.Equals(_planJointNames[k], j.Name, StringComparison.OrdinalIgnoreCase))
                    {
                        ai = k;
                        break;
                    }
                }
            }

            if (ai < 0 && _planJointNames is null)
                ai = di < planQ.Count ? di : -1;

            if (ai >= 0 && ai < planQ.Count)
                _driverQ[di] = planQ[ai];
            else if (homeComplete)
                _driverQ[di] = _treeDriverHome![di];
            else
                return $"TreeFK collision: no value for driver '{j.Name}'.";
        }
        return null;
    }

    private static IReadOnlyList<double> TipSlice(IReadOnlyList<double> q, int tipCount)
    {
        if (tipCount <= 0 || q.Count == tipCount) return q;
        if (q.Count < tipCount) return q;
        var tip = new double[tipCount];
        for (var i = 0; i < tipCount; i++)
            tip[i] = q[i];
        return tip;
    }

    private void ComposeWorldInto(double[] dest, double[] baseM, double[] linkM, Frame localPose)
    {
        Transforms.MultiplyInto(_linkScratch, baseM, linkM);
        Transforms.MultiplyInto(dest, _linkScratch, Transforms.FromFrame(localPose));
    }
}
