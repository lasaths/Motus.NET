using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Scene collision via TreeFK — poses every named collision link (tip path + side branches).
/// Plan joint state may be tip-only or tip+branches (see constructor planJointNames).
/// Unmatched tree drivers use treeDriverHome (open gripper, etc.).
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
    private double[]? _segmentQ;
    private JointState? _segmentState;

    private sealed class LinkEntry
    {
        public required CollisionObject Geometry { get; init; }
        public required string LinkName { get; init; }
        public double EnvelopeRadius { get; init; }
    }

    public TreeFkCollisionChecker(
        RobotModel robot,
        KinematicTree tree,
        SerialJointChain? tipChain = null,
        IReadOnlyList<string>? planJointNames = null,
        IReadOnlyList<double>? treeDriverHome = null,
        IReadOnlyList<AttachedBody>? attached = null)
    {
        if (robot.CollisionModel is null || robot.CollisionModel.Links.Count == 0)
            throw new ArgumentException("TreeFkCollisionChecker requires a CollisionModel with links.", nameof(robot));

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
        if (FillDriverQ(state.Positions) is not null)
            return false;

        _treeFk.ComputeLinkTransformsInto(_driverQ, _treeMats);

        foreach (var obj in scene.Objects)
        {
            for (var i = 0; i < _links.Count; i++)
            {
                var entry = _links[i];
                var li = _treeLinkOfEntry[i];
                if (li < 0 || li >= _treeMats.Length) continue;
                if (scene.IsPairAllowed(CollisionBodies.RobotLink(li), obj.Name))
                    continue;
                ComposeWorldInto(_worldScratch, _baseM, _treeMats[li], entry.Geometry.Pose);
                if (!CollisionGeometry.EnvelopeMayHit(entry.Geometry, _worldScratch, entry.EnvelopeRadius, obj, _meshBvhCache, _scratch))
                    continue;
                if (CollisionGeometry.IntersectsAtPose(entry.Geometry, _worldScratch, obj, _meshBvhCache, _scratch))
                    return false;
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
                    if (CollisionGeometry.IntersectsAtPose(body.Geometry, attWorldM, obj, _meshBvhCache, _scratch))
                        return false;
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
            if (!CollisionGeometry.EnvelopeMayHit(_toolEntry.Geometry, worldM, _toolEntry.EnvelopeRadius, obj, _meshBvhCache, _scratch))
                continue;
            if (CollisionGeometry.IntersectsAtPose(_toolEntry.Geometry, worldM, obj, _meshBvhCache, _scratch))
                return false;
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
