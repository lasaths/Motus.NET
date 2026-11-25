using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Scene collision via TreeFK — poses every named collision link (tip path + side branches).
/// Plan joint state may be tip-only or tip+branches (see constructor planJointNames).
/// Unmatched tree drivers use treeDriverHome (open gripper, etc.).
/// With a <see cref="TreeSelfCollision"/>, links are also tested against each other (off when null).
/// Self pairs of two meshes and scene capsules then go through a conservative broad phase (padded per-triangle
/// world AABBs, capsule axis clipped to the link AABBs); the exact tests decide, so the verdicts do not change.
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
    private readonly double[] _linkScratch = new double[16];
    private readonly double[][] _entryWorld;
    private double[]? _segmentQ;
    private JointState? _segmentState;

    // Self-collision (null / empty when off). Pairs are (robot entry, grown obstacle entry), both directions.
    private readonly (int Robot, int Obstacle)[] _selfPairs = Array.Empty<(int, int)>();
    private readonly CollisionObject[]? _grown;
    private readonly CollisionObject?[]? _posedGrown; // per query, exact path only
    // Broad phase per entry: world vertices and per-triangle world AABBs [min xyz, max xyz] + the link AABB (last 6),
    // for the link (_rw/_ra) and its grown copy (_gw/_ga, posed by _gPose like the exact test).
    private readonly double[][]? _rw, _ra, _gw, _ga, _gPoseM;
    private readonly Frame[]? _gPose;
    // Broad-phase pad: far above the rounding gap between these world vertices and those inside the exact test.
    private const double Pad = 1e-6;

    /// <summary>Test hook: no broad phase; self pairs go through the same exact path as scene objects.</summary>
    internal bool ExactOnly { get; init; }

    /// <summary>The first failing pair ("A vs B") or the driver error after <see cref="IsCollisionFree"/> returns false.</summary>
    public string? LastHit { get; private set; }

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
        IReadOnlyList<AttachedBody>? attached = null,
        TreeSelfCollision? selfCollision = null)
    {
        if (robot.CollisionModel is null || robot.CollisionModel.Links.Count == 0)
            throw new ArgumentException("TreeFkCollisionChecker requires a CollisionModel with links.", nameof(robot));
        if (selfCollision is not null && (!double.IsFinite(selfCollision.ClearanceM) || selfCollision.ClearanceM < 0))
            throw new ArgumentOutOfRangeException(nameof(selfCollision), selfCollision.ClearanceM, "Self-collision clearance must be a finite value >= 0.");

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

        _entryWorld = _links.Select(_ => new double[16]).ToArray();
        if (selfCollision is null)
            return;

        var allowed = new HashSet<(string, string)>();
        foreach (var (a, b) in selfCollision.AllowedPairs)
        {
            allowed.Add((a, b));
            allowed.Add((b, a));
        }
        _grown = _links.Select(e => CollisionGeometry.Grow(e.Geometry, selfCollision.ClearanceM)).ToArray();
        _selfPairs = (from o in Enumerable.Range(0, _links.Count)
                      from r in Enumerable.Range(0, _links.Count)
                      where _treeLinkOfEntry[o] != _treeLinkOfEntry[r] && !allowed.Contains((_links[r].LinkName, _links[o].LinkName))
                      select (r, o)).ToArray();
        _rw = _links.Select(e => new double[(e.Geometry.MeshVertices?.Count ?? 0) * 3]).ToArray();
        _gw = _grown.Select(g => new double[(g.MeshVertices?.Count ?? 0) * 3]).ToArray();
        _ra = _links.Select(e => new double[(e.Geometry.MeshIndices?.Count ?? 0) / 3 * 6 + 6]).ToArray();
        _ga = _grown.Select(g => new double[(g.MeshIndices?.Count ?? 0) / 3 * 6 + 6]).ToArray();
        _posedGrown = new CollisionObject?[_links.Count];
        _gPose = new Frame[_links.Count];
        _gPoseM = _links.Select(_ => new double[16]).ToArray();
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
        if (FillDriverQ(state.Positions) is { } error)
        {
            LastHit = error;
            return false;
        }

        _treeFk.ComputeLinkTransformsInto(_driverQ, _treeMats);
        for (var i = 0; i < _links.Count; i++)
            ComposeWorldInto(_entryWorld[i], _baseM, _treeMats[_treeLinkOfEntry[i]], _links[i].Geometry.Pose);

        var broadPhase = _grown is not null && !ExactOnly;
        if (broadPhase)
            UpdateBoxes();
        if (_grown is not null && !SelfCollisionFree(scene, broadPhase))
            return false;

        foreach (var obj in scene.Objects)
        {
            var target = broadPhase && obj.Shape == CollisionShape.Capsule ? Clip(obj) : obj;
            if (target is null)
                continue;
            for (var i = 0; i < _links.Count; i++)
            {
                if (scene.IsPairAllowed(CollisionBodies.RobotLink(_treeLinkOfEntry[i]), obj.Name))
                    continue;
                if (LinkHits(i, target))
                {
                    LastHit = $"{_links[i].LinkName} vs {obj.Name}";
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
                    if (CollisionGeometry.IntersectsAtPose(body.Geometry, attWorldM, obj, _meshBvhCache, _scratch))
                    {
                        LastHit = $"{body.Geometry.Name} vs {obj.Name}";
                        return false;
                    }
                }
            }
        }

        return true;
    }

    // Exact test of link entry i at its pose against a world obstacle (envelope prefilter first).
    private bool LinkHits(int i, CollisionObject obstacle)
    {
        var entry = _links[i];
        return CollisionGeometry.EnvelopeMayHit(entry.Geometry, _entryWorld[i], entry.EnvelopeRadius, obstacle, _meshBvhCache, _scratch)
            && CollisionGeometry.IntersectsAtPose(entry.Geometry, _entryWorld[i], obstacle, _meshBvhCache, _scratch);
    }

    private bool SelfCollisionFree(CollisionScene scene, bool broadPhase)
    {
        var posed = _posedGrown!;
        Array.Clear(posed);
        foreach (var (r, o) in _selfPairs)
        {
            if (scene.IsPairAllowed(_links[r].LinkName, _links[o].LinkName) ||
                scene.IsPairAllowed(CollisionBodies.RobotLink(_treeLinkOfEntry[r]), CollisionBodies.RobotLink(_treeLinkOfEntry[o])))
                continue;
            var hit = broadPhase && _links[r].Geometry.Shape == CollisionShape.Mesh && _grown![o].Shape == CollisionShape.Mesh
                ? MeshPairHits(r, o)
                : LinkHits(r, posed[o] ??= CollisionGeometry.Transform(_grown![o], Transforms.Multiply(_baseM, _treeMats[_treeLinkOfEntry[o]])));
            if (hit)
            {
                LastHit = $"{_links[r].LinkName} vs {_links[o].LinkName}";
                return false;
            }
        }
        return true;
    }

    // World vertices and padded-AABB inputs for every entry: meshes from their vertices, primitives from the envelope.
    private void UpdateBoxes()
    {
        for (var k = 0; k < _links.Count; k++)
        {
            var g = _links[k].Geometry;
            if (g.Shape == CollisionShape.Mesh)
            {
                WorldBoxes(_entryWorld[k], g.MeshVertices!, g.MeshIndices!, _rw![k], _ra![k]);
                _gPose![k] = Transforms.ToFrame(_entryWorld[k]);
                Transforms.FromFrameInto(_gPoseM![k], _gPose[k]);
                WorldBoxes(_gPoseM[k], _grown![k].MeshVertices!, _grown[k].MeshIndices!, _gw![k], _ga![k]);
                continue;
            }
            double[] m = _entryWorld[k], box = _ra![k];
            var r = _links[k].EnvelopeRadius;
            for (var d = 0; d < 3; d++)
            {
                box[d] = m[4 * d + 3] - r;
                box[3 + d] = m[4 * d + 3] + r;
            }
        }
    }

    /// <summary>
    /// Mesh link r against the grown mesh of link o. Same inputs to <see cref="TriangleCollision.TriangleTriangleOverlap"/>
    /// as the exact mesh-mesh test (robot triangle in world, obstacle triangle local + its pose); triangle pairs whose
    /// padded world AABBs do not overlap are skipped.
    /// </summary>
    private bool MeshPairHits(int r, int o)
    {
        double[] ra = _ra![r], ga = _ga![o], rw = _rw![r];
        int rl = ra.Length - 6, gl = ga.Length - 6;
        if (!Overlap(ra, rl, ga, gl))
            return false;
        var ri = _links[r].Geometry.MeshIndices!;
        var (gv, gi) = (_grown![o].MeshVertices!, _grown[o].MeshIndices!);
        for (var a = 0; a < rl; a += 6)
        {
            if (!Overlap(ra, a, ga, gl)) continue;
            var t = a / 2;
            Frame a0 = P(rw, ri[t]), a1 = P(rw, ri[t + 1]), a2 = P(rw, ri[t + 2]);
            for (var b = 0; b < gl; b += 6)
            {
                if (!Overlap(ra, a, ga, b)) continue;
                var u = b / 2;
                if (TriangleCollision.TriangleTriangleOverlap(a0, a1, a2, Frame.Identity,
                        V(gv[gi[u]]), V(gv[gi[u + 1]]), V(gv[gi[u + 2]]), _gPose![o]))
                    return true;
            }
        }
        return false;

        static Frame P(double[] w, int i) => new(w[3 * i], w[3 * i + 1], w[3 * i + 2]);
        static Frame V(double[] v) => new(v[0], v[1], v[2]);
    }

    private static bool Overlap(double[] x, int i, double[] y, int j) =>
        x[i] <= y[j + 3] + Pad && y[j] <= x[i + 3] + Pad &&
        x[i + 1] <= y[j + 4] + Pad && y[j + 1] <= x[i + 4] + Pad &&
        x[i + 2] <= y[j + 5] + Pad && y[j + 2] <= x[i + 5] + Pad;

    // World vertices (as the exact test transforms them) and per-triangle world AABBs; the last 6 hold the union.
    private static void WorldBoxes(double[] m, IReadOnlyList<double[]> v, IReadOnlyList<int> idx, double[] w, double[] box)
    {
        for (var i = 0; i < v.Count; i++)
        {
            Transforms.TransformPointInto(m, v[i][0], v[i][1], v[i][2], out var x, out var y, out var z);
            w[3 * i] = x; w[3 * i + 1] = y; w[3 * i + 2] = z;
        }
        var all = box.Length - 6;
        for (var d = 0; d < 3; d++) { box[all + d] = double.PositiveInfinity; box[all + 3 + d] = double.NegativeInfinity; }
        for (var t = 0; t < all; t += 6)
        {
            var i0 = 3 * idx[t / 2]; var i1 = 3 * idx[t / 2 + 1]; var i2 = 3 * idx[t / 2 + 2];
            for (var d = 0; d < 3; d++)
            {
                double lo = Math.Min(w[i0 + d], Math.Min(w[i1 + d], w[i2 + d])), hi = Math.Max(w[i0 + d], Math.Max(w[i1 + d], w[i2 + d]));
                box[t + d] = lo; box[t + 3 + d] = hi;
                box[all + d] = Math.Min(box[all + d], lo); box[all + 3 + d] = Math.Max(box[all + 3 + d], hi);
            }
        }
    }

    /// <summary>
    /// Capsule <paramref name="c"/> cut to the part of its axis that can come within its radius of a link AABB, or null
    /// when no part can. Every axis point near a link stays, so the exact capsule test against the links gives the same
    /// answer with a shorter axis (at an exact touch, distance 0 to the last bit, the rounding of the new end points
    /// decides). Only for the link loop: the tool and attached bodies use the full capsule.
    /// </summary>
    private CollisionObject? Clip(CollisionObject c)
    {
        var m = Transforms.FromFrame(c.Pose);
        Transforms.TransformPointInto(m, 0, 0, -c.ExtentY, out var px, out var py, out var pz);
        Transforms.TransformPointInto(m, 0, 0, c.ExtentY, out var qx, out var qy, out var qz);
        double[] p = [px, py, pz], d = [qx - px, qy - py, qz - pz];
        double lo = 1, hi = 0, r = c.ExtentX + Pad;
        foreach (var box in _ra!)
        {
            var b = box.Length - 6;
            double t0 = 0, t1 = 1;
            for (var k = 0; k < 3 && t0 <= t1; k++)
            {
                double min = box[b + k] - r, max = box[b + 3 + k] + r;
                if (Math.Abs(d[k]) < 1e-15) { if (p[k] < min || p[k] > max) t1 = -1; continue; }
                double a = (min - p[k]) / d[k], e = (max - p[k]) / d[k];
                if (a > e) (a, e) = (e, a);
                t0 = Math.Max(t0, a); t1 = Math.Min(t1, e);
            }
            if (t0 <= t1) { lo = Math.Min(lo, t0); hi = Math.Max(hi, t1); }
        }
        if (lo > hi) return null;
        if (lo <= 0 && hi >= 1) return c;
        var mid = (lo + hi) / 2;
        return CollisionObject.Capsule(c.Name, new Frame(px + d[0] * mid, py + d[1] * mid, pz + d[2] * mid,
            c.Pose.Qw, c.Pose.Qx, c.Pose.Qy, c.Pose.Qz), c.ExtentX, c.ExtentY * (hi - lo));
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
