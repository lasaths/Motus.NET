namespace Motus.Geometry;

/// <summary>
/// Tree FK: poses every link from driver q.
/// <see cref="ComputeLinkTransformsInto"/> writes one 4×4 (row-major) mat per link;
/// index order matches <see cref="KinematicTree.Links"/>.
/// Mimic joints expand into a pre-sized reused buffer (no per-call alloc of q or mats).
/// </summary>
public sealed class TreeForwardKinematics
{
    private readonly KinematicTree _tree;
    private readonly int[] _parentJointOfLink; // -1 for root
    private readonly int[] _topo; // link indices, parents before children
    private readonly double[] _jointQ;
    private readonly double[] _origin = new double[16];
    private readonly double[] _motion = new double[16];
    private readonly double[] _local = new double[16];

    public TreeForwardKinematics(KinematicTree tree)
    {
        _tree = tree;
        _jointQ = new double[tree.Joints.Count];
        _parentJointOfLink = new int[tree.Links.Count];
        for (var i = 0; i < _parentJointOfLink.Length; i++)
            _parentJointOfLink[i] = -1;
        for (var ji = 0; ji < tree.Joints.Count; ji++)
            _parentJointOfLink[tree.Joints[ji].ChildLinkIndex] = ji;
        _topo = BuildTopo(tree, _parentJointOfLink);
    }

    public KinematicTree Tree => _tree;
    public int DriverCount => _tree.DriverCount;
    public int LinkCount => _tree.Links.Count;

    /// <summary>
    /// Writes world transforms for every link into <paramref name="mats"/> (caller-owned).
    /// Requires <c>mats.Length &gt;= Links.Count</c> and each <c>mats[i].Length &gt;= 16</c>.
    /// <paramref name="driverQ"/> length must equal <see cref="DriverCount"/>.
    /// </summary>
    public void ComputeLinkTransformsInto(IReadOnlyList<double> driverQ, double[][] mats)
    {
        if (driverQ.Count != _tree.DriverCount)
            throw new ArgumentException($"Expected {_tree.DriverCount} driver joints, got {driverQ.Count}.");
        if (mats.Length < _tree.Links.Count)
            throw new ArgumentException($"Expected at least {_tree.Links.Count} matrix slots, got {mats.Length}.");

        ExpandJointQ(driverQ);

        for (var t = 0; t < _topo.Length; t++)
        {
            var li = _topo[t];
            var dest = mats[li] ?? throw new ArgumentException($"mats[{li}] is null.");
            if (dest.Length < 16)
                throw new ArgumentException($"mats[{li}] must have length >= 16.");

            var pji = _parentJointOfLink[li];
            if (pji < 0)
            {
                Transforms.IdentityInto(dest);
                continue;
            }

            var j = _tree.Joints[pji];
            WriteOriginInto(_origin, j);
            WriteMotionInto(_motion, j, _jointQ[pji]);
            Transforms.MultiplyInto(_local, _origin, _motion);
            Transforms.MultiplyInto(dest, mats[j.ParentLinkIndex], _local);
        }
    }

    /// <summary>TCP helper: copies tip link translation after a full tree FK into xyz.</summary>
    public void ComputeTipTranslationInto(IReadOnlyList<double> driverQ, int tipLinkIndex, double[][] mats, out double x, out double y, out double z)
    {
        ComputeLinkTransformsInto(driverQ, mats);
        var m = mats[tipLinkIndex];
        x = m[3]; y = m[7]; z = m[11];
    }

    private void ExpandJointQ(IReadOnlyList<double> driverQ)
    {
        var joints = _tree.Joints;
        for (var i = 0; i < joints.Count; i++)
        {
            var j = joints[i];
            if (j.Type == KinematicJointType.Fixed)
            {
                _jointQ[i] = 0;
                continue;
            }

            if (j.Mimic is { } mimic)
            {
                // ponytail: one-level mimic; nested mimic resolved via already-filled driver slot only
                var src = joints[mimic.JointIndex];
                double srcQ;
                if (src.DriverIndex >= 0)
                    srcQ = driverQ[src.DriverIndex];
                else if (src.Mimic is { } nested)
                    srcQ = ResolveMimic(nested, driverQ, 0);
                else
                    srcQ = 0;
                _jointQ[i] = mimic.Multiplier * srcQ + mimic.Offset;
                continue;
            }

            if (j.DriverIndex < 0)
                throw new InvalidOperationException($"Actuated joint '{j.Name}' has no driver index.");
            _jointQ[i] = driverQ[j.DriverIndex];
        }
    }

    private double ResolveMimic(KinematicMimic mimic, IReadOnlyList<double> driverQ, int depth)
    {
        if (depth > 8)
            throw new InvalidOperationException("Mimic chain exceeded depth limit.");
        var src = _tree.Joints[mimic.JointIndex];
        double srcQ;
        if (src.DriverIndex >= 0)
            srcQ = driverQ[src.DriverIndex];
        else if (src.Mimic is { } nested)
            srcQ = ResolveMimic(nested, driverQ, depth + 1);
        else
            srcQ = 0;
        return mimic.Multiplier * srcQ + mimic.Offset;
    }

    private static int[] BuildTopo(KinematicTree tree, int[] parentJointOfLink)
    {
        var n = tree.Links.Count;
        var childCount = new int[n];
        for (var i = 0; i < tree.Joints.Count; i++)
            childCount[tree.Joints[i].ParentLinkIndex]++;

        var children = new int[n][];
        var fill = new int[n];
        for (var i = 0; i < n; i++)
            children[i] = childCount[i] == 0 ? Array.Empty<int>() : new int[childCount[i]];
        for (var i = 0; i < tree.Joints.Count; i++)
        {
            var j = tree.Joints[i];
            children[j.ParentLinkIndex][fill[j.ParentLinkIndex]++] = j.ChildLinkIndex;
        }

        var topo = new int[n];
        var stack = new int[n];
        var sp = 0;
        var outi = 0;
        stack[sp++] = tree.RootLinkIndex;
        while (sp > 0)
        {
            var li = stack[--sp];
            topo[outi++] = li;
            var kids = children[li];
            for (var k = kids.Length - 1; k >= 0; k--)
                stack[sp++] = kids[k];
        }

        if (outi != n)
            throw new InvalidOperationException("Kinematic tree is not a single connected tree from root.");
        return topo;
    }

    private static void WriteOriginInto(double[] m, KinematicJoint j) =>
        Transforms.FromRpyInto(m, j.OriginX, j.OriginY, j.OriginZ, j.Roll, j.Pitch, j.Yaw);

    private static void WriteMotionInto(double[] m, KinematicJoint j, double q)
    {
        if (j.Type == KinematicJointType.Fixed)
        {
            Transforms.IdentityInto(m);
            return;
        }

        if (j.Type == KinematicJointType.Prismatic)
            Transforms.FromPrismaticInto(m, j.AxisX, j.AxisY, j.AxisZ, q);
        else
            Transforms.FromAxisAngleInto(m, j.AxisX, j.AxisY, j.AxisZ, q);
    }
}
