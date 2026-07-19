using Motus.Geometry;

namespace Motus.Core.Tests;

public class SerialKinematicTreesTests
{
    [Fact]
    public void FromLengths_SixRevolute_DriverCountAndTipFk()
    {
        var lengths = new[] { 0.15, 0.35, 0.30, 0.20, 0.15, 0.10 };
        var tree = SerialKinematicTrees.FromLengths(lengths);
        Assert.Equal(6, tree.DriverCount);
        Assert.Equal(0, tree.RootLinkIndex);

        var tip = tree.ExtractSerialTip("base_link", "tool0");
        Assert.Equal(6, tip.Chain.Joints.Length);

        var serial = new SerialForwardKinematics(tip.Chain);
        var treeFk = new TreeForwardKinematics(tree);
        var mats = Alloc(tree.Links.Count);
        var q = new double[] { 0.1, -0.2, 0.3, -0.4, 0.2, 0.1 };
        var flange = serial.ComputeFlangeTransform(q);
        treeFk.ComputeLinkTransformsInto(q, mats);
        AssertMatClose(flange, mats[tree.IndexOfLink("tool0")], 1e-9);
    }

    [Fact]
    public void FromLengths_Rail_FirstPrismatic()
    {
        var tree = SerialKinematicTrees.FromLengths(new[] { 1.0, 0.3, 0.3, 0.2, 0.15, 0.1, 0.08 }, rail: true);
        Assert.Equal(7, tree.DriverCount);
        Assert.Equal(KinematicJointType.Prismatic, tree.Joints[0].Type);
        Assert.Equal(KinematicJointType.Revolute, tree.Joints[1].Type);

        var tip = tree.ExtractSerialTip("base_link", "tool0");
        Assert.Equal(JointMotionType.Prismatic, tip.Chain.Joints[0].Motion);
    }

    [Fact]
    public void ReachSampling_CapsAtMax()
    {
        var tree = SerialKinematicTrees.FromLengths(new[] { 0.2, 0.3, 0.25 });
        var fk = new TreeForwardKinematics(tree);
        var lower = new double[tree.DriverCount];
        var upper = new double[tree.DriverCount];
        for (var i = 0; i < tree.DriverCount; i++)
        {
            var j = tree.Joints[tree.DriverJointIndices[i]];
            lower[i] = j.Lower;
            upper[i] = j.Upper;
        }

        var xyz = new double[32 * 3];
        Assert.Equal(32, ReachSampling.FillTcpPointsInto(fk, tree.IndexOfLink("tool0"), lower, upper, xyz, 32));
    }

    private static double[][] Alloc(int n)
    {
        var m = new double[n][];
        for (var i = 0; i < n; i++) m[i] = new double[16];
        return m;
    }

    private static void AssertMatClose(double[] a, double[] b, double tol)
    {
        for (var i = 0; i < 16; i++)
            Assert.True(Math.Abs(a[i] - b[i]) < tol, $"mat[{i}] {a[i]} vs {b[i]}");
    }
}
