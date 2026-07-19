using System.Diagnostics;
using System.Xml.Linq;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class TreeForwardKinematicsTests
{
    private static string FixturePath(string name) =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", name));

    [Fact]
    public void TwoLink_TreeTipMatchesSerialAfterExtract()
    {
        var path = FixturePath("two_link.urdf");
        var tree = UrdfRobotLoader.LoadTree(path);
        var tip = tree.ExtractSerialTip("base_link", "tip_link");
        var serial = new SerialForwardKinematics(tip.Chain);
        var treeFk = new TreeForwardKinematics(tree);
        var tipIdx = tree.IndexOfLink("tip_link");
        var mats = AllocMats(tree.Links.Count);
        var q = new double[] { 0.3, -0.5 };

        var serialM = serial.ComputeFlangeTransform(q);
        treeFk.ComputeLinkTransformsInto(q, mats);
        AssertMatClose(serialM, mats[tipIdx], 1e-9);
    }

    [Fact]
    public void Mimic_TipMovesWhenDriverChanges()
    {
        var doc = XDocument.Parse("""
            <?xml version="1.0"?>
            <robot name="mimic_smoke">
              <link name="base_link"/>
              <link name="mid_link"/>
              <link name="tip_link"/>
              <joint name="driver" type="revolute">
                <parent link="base_link"/><child link="mid_link"/>
                <origin xyz="0.1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
                <limit lower="-1.5" upper="1.5" velocity="1"/>
              </joint>
              <joint name="follower" type="revolute">
                <parent link="mid_link"/><child link="tip_link"/>
                <origin xyz="0.1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
                <mimic joint="driver" multiplier="2" offset="0"/>
              </joint>
            </robot>
            """);

        var tree = UrdfRobotLoader.LoadTree(doc);
        Assert.Equal(1, tree.DriverCount);
        var fk = new TreeForwardKinematics(tree);
        var tip = tree.IndexOfLink("tip_link");
        var mats = AllocMats(tree.Links.Count);

        fk.ComputeLinkTransformsInto(new[] { 0.0 }, mats);
        var x0 = mats[tip][3];
        var y0 = mats[tip][7];

        fk.ComputeLinkTransformsInto(new[] { 0.4 }, mats);
        var x1 = mats[tip][3];
        var y1 = mats[tip][7];

        var dist = Math.Sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
        Assert.True(dist > 1e-3, $"Expected tip to move under mimic; dist={dist}");
    }

    [Fact]
    public void PrismaticLift_DriverCountAndFkRuns()
    {
        var tree = UrdfRobotLoader.LoadTree(FixturePath("prismatic_lift.urdf"));
        Assert.Equal(2, tree.DriverCount);
        Assert.Contains(tree.Joints, j => j.Type == KinematicJointType.Prismatic);

        var fk = new TreeForwardKinematics(tree);
        var mats = AllocMats(tree.Links.Count);
        fk.ComputeLinkTransformsInto(new[] { 0.0, 0.0 }, mats);
        fk.ComputeLinkTransformsInto(new[] { 0.4, 0.2 }, mats);

        var tip = tree.IndexOfLink("tip_link");
        Assert.True(mats[tip][11] > 0.3);
    }

    [Fact]
    public void Microbench_10k_N7_Under500ms()
    {
        var tree = UrdfRobotLoader.LoadTree(BuildRailArm7());
        Assert.Equal(7, tree.DriverCount);

        var fk = new TreeForwardKinematics(tree);
        var mats = AllocMats(tree.Links.Count);
        var q = new double[] { 0.1, 0.05, -0.1, 0.2, -0.15, 0.1, 0.05 };

        for (var i = 0; i < 100; i++)
            fk.ComputeLinkTransformsInto(q, mats);

        var sw = Stopwatch.StartNew();
        const int iters = 10_000;
        for (var i = 0; i < iters; i++)
            fk.ComputeLinkTransformsInto(q, mats);
        sw.Stop();

        Assert.True(sw.ElapsedMilliseconds < 500,
            $"TreeFK {iters} iters N={tree.DriverCount} took {sw.ElapsedMilliseconds}ms (limit 500ms)");
    }

    private static double[][] AllocMats(int n)
    {
        var mats = new double[n][];
        for (var i = 0; i < n; i++)
            mats[i] = new double[16];
        return mats;
    }

    private static void AssertMatClose(double[] expected, double[] actual, double tol)
    {
        for (var i = 0; i < 16; i++)
            Assert.True(Math.Abs(expected[i] - actual[i]) < tol,
                $"mat[{i}] expected {expected[i]} got {actual[i]}");
    }

    private static XDocument BuildRailArm7()
    {
        var robot = new XElement("robot", new XAttribute("name", "rail_arm7"));
        robot.Add(new XElement("link", new XAttribute("name", "base_link")));
        robot.Add(new XElement("link", new XAttribute("name", "rail_link")));
        for (var i = 1; i <= 5; i++)
            robot.Add(new XElement("link", new XAttribute("name", $"link{i}")));
        robot.Add(new XElement("link", new XAttribute("name", "tool0")));

        robot.Add(J("rail", "prismatic", "base_link", "rail_link", "0 0 0", "0 0 1", 0, 1));
        robot.Add(J("j1", "revolute", "rail_link", "link1", "0 0 0.2", "0 0 1", -3, 3));
        robot.Add(J("j2", "revolute", "link1", "link2", "0.3 0 0", "0 1 0", -3, 3));
        robot.Add(J("j3", "revolute", "link2", "link3", "0.3 0 0", "0 1 0", -3, 3));
        robot.Add(J("j4", "revolute", "link3", "link4", "0.2 0 0", "1 0 0", -3, 3));
        robot.Add(J("j5", "revolute", "link4", "link5", "0 0 0.1", "0 1 0", -3, 3));
        robot.Add(J("j6", "revolute", "link5", "tool0", "0 0 0.08", "0 0 1", -3, 3));
        return new XDocument(robot);
    }

    private static XElement J(string name, string type, string parent, string child, string xyz, string axis, double lo, double hi) =>
        new("joint",
            new XAttribute("name", name),
            new XAttribute("type", type),
            new XElement("parent", new XAttribute("link", parent)),
            new XElement("child", new XAttribute("link", child)),
            new XElement("origin", new XAttribute("xyz", xyz), new XAttribute("rpy", "0 0 0")),
            new XElement("axis", new XAttribute("xyz", axis)),
            new XElement("limit",
                new XAttribute("lower", lo.ToString(System.Globalization.CultureInfo.InvariantCulture)),
                new XAttribute("upper", hi.ToString(System.Globalization.CultureInfo.InvariantCulture)),
                new XAttribute("velocity", "1")));
}
