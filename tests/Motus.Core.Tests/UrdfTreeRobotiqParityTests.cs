using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>Full-tree LoadTree + mimic on sibling Motus.Grasshopper bundled UR10e+Robotiq.</summary>
public class UrdfTreeRobotiqParityTests
{
    private static string? FindBundledUr10eRobotiq()
    {
        // tests/Motus.Core.Tests/bin/Release/net8.0 → repo root → sibling Grasshopper
        var testBin = AppContext.BaseDirectory;
        var motusNet = Path.GetFullPath(Path.Combine(testBin, "..", "..", "..", "..", ".."));
        var sibling = Path.GetFullPath(Path.Combine(motusNet, "..", "Motus.Grasshopper",
            "resources", "robots", "ur10e_robotiq", "ur10e_robotiq.urdf"));
        if (File.Exists(sibling)) return sibling;

        // walk up as fallback
        var dir = testBin;
        for (var i = 0; i < 14 && dir is not null; i++)
        {
            foreach (var rel in new[]
                     {
                         Path.Combine("resources", "robots", "ur10e_robotiq", "ur10e_robotiq.urdf"),
                         Path.Combine("Motus.Grasshopper", "resources", "robots", "ur10e_robotiq", "ur10e_robotiq.urdf"),
                     })
            {
                var c = Path.GetFullPath(Path.Combine(dir, rel));
                if (File.Exists(c)) return c;
            }
            dir = Directory.GetParent(dir)?.FullName;
        }
        return null;
    }

    [Fact]
    public void LoadTree_BundledRobotiq_MimicDriverAndFingerMoves()
    {
        var path = FindBundledUr10eRobotiq();
        Assert.True(path is not null, "Expected sibling Motus.Grasshopper resources/robots/ur10e_robotiq/ur10e_robotiq.urdf");

        var tree = UrdfRobotLoader.LoadTree(path!);
        Assert.True(tree.DriverCount >= 7, $"expected arm+knuckle drivers, got {tree.DriverCount}");

        var knuckleDrivers = 0;
        for (var i = 0; i < tree.DriverCount; i++)
        {
            var j = tree.Joints[tree.DriverJointIndices[i]];
            if (IsLeftKnuckleDriver(j.Name))
                knuckleDrivers++;
        }
        Assert.Equal(1, knuckleDrivers);
        Assert.True(tree.Joints.Count(j => j.Mimic is not null) >= 4);

        var tip = tree.ExtractSerialTip("base_link", "tool0");
        Assert.Equal(6, tip.Chain.Joints.Length);

        var fk = new TreeForwardKinematics(tree);
        var mats = new double[tree.Links.Count][];
        for (var i = 0; i < mats.Length; i++) mats[i] = new double[16];

        var qOpen = new double[tree.DriverCount];
        var qClosed = (double[])qOpen.Clone();
        for (var i = 0; i < tree.DriverCount; i++)
        {
            if (IsLeftKnuckleDriver(tree.Joints[tree.DriverJointIndices[i]].Name))
                qClosed[i] = 0.8;
        }

        var leftTip = tree.IndexOfLink("robotiq_left_finger_tip");
        fk.ComputeLinkTransformsInto(qOpen, mats);
        var x0 = mats[leftTip][3];
        var y0 = mats[leftTip][7];
        fk.ComputeLinkTransformsInto(qClosed, mats);
        var dist = Math.Sqrt(
            Math.Pow(mats[leftTip][3] - x0, 2) + Math.Pow(mats[leftTip][7] - y0, 2));
        Assert.True(dist > 1e-3, $"finger tip should move under knuckle drive; dist={dist}");

        var serial = new SerialForwardKinematics(tip.Chain);
        var armQ = new double[6];
        var serialFlange = serial.ComputeFlangeTransform(armQ);
        var expectedTool0 = tip.TipToolOffset is { } tipOff
            ? Transforms.Multiply(serialFlange, Transforms.FromFrame(tipOff))
            : serialFlange;

        var treeQ = new double[tree.DriverCount];
        for (var i = 0; i < tree.DriverCount; i++)
        {
            var j = tree.Joints[tree.DriverJointIndices[i]];
            for (var a = 0; a < tip.JointNames.Count; a++)
            {
                if (string.Equals(tip.JointNames[a], j.Name, StringComparison.OrdinalIgnoreCase))
                {
                    treeQ[i] = armQ[a];
                    break;
                }
            }
        }
        fk.ComputeLinkTransformsInto(treeQ, mats);
        var tool0 = mats[tree.IndexOfLink("tool0")];
        for (var i = 0; i < 16; i++)
            Assert.True(Math.Abs(expectedTool0[i] - tool0[i]) < 1e-6, $"tool0 mat[{i}]");
    }

    private static bool IsLeftKnuckleDriver(string name) =>
        name.Contains("robotiq_left_knuckle", StringComparison.OrdinalIgnoreCase)
        && !name.Contains("finger", StringComparison.OrdinalIgnoreCase)
        && !name.Contains("inner", StringComparison.OrdinalIgnoreCase);
}
