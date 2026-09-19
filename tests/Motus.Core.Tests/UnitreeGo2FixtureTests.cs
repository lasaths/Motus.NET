using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>
/// Experimental Unitree Go2 fixture smoke (meshless base-rooted mammal quad).
/// Honest scope: LoadTree + Tree FK + stance pose — NOT insectoid <c>Family=legged</c> Walk.
/// Source: unitree_ros go2_description (BSD-3-Clause).
/// </summary>
public class UnitreeGo2FixtureTests
{
    private static string FixturePath(string name) =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", name));

    private static readonly string[] DriverOrder =
    [
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    ];

    private static readonly string[] FootLinks = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];

    [Fact]
    public void LoadTree_Go2Minimal_HasTwelveRevoluteDrivers()
    {
        var tree = UrdfRobotLoader.LoadTree(FixturePath("unitree_go2/go2_minimal.urdf"));

        Assert.Equal("go2_minimal", tree.Name);
        Assert.Equal("base", tree.Links[tree.RootLinkIndex].Name);
        Assert.Equal(12, tree.DriverCount);
        Assert.DoesNotContain(tree.Joints, j => j.Type == KinematicJointType.Prismatic);
        Assert.All(
            tree.DriverJointIndices.Select(i => tree.Joints[i]),
            j => Assert.Equal(KinematicJointType.Revolute, j.Type));

        Assert.Equal(3, CountDrivers(tree, "FL_"));
        Assert.Equal(3, CountDrivers(tree, "FR_"));
        Assert.Equal(3, CountDrivers(tree, "RL_"));
        Assert.Equal(3, CountDrivers(tree, "RR_"));

        var names = tree.DriverJointIndices.Select(i => tree.Joints[i].Name).ToArray();
        Assert.Equal(DriverOrder, names);
    }

    [Fact]
    public void TreeFk_Go2Minimal_HomeAndStanceMoveFeet()
    {
        var tree = UrdfRobotLoader.LoadTree(FixturePath("unitree_go2/go2_minimal.urdf"));
        var fk = new TreeForwardKinematics(tree);
        var mats = AllocMats(tree.Links.Count);
        var qHome = new double[tree.DriverCount];

        fk.ComputeLinkTransformsInto(qHome, mats);
        var flHome = FootXyz(tree, mats, "FL_foot");
        Assert.False(double.IsNaN(flHome.z) || double.IsInfinity(flHome.z));

        // Approximate standing crouch (radians) — mammal sagittal thigh/calf, hips near 0.
        // Not a claimed IK solve; just a finite stance pose for FK smoke.
        var qStance = StancePose(tree);
        fk.ComputeLinkTransformsInto(qStance, mats);

        var feetZ = FootLinks.Select(n => FootXyz(tree, mats, n).z).ToArray();
        Assert.All(feetZ, z => Assert.True(double.IsFinite(z)));

        // Stance should move FL foot relative to zero-pose stretch.
        var flStance = FootXyz(tree, mats, "FL_foot");
        var dist = Math.Sqrt(
            Math.Pow(flStance.x - flHome.x, 2) +
            Math.Pow(flStance.y - flHome.y, 2) +
            Math.Pow(flStance.z - flHome.z, 2));
        Assert.True(dist > 0.05, $"FL_foot should move under stance; dist={dist}");

        // Feet roughly coplanar in stance (mammal stand smoke — not Walk contact schedule).
        var zMean = feetZ.Average();
        Assert.All(feetZ, z => Assert.InRange(z - zMean, -0.05, 0.05));
    }

    [Fact]
    public void ExtractSerialTip_FrontLeftLeg_ThreeRevolute()
    {
        var tree = UrdfRobotLoader.LoadTree(FixturePath("unitree_go2/go2_minimal.urdf"));
        var tip = tree.ExtractSerialTip("base", "FL_foot");
        Assert.Equal(3, tip.Chain.Joints.Length);
        Assert.Equal(
            new[] { "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint" },
            tip.JointNames);
    }

    [Fact]
    public void Fixture_IsNotInsectoidLeggedFamily()
    {
        // Explicit honesty gate: Go2 fixture must not be confused with QuadSmoke / Family=legged.
        var tree = UrdfRobotLoader.LoadTree(FixturePath("unitree_go2/go2_minimal.urdf"));
        var (robot, _) = ToFullDriverModel(tree);
        Assert.False(Units.IsLegged(robot.Preset));
        Assert.False(Units.IsStewart(robot.Preset));
        Assert.False(Units.IsAerial(robot.Preset));
        Assert.Equal("urdf", robot.Preset.Family);
        Assert.Contains("not insectoid", robot.Preset.Notes!, StringComparison.OrdinalIgnoreCase);
    }

    private static double[] StancePose(KinematicTree tree)
    {
        var q = new double[tree.DriverCount];
        for (var i = 0; i < tree.DriverCount; i++)
        {
            var name = tree.Joints[tree.DriverJointIndices[i]].Name;
            if (name.EndsWith("_hip_joint", StringComparison.Ordinal))
                q[i] = 0.0;
            else if (name.EndsWith("_thigh_joint", StringComparison.Ordinal))
                q[i] = 0.8;
            else if (name.EndsWith("_calf_joint", StringComparison.Ordinal))
                q[i] = -1.5;
        }

        return q;
    }

    private static (double x, double y, double z) FootXyz(KinematicTree tree, double[][] mats, string link)
    {
        var i = tree.IndexOfLink(link);
        return (mats[i][3], mats[i][7], mats[i][11]);
    }

    private static int CountDrivers(KinematicTree tree, string prefix)
    {
        var n = 0;
        for (var i = 0; i < tree.DriverCount; i++)
        {
            var name = tree.Joints[tree.DriverJointIndices[i]].Name;
            if (name.StartsWith(prefix, StringComparison.Ordinal))
                n++;
        }

        return n;
    }

    private static (RobotModel Model, string[] JointNames) ToFullDriverModel(KinematicTree tree)
    {
        var names = new string[tree.DriverCount];
        var limits = new List<JointLimit>(tree.DriverCount);
        for (var i = 0; i < tree.DriverCount; i++)
        {
            var j = tree.Joints[tree.DriverJointIndices[i]];
            names[i] = j.Name;
            var lo = double.IsFinite(j.Lower) ? j.Lower : -Math.PI;
            var hi = double.IsFinite(j.Upper) ? j.Upper : Math.PI;
            var vel = j.Velocity is > 0 and var v ? v : Math.PI;
            limits.Add(new JointLimit(lo, hi, vel, vel * 2));
        }

        var preset = new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = "Unitree Go2 (experimental tree)",
            // Intentionally not Family=legged / quadruped GA — mammal tree FK / stance only.
            Family = "urdf",
            AxisCount = tree.DriverCount,
            JointLimits = limits,
            BaseFrame = BaseFrame.Identity,
            ToolFrame = ToolFrame.Identity,
            Notes = "Experimental Go2 meshless tree — mammal stance/FK, not insectoid Walk.",
        };
        return (new RobotModel(preset, jointNames: names), names);
    }

    private static double[][] AllocMats(int n)
    {
        var mats = new double[n][];
        for (var i = 0; i < n; i++)
            mats[i] = new double[16];
        return mats;
    }
}
