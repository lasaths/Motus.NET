using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>
/// Load / plan smoke for meshless URDFs selected from awesome-robot-descriptions
/// (see internal plan robot-test-expansion). Not inbuilt JSON presets.
/// </summary>
public class CatalogRobotSmokeTests
{
    private static string FixturePath(string relative) =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", relative));

    [Fact]
    public void UrdfLoad_Ur10eRobotiqMinimal_TipHasSixAxes()
    {
        var robot = UrdfRobotLoader.Load(FixturePath("ur10e_robotiq/ur10e_robotiq_minimal.urdf"), new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tool0",
            ModelName = "UR10e"
        });

        Assert.Equal(6, robot.Preset.AxisCount);
        Assert.Equal(6, robot.JointNames.Count);
        Assert.Contains("shoulder_pan_joint", robot.JointNames);
    }

    [Fact]
    public void UrdfLoad_PandaMinimal_TipHasSevenAxes()
    {
        var robot = UrdfRobotLoader.Load(FixturePath("panda/panda_minimal.urdf"), new UrdfLoadOptions
        {
            BaseLink = "panda_link0",
            TipLink = "panda_hand_tcp",
            ModelName = "Panda"
        });

        Assert.Equal(7, robot.Preset.AxisCount);
        Assert.Equal(7, robot.JointNames.Count);
        Assert.Equal("urdf", robot.Preset.Family, ignoreCase: true);
        Assert.Contains("panda_joint1", robot.JointNames);
        Assert.DoesNotContain(robot.JointNames, n => n.Contains("finger", StringComparison.OrdinalIgnoreCase));
    }

    [Fact]
    public void LoadTree_PandaMinimal_FingerMimicMoves()
    {
        var tree = UrdfRobotLoader.LoadTree(FixturePath("panda/panda_minimal.urdf"));
        // 7 arm revolute + 1 finger prismatic driver (second finger is mimic)
        Assert.Equal(8, tree.DriverCount);
        Assert.Contains(tree.Joints, j => j.Mimic is not null &&
            j.Name.Contains("finger_joint2", StringComparison.OrdinalIgnoreCase));

        var fk = new TreeForwardKinematics(tree);
        var mats = new double[tree.Links.Count][];
        for (var i = 0; i < mats.Length; i++) mats[i] = new double[16];

        var qOpen = new double[tree.DriverCount];
        var qClosed = (double[])qOpen.Clone();
        for (var i = 0; i < tree.DriverCount; i++)
        {
            var name = tree.Joints[tree.DriverJointIndices[i]].Name;
            if (name.Contains("finger_joint1", StringComparison.OrdinalIgnoreCase))
                qClosed[i] = 0.04;
        }

        var left = tree.IndexOfLink("panda_leftfinger");
        var right = tree.IndexOfLink("panda_rightfinger");
        static double Gap(double[] a, double[] b) => Math.Sqrt(
            Math.Pow(a[3] - b[3], 2) + Math.Pow(a[7] - b[7], 2) + Math.Pow(a[11] - b[11], 2));

        fk.ComputeLinkTransformsInto(qOpen, mats);
        var openGap = Gap(mats[left], mats[right]);
        fk.ComputeLinkTransformsInto(qClosed, mats);
        var closedGap = Gap(mats[left], mats[right]);
        Assert.True(closedGap > openGap + 0.02,
            $"mimic fingers should separate under joint1 drive; open={openGap} closed={closedGap}");
    }

    [Fact]
    public void PandaMinimal_FkIkRoundTrip_AndJointLinear()
    {
        var urdf = UrdfRobotLoader.Load(FixturePath("panda/panda_minimal.urdf"), new UrdfLoadOptions
        {
            BaseLink = "panda_link0",
            TipLink = "panda_hand_tcp",
            ModelName = "Panda"
        });
        var model = urdf.ToModel();
        var fk = KinematicsResolver.CreateFkSolver(urdf.Preset, urdf.Chain);
        var ik = KinematicsResolver.CreateInverseKinematics(urdf.Preset, urdf.Chain);

        // Mid-range seed (joint4 limits are negative-only)
        var seed = new JointState(new[] { 0.1, -0.4, 0.2, -1.5, 0.1, 1.2, 0.3 });
        var pose = fk.ComputeTcp(seed, urdf.Preset.BaseFrame, urdf.Preset.ToolFrame);
        Assert.True(ik.TrySolve(pose, seed, out var solved), "Panda numerical IK should converge near seed");

        var check = fk.ComputeTcp(solved, urdf.Preset.BaseFrame, urdf.Preset.ToolFrame);
        var posErr = Math.Sqrt(
            Math.Pow(check.Tcp.X - pose.Tcp.X, 2) +
            Math.Pow(check.Tcp.Y - pose.Tcp.Y, 2) +
            Math.Pow(check.Tcp.Z - pose.Tcp.Z, 2));
        Assert.True(posErr < 0.005, $"Round-trip error {posErr:F4}m");

        var start = new JointState(new double[7]);
        // joint4 lower=-3.07 upper=-0.07 — zero is out of range; use feasible home-ish
        start = new JointState(new[] { 0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0 });
        var goal = new JointState(new[] { 0.2, -0.3, 0.1, -1.2, 0.1, 1.4, 0.2 });
        var plan = new JointLinearPlanner().Plan(new PlanningRequest(model, start, goal));
        Assert.True(plan.Success, string.Join("; ", plan.Errors));
        Assert.True(new TrajectoryValidator().Validate(plan.Trajectory!).IsValid);
    }

    [Fact]
    public void Ur10eRobotiqMinimal_TipJointLinear()
    {
        var urdf = UrdfRobotLoader.Load(FixturePath("ur10e_robotiq/ur10e_robotiq_minimal.urdf"), new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tool0",
            ModelName = "UR10e"
        });
        var model = urdf.ToModel();
        var start = new JointState(new double[6]);
        var goal = new JointState(new[] { 0.1, -0.2, 0.3, -0.1, 0.2, 0.0 });
        var plan = new JointLinearPlanner().Plan(new PlanningRequest(model, start, goal));
        Assert.True(plan.Success, string.Join("; ", plan.Errors));
        Assert.True(new TrajectoryValidator().Validate(plan.Trajectory!).IsValid);
    }
}
