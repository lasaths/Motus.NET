using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class CartesianGoalSolverTests
{
    private static string ResourcesRoot =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

    [Fact]
    public void When_FkGoalSeededWithSourceConfig_Then_ReachSucceeds()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var fk = new DhForwardKinematics(preset);
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var goalTcp = fk.ComputeTcp(goalJ, preset.BaseFrame, preset.ToolFrame);

        var result = new CartesianGoalSolver().TryReach(
            robot,
            goalTcp,
            CartesianGoalSolver.EnumerateDefaultSeeds(goalJ, robot));

        Assert.True(result.Success);
        Assert.NotNull(result.Solution);
    }

    [Fact]
    public void When_GoalFiveMetersAway_Then_ReachFails()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var fk = new DhForwardKinematics(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var startTcp = fk.ComputeTcp(home, preset.BaseFrame, preset.ToolFrame);
        var farGoal = new CartesianPose(new Frame(
            startTcp.Tcp.X + 5.0, startTcp.Tcp.Y, startTcp.Tcp.Z,
            startTcp.Tcp.Qw, startTcp.Tcp.Qx, startTcp.Tcp.Qy, startTcp.Tcp.Qz));

        var result = new CartesianGoalSolver().TryReach(
            robot,
            farGoal,
            CartesianGoalSolver.EnumerateDefaultSeeds(home, robot));

        Assert.False(result.Success);
        Assert.True(result.Errors.Count > 0);
    }

    [Fact]
    public void When_HomeSeededToExample01Tcp_Then_ReachSucceeds()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var fk = new DhForwardKinematics(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var goalTcp = fk.ComputeTcp(goalJ, preset.BaseFrame, preset.ToolFrame);

        var result = new CartesianGoalSolver().TryReach(
            robot,
            goalTcp,
            CartesianGoalSolver.EnumerateDefaultSeeds(home, robot));

        Assert.True(result.Success, string.Join("; ", result.Errors));
    }


    [Fact]
    public void Ur10eRobotiq_JointLinearHomeToExample02Goal_Succeeds()
    {
        var preset = PresetLoader.LoadByModelName("UR10e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var joint = new JointLinearPlanner().Plan(new PlanningRequest(robot, home, goalJ));
        Assert.True(joint.Success, string.Join("; ", joint.Errors));
    }

    [Fact]
    public void Ur10eRobotiq_IkFromHomeToTinyTcpDelta()
    {
        var preset = PresetLoader.LoadByModelName("UR10e", ResourcesRoot);
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var ik = KinematicsResolver.CreateInverseKinematics(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var homeTcp = fk.ComputeTcp(home, preset.BaseFrame, preset.ToolFrame);
        var nudged = new CartesianPose(new Frame(
            homeTcp.Tcp.X + 0.01, homeTcp.Tcp.Y, homeTcp.Tcp.Z,
            homeTcp.Tcp.Qw, homeTcp.Tcp.Qx, homeTcp.Tcp.Qy, homeTcp.Tcp.Qz));
        Assert.True(ik.TrySolve(nudged, home, out _), "1cm TCP nudge from home should be IK-able");
    }

    [Fact]
    public void NonUrSerial_FarGoal_StatusNamesNumericalIkReason()
    {
        var tree = SerialKinematicTrees.FromLengths(new[] { 0.3, 0.3, 0.2, 0.15, 0.1, 0.08 }, rail: false);
        var tip = tree.ExtractSerialTip("base_link", "tool0");
        var limits = new List<JointLimit>(tip.Chain.Joints.Length);
        foreach (var name in tip.JointNames)
        {
            var j = tree.Joints.First(jj => string.Equals(jj.Name, name, StringComparison.OrdinalIgnoreCase));
            limits.Add(new JointLimit(j.Lower, j.Upper, Math.PI, Math.PI * 2));
        }

        var preset = new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = "serial_arm",
            Family = "serial",
            AxisCount = tip.Chain.Joints.Length,
            JointLimits = limits,
            BaseFrame = BaseFrame.Identity,
            ToolFrame = ToolFrame.Identity,
        };
        var robot = new RobotModel(preset);
        var home = new JointState(new double[preset.AxisCount]);
        var far = new CartesianPose(new Frame(50, 0, 0));

        var result = new CartesianGoalSolver().TryReach(
            robot, far, CartesianGoalSolver.EnumerateDefaultSeeds(home, robot), tip.Chain);

        Assert.False(result.Success);
        Assert.Contains(result.Errors, e =>
            e.Contains("IK NoConvergence", StringComparison.Ordinal) ||
            e.Contains("IK SingularJacobian", StringComparison.Ordinal) ||
            e.Contains("IK InvalidInput", StringComparison.Ordinal));
    }
}
