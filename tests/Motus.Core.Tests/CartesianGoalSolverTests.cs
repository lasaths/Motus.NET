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
        var urdfPath = Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory,
            "..", "..", "..", "..", "..", "..",
            "Motus.Grasshopper", "resources", "robots", "ur10e_robotiq", "ur10e_robotiq.urdf"));
        Assert.True(File.Exists(urdfPath));

        var bundle = UrdfRobotLoader.Load(urdfPath, new UrdfLoadOptions { BaseLink = "base_link", TipLink = "tool0" });
        var robot = bundle.ToModel();
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var joint = new JointLinearPlanner().Plan(new PlanningRequest(robot, home, goalJ));
        Assert.True(joint.Success, string.Join("; ", joint.Errors));
    }

    [Fact]
    public void Ur10eRobotiq_IkFromHomeToTinyTcpDelta()
    {
        var urdfPath = Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory,
            "..", "..", "..", "..", "..", "..",
            "Motus.Grasshopper", "resources", "robots", "ur10e_robotiq", "ur10e_robotiq.urdf"));
        var bundle = UrdfRobotLoader.Load(urdfPath, new UrdfLoadOptions { BaseLink = "base_link", TipLink = "tool0" });
        var robot = bundle.ToModel();
        var chain = bundle.Chain;
        var fk = KinematicsResolver.CreateFkSolver(robot.Preset, chain);
        var ik = KinematicsResolver.CreateInverseKinematics(robot.Preset, chain);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var homeTcp = fk.ComputeTcp(home, robot.Preset.BaseFrame, robot.Preset.ToolFrame);
        var nudged = new CartesianPose(new Frame(
            homeTcp.Tcp.X + 0.01, homeTcp.Tcp.Y, homeTcp.Tcp.Z,
            homeTcp.Tcp.Qw, homeTcp.Tcp.Qx, homeTcp.Tcp.Qy, homeTcp.Tcp.Qz));
        Assert.True(ik.TrySolve(nudged, home, out _), "1cm TCP nudge from home should be IK-able");
    }
}
