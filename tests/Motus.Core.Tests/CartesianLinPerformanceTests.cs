using System.Diagnostics;
using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class CartesianLinPerformanceTests
{
    private static string ResourcesRoot =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

    [Fact]
    public void When_UnreachableFiveMeterTarget_Then_FailsUnderFiveHundredMs()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var fk = new DhForwardKinematics(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var startTcp = fk.ComputeTcp(home, preset.BaseFrame, preset.ToolFrame);
        var farGoal = new CartesianPose(new Frame(
            startTcp.Tcp.X + 5.0, startTcp.Tcp.Y, startTcp.Tcp.Z,
            startTcp.Tcp.Qw, startTcp.Tcp.Qx, startTcp.Tcp.Qy, startTcp.Tcp.Qz));

        var sw = Stopwatch.StartNew();
        var result = new CartesianLinearPathPlanner(preset).PlanToResult(
            new CartesianPlanningRequest(robot, home, farGoal, new PlanningOptions()));
        sw.Stop();

        Assert.False(result.Success);
        Assert.True(sw.ElapsedMilliseconds < 500, $"Expected fast reject, took {sw.ElapsedMilliseconds}ms");
    }

    [Fact]
    public void When_FarOnePointFiveMeterMove_Then_CompletesOrFailsUnderOneSecond()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var fk = new DhForwardKinematics(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var startTcp = fk.ComputeTcp(home, preset.BaseFrame, preset.ToolFrame);
        var farGoal = new CartesianPose(new Frame(
            0.8, 0.5, 0.9,
            startTcp.Tcp.Qw, startTcp.Tcp.Qx, startTcp.Tcp.Qy, startTcp.Tcp.Qz));

        var sw = Stopwatch.StartNew();
        _ = new CartesianLinearPathPlanner(preset).PlanToResult(
            new CartesianPlanningRequest(robot, home, farGoal, new PlanningOptions()));
        sw.Stop();

        Assert.True(sw.ElapsedMilliseconds < 1000, $"Expected bounded LIN cost, took {sw.ElapsedMilliseconds}ms");
    }
}
