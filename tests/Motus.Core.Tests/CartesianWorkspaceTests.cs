using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class CartesianWorkspaceTests
{
    private static string ResourcesRoot =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

    [Fact]
    public void When_GoalFiveMetersAway_Then_OutsideReach()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var fk = new DhForwardKinematics(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var startTcp = fk.ComputeTcp(home, preset.BaseFrame, preset.ToolFrame);
        var farGoal = new CartesianPose(new Frame(
            startTcp.Tcp.X + 5.0, startTcp.Tcp.Y, startTcp.Tcp.Z,
            startTcp.Tcp.Qw, startTcp.Tcp.Qx, startTcp.Tcp.Qy, startTcp.Tcp.Qz));

        var check = CartesianWorkspace.CheckReach(preset, farGoal, startTcp);

        Assert.False(check.IsWithinReach);
        Assert.Contains("reach", check.Reason, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void When_Example01GoalFromHome_Then_WithinReach()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var fk = new DhForwardKinematics(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var goalTcp = fk.ComputeTcp(goalJ, preset.BaseFrame, preset.ToolFrame);
        var startTcp = fk.ComputeTcp(home, preset.BaseFrame, preset.ToolFrame);

        var check = CartesianWorkspace.CheckReach(preset, goalTcp, startTcp);

        Assert.True(check.IsWithinReach);
    }
}
