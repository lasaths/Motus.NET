using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class Grasshopper01PlaneGoalTests
{
    private static string ResourcesRoot =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

    [Fact]
    public void Ur5e_Example01JointGoal_FkRoundTripIkWorks()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var fk = new DhForwardKinematics(preset);
        var ik = KinematicsResolver.CreateInverseKinematics(preset);
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var goalTcp = fk.ComputeTcp(goalJ, preset.BaseFrame, preset.ToolFrame);

        Assert.True(ik.TrySolve(goalTcp, goalJ, out _), "IK should solve when seeded with the FK source config");
    }

    [Fact]
    public void Ur5e_AnalyticIk_FkVerify_Example01Goal()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var fk = new DhForwardKinematics(preset);
        var chain = KinematicsProfiles.GetRequired(preset);
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var goalTcp = fk.ComputeTcp(goalJ, preset.BaseFrame, preset.ToolFrame);
        var targetM = Transforms.FromFrame(goalTcp.Tcp);

        var minErr = double.MaxValue;
        var verified = 0;
        foreach (var c in UrAnalyticInverseKinematics.EnumerateSolutions(chain, targetM, preset.JointLimits))
        {
            var actual = fk.ComputeTcp(c, preset.BaseFrame, preset.ToolFrame);
            var err = Math.Sqrt(
                Math.Pow(actual.Tcp.X - goalTcp.Tcp.X, 2) +
                Math.Pow(actual.Tcp.Y - goalTcp.Tcp.Y, 2) +
                Math.Pow(actual.Tcp.Z - goalTcp.Tcp.Z, 2));
            minErr = Math.Min(minErr, err);
            if (err < 5e-3) verified++;
        }

        Assert.True(verified > 0, $"No analytic candidate FK-verified; min error {minErr:F4}m");
    }

    [Fact]
    public void Ur5e_HomeToExample01JointGoalTcp_IkAndLin()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var fk = new DhForwardKinematics(preset);
        var ik = KinematicsResolver.CreateInverseKinematics(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var goalTcp = fk.ComputeTcp(goalJ, preset.BaseFrame, preset.ToolFrame);

        Assert.True(ik.TrySolve(goalTcp, home, out _), "IK from viewer home should reach example 01 TCP goal");

        var lin = new CartesianLinearPathPlanner(preset).PlanToResult(
            new CartesianPlanningRequest(robot, home, goalTcp, new PlanningOptions()), 0.005);
        Assert.True(lin.Success, string.Join("; ", lin.Errors));
    }
}
