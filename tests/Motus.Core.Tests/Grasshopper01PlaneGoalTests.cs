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
        var minOriErr = double.MaxValue;
        var verified = 0;
        foreach (var c in UrAnalyticInverseKinematics.EnumerateSolutions(chain, targetM, preset.JointLimits))
        {
            var actual = fk.ComputeTcp(c, preset.BaseFrame, preset.ToolFrame);
            var err = Math.Sqrt(
                Math.Pow(actual.Tcp.X - goalTcp.Tcp.X, 2) +
                Math.Pow(actual.Tcp.Y - goalTcp.Tcp.Y, 2) +
                Math.Pow(actual.Tcp.Z - goalTcp.Tcp.Z, 2));
            var dot = Math.Abs(
                actual.Tcp.Qw * goalTcp.Tcp.Qw + actual.Tcp.Qx * goalTcp.Tcp.Qx +
                actual.Tcp.Qy * goalTcp.Tcp.Qy + actual.Tcp.Qz * goalTcp.Tcp.Qz);
            var oriErr = 2 * Math.Acos(Math.Clamp(dot, -1, 1));
            minErr = Math.Min(minErr, err);
            minOriErr = Math.Min(minOriErr, oriErr);
            if (err < 5e-3 && oriErr < 0.05) verified++;
        }

        Assert.True(verified > 0, $"No analytic candidate FK-verified; min pos {minErr:F4}m, min ori {minOriErr:F4} rad");
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

    [Fact]
    public void Ur5e_HomeToExample02Tcp_LinFinalSegmentContinuous()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var fk = new DhForwardKinematics(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var goalTcp = fk.ComputeTcp(goalJ, preset.BaseFrame, preset.ToolFrame);
        var startPose = fk.ComputeTcp(home, preset.BaseFrame, preset.ToolFrame);

        var traj = new CartesianLinearPathPlanner(preset).Plan(startPose, goalTcp, home, 0.005, continueOnIKFailure: false);
        Assert.NotNull(traj);
        Assert.True(traj!.Points.Count >= 2);

        var prev = traj.Points[^2].JointState;
        var last = traj.Points[^1].JointState;
        var finalJump = MaxJointDelta(prev, last);
        Assert.True(finalJump < 0.35, $"Final segment joint jump {finalJump:F3} rad (likely wrist flip at goal)");

        var lastTcp = fk.ComputeTcp(last, preset.BaseFrame, preset.ToolFrame).Tcp;
        var oriErr = OrientationErrorRad(lastTcp, goalTcp.Tcp);
        Assert.True(oriErr < 0.05, $"Final TCP orientation error {oriErr:F4} rad");
    }

    private static double MaxJointDelta(JointState a, JointState b)
    {
        var max = 0.0;
        for (var i = 0; i < a.AxisCount; i++)
            max = Math.Max(max, Math.Abs(b.Positions[i] - a.Positions[i]));
        return max;
    }

    private static double OrientationErrorRad(Motus.Core.Frame a, Motus.Core.Frame b)
    {
        var dot = Math.Abs(a.Qw * b.Qw + a.Qx * b.Qx + a.Qy * b.Qy + a.Qz * b.Qz);
        return 2 * Math.Acos(Math.Clamp(dot, -1, 1));
    }
}
