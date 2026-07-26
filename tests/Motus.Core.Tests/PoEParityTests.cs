using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class PoEParityTests
{
    private static string FixturePath(string name) =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", name));

    [Fact]
    public void TwoLink_FKinSpace_MatchesSerialFk_AndBody()
    {
        var robot = UrdfRobotLoader.Load(FixturePath("two_link.urdf"), new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tip_link",
            ModelName = "two_link"
        });

        var poe = ProductOfExponentials.FromSerialChain(robot.Chain);
        var serial = new SerialForwardKinematics(robot.Chain);
        double[][] configs =
        [
            [0, 0],
            [0.3, -0.5],
            [1.0, 0.25],
            [-0.7, 1.1]
        ];

        foreach (var q in configs)
        {
            var serialT = serial.ComputeFlangeTransform(q);
            var spaceT = poe.FKinSpace(q);
            var bodyT = poe.FKinBody(q);
            AssertPoseClose(serialT, spaceT, $"space vs serial at [{q[0]},{q[1]}]");
            AssertPoseClose(spaceT, bodyT, $"space vs body at [{q[0]},{q[1]}]");
        }
    }

    [Fact]
    public void Ur10e_FKinSpace_MatchesSerialFk()
    {
        var robot = UrdfRobotLoader.Load(FixturePath("ur10e/ur10e.urdf"), new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tool0",
            ModelName = "ur10e"
        });

        var poe = ProductOfExponentials.FromSerialChain(robot.Chain);
        var serial = new SerialForwardKinematics(robot.Chain);
        double[][] configs =
        [
            [0, 0, 0, 0, 0, 0],
            [0.1, -0.5, 0.8, -0.3, -0.4, 0.2],
            [1.0, -1.2, 1.5, -0.8, 0.5, -0.3]
        ];

        foreach (var q in configs)
        {
            var serialT = serial.ComputeFlangeTransform(q);
            var spaceT = poe.FKinSpace(q);
            var bodyT = poe.FKinBody(q);
            AssertPoseClose(serialT, spaceT, "space vs serial");
            AssertPoseClose(spaceT, bodyT, "space vs body");
        }
    }

    private static void AssertPoseClose(double[] a, double[] b, string label, double posTol = 1e-8, double rotTol = 1e-7)
    {
        var dx = a[3] - b[3];
        var dy = a[7] - b[7];
        var dz = a[11] - b[11];
        var pos = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        Assert.True(pos < posTol, $"{label}: pos err {pos}");

        for (var i = 0; i < 3; i++)
        for (var j = 0; j < 3; j++)
        {
            var ia = i * 4 + j;
            Assert.True(Math.Abs(a[ia] - b[ia]) < rotTol, $"{label}: R[{i},{j}] {a[ia]} vs {b[ia]}");
        }
    }
}

public class NumericalIkOptionsTests
{
    private static string FixturePath(string name) =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", name));

    [Fact]
    public void TrySolveDetailed_InvalidSeed_ReturnsInvalidInput()
    {
        var robot = UrdfRobotLoader.Load(FixturePath("two_link.urdf"), new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tip_link"
        });
        var ik = new NumericalInverseKinematics(
            KinematicsResolver.CreateFkSolver(robot.Preset, robot.Chain),
            robot.Preset);
        var target = new CartesianPose(new Frame(0.2, 0, 0.1));
        var bad = new JointState([double.NaN, 0]);
        var r = ik.TrySolveDetailed(target, bad);
        Assert.False(r.Success);
        Assert.Equal(NumericalIkFailureReasons.InvalidInput, r.FailureReason);
    }

    [Fact]
    public void AggressiveMaxIterations_StillSolvesEasyTwoLink()
    {
        var robot = UrdfRobotLoader.Load(FixturePath("two_link.urdf"), new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tip_link"
        });
        var fk = KinematicsResolver.CreateFkSolver(robot.Preset, robot.Chain);
        var seed = new JointState([0.2, -0.3]);
        var target = fk.ComputeTcp(seed, robot.Preset.BaseFrame, robot.Preset.ToolFrame);
        var ik = new NumericalInverseKinematics(fk, robot.Preset, NumericalIkOptions.ModernRoboticsAggressive);
        var r = ik.TrySolveDetailed(target, seed);
        Assert.True(r.Success, r.FailureReason);
        Assert.True(r.Iterations <= 20);
    }

    [Fact]
    public void DefaultOptions_PreservesRoundTrip()
    {
        var robot = UrdfRobotLoader.Load(FixturePath("two_link.urdf"), new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tip_link"
        });
        var fk = KinematicsResolver.CreateFkSolver(robot.Preset, robot.Chain);
        var ik = new NumericalInverseKinematics(fk, robot.Preset);
        var joints = new JointState([0.3, -0.5]);
        var pose = fk.ComputeTcp(joints, robot.Preset.BaseFrame, robot.Preset.ToolFrame);
        Assert.True(ik.TrySolve(pose, joints, out var solved));
        var check = fk.ComputeTcp(solved, robot.Preset.BaseFrame, robot.Preset.ToolFrame);
        var posErr = Math.Sqrt(
            Math.Pow(check.Tcp.X - pose.Tcp.X, 2) +
            Math.Pow(check.Tcp.Y - pose.Tcp.Y, 2) +
            Math.Pow(check.Tcp.Z - pose.Tcp.Z, 2));
        Assert.True(posErr < 0.005);
    }
}
