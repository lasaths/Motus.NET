using Motus.Core;
using Motus.Geometry;
using Xunit;

namespace Motus.Core.Tests;

public class StewartPlatformIkTests
{
    private static StewartRobot Classic() => StewartRobot.CreateClassic();

    [Fact]
    public void CreateClassic_Preset_FamilyIsStewart_LimitsAreMeters()
    {
        var robot = Classic();
        Assert.Equal(Units.StewartFamily, robot.Model.Preset.Family);
        Assert.Equal(6, robot.Model.Preset.AxisCount);
        Assert.All(robot.Model.Preset.JointLimits, l => Assert.Equal(JointCoordinateUnit.Meters, l.Unit));
    }

    [Fact]
    public void Ik_HomePose_WithinStroke()
    {
        var robot = Classic();
        var mid = 0.5 * (robot.Platform.StrokeLimits[0].Min + robot.Platform.StrokeLimits[0].Max);
        var pose = new CartesianPose(new Frame(0, 0, mid));
        var ik = robot.InverseKinematics.TrySolveDetailed(pose);
        Assert.True(ik.Success, ik.ToString());
        Assert.NotNull(ik.JointState);
        Assert.Equal(6, ik.JointState!.AxisCount);
        for (var i = 0; i < 6; i++)
            Assert.True(robot.Platform.StrokeLimits[i].Contains(ik.JointState.Positions[i]));
    }

    [Fact]
    public void Ik_StrokeLimit_FailsWithReason()
    {
        var robot = Classic();
        // Far above workspace.
        var pose = new CartesianPose(new Frame(0, 0, 5.0));
        var ik = robot.InverseKinematics.TrySolveDetailed(pose);
        Assert.False(ik.Success);
        Assert.Equal(KinematicsReason.StrokeLimit, ik.Reason);
    }

    [Fact]
    public void Ik_NonFinite_FailsInvalidInput()
    {
        var robot = Classic();
        var pose = new CartesianPose(new Frame(double.NaN, 0, 0.6));
        var ik = robot.InverseKinematics.TrySolveDetailed(pose);
        Assert.False(ik.Success);
        Assert.Equal(KinematicsReason.InvalidInput, ik.Reason);
    }

    [Fact]
    public void DegenerateAnchors_Throw()
    {
        var zeros = Enumerable.Repeat(new Vec3(0, 0, 0), 6).ToArray();
        var limits = Enumerable.Range(0, 6).Select(_ => JointLimit.Meters(0.4, 0.7)).ToArray();
        Assert.ThrowsAny<ArgumentException>(() =>
            new StewartPlatform("bad", zeros, zeros, limits));
    }

    [Fact]
    public void Loader_RoundTrip_Json()
    {
        var robot = Classic();
        var json = """
            {
              "schemaVersion": 1,
              "modelName": "from_json",
              "baseAnchors": [
                {"x":0.5,"y":0,"z":0},{"x":0.25,"y":0.433,"z":0},{"x":-0.25,"y":0.433,"z":0},
                {"x":-0.5,"y":0,"z":0},{"x":-0.25,"y":-0.433,"z":0},{"x":0.25,"y":-0.433,"z":0}
              ],
              "platformAnchors": [
                {"x":0.3,"y":0,"z":0},{"x":0.15,"y":0.26,"z":0},{"x":-0.15,"y":0.26,"z":0},
                {"x":-0.3,"y":0,"z":0},{"x":-0.15,"y":-0.26,"z":0},{"x":0.15,"y":-0.26,"z":0}
              ],
              "strokeMinMeters": 0.45,
              "strokeMaxMeters": 0.75
            }
            """;
        var loaded = StewartPlatformLoader.LoadJson(json);
        Assert.Equal("from_json", loaded.ModelName);
        var ik = new StewartInverseKinematics(loaded).TrySolveDetailed(new CartesianPose(new Frame(0, 0, 0.6)));
        Assert.True(ik.Success, ik.ToString());
    }

    [Fact]
    public void Loader_WrongSchema_Throws()
    {
        Assert.Throws<InvalidDataException>(() =>
            StewartPlatformLoader.LoadJson("""{"schemaVersion":99,"modelName":"x","baseAnchors":[],"platformAnchors":[]}"""));
    }
}

public class VerifiedStewartKinematicsTests
{
    private static StewartRobot Classic() => StewartRobot.CreateClassic();

    [Fact]
    public void IkFk_RoundTrip_Home()
    {
        var robot = Classic();
        var mid = 0.5 * (robot.Platform.StrokeLimits[0].Min + robot.Platform.StrokeLimits[0].Max);
        var target = new CartesianPose(new Frame(0, 0, mid));
        AssertIkFkRoundTrip(robot, target);
    }

    [Fact]
    public void IkFk_RoundTrip_Translate()
    {
        var robot = Classic();
        var mid = 0.5 * (robot.Platform.StrokeLimits[0].Min + robot.Platform.StrokeLimits[0].Max);
        var target = new CartesianPose(new Frame(0.01, -0.008, mid));
        AssertIkFkRoundTrip(robot, target);
    }

    [Fact]
    public void IkFk_RoundTrip_SmallTilt()
    {
        var robot = Classic();
        var mid = 0.5 * (robot.Platform.StrokeLimits[0].Min + robot.Platform.StrokeLimits[0].Max);
        // ~2 deg about X
        var m = Transforms.FromRpy(0, 0, mid, 0.035, 0, 0);
        var target = new CartesianPose(Transforms.ToFrame(m));
        AssertIkFkRoundTrip(robot, target);
    }

    [Fact]
    public void Path_TcpLin_Succeeds()
    {
        var robot = Classic();
        var mid = 0.5 * (robot.Platform.StrokeLimits[0].Min + robot.Platform.StrokeLimits[0].Max);
        var start = new CartesianPose(new Frame(0, 0, mid));
        var goal = new CartesianPose(new Frame(0.015, 0, mid));
        var result = robot.PathPlanner.PlanToResult(start, goal, stepMeters: 0.005);
        Assert.True(result.Success, string.Join("; ", result.Errors));
        Assert.NotNull(result.Trajectory);
        Assert.True(result.Trajectory!.Points.Count >= 2);
    }

    [Fact]
    public void Path_Toolpath_MultiWaypoint()
    {
        var robot = Classic();
        var mid = 0.5 * (robot.Platform.StrokeLimits[0].Min + robot.Platform.StrokeLimits[0].Max);
        var wps = new[]
        {
            new CartesianPose(new Frame(0, 0, mid)),
            new CartesianPose(new Frame(0.01, 0, mid)),
            new CartesianPose(new Frame(0.01, 0.01, mid))
        };
        var result = robot.PathPlanner.PlanToolpath(wps, stepMeters: 0.005);
        Assert.True(result.Success, string.Join("; ", result.Errors));
        Assert.NotNull(result.Trajectory);
    }

    [Fact]
    public void Path_OutOfWorkspace_FailsStroke()
    {
        var robot = Classic();
        var mid = 0.5 * (robot.Platform.StrokeLimits[0].Min + robot.Platform.StrokeLimits[0].Max);
        var start = new CartesianPose(new Frame(0, 0, mid));
        var goal = new CartesianPose(new Frame(0, 0, 5));
        var result = robot.PathPlanner.PlanToResult(start, goal);
        Assert.False(result.Success);
        Assert.Contains(result.Errors, e => e.Contains("StrokeLimit", StringComparison.OrdinalIgnoreCase) || e.Contains("stroke", StringComparison.OrdinalIgnoreCase));
    }

    private static void AssertIkFkRoundTrip(StewartRobot robot, CartesianPose target)
    {
        var ik = robot.InverseKinematics.TrySolveDetailed(target);
        Assert.True(ik.Success, $"IK failed: {ik}");
        var fk = robot.ForwardKinematics.TrySolve(ik.JointState!, seedPose: target);
        Assert.True(fk.Success, $"FK failed: {fk}");
        Assert.NotNull(fk.Pose);
        var a = target.Tcp;
        var b = fk.Pose!.Tcp;
        var posErr = Math.Sqrt(
            (a.X - b.X) * (a.X - b.X) +
            (a.Y - b.Y) * (a.Y - b.Y) +
            (a.Z - b.Z) * (a.Z - b.Z));
        Assert.True(posErr < 1e-4, $"FK position error {posErr} m");
        var dot = Math.Abs(a.Qw * b.Qw + a.Qx * b.Qx + a.Qy * b.Qy + a.Qz * b.Qz);
        Assert.True(dot > 0.999, $"FK orientation dot {dot}");
    }
}
