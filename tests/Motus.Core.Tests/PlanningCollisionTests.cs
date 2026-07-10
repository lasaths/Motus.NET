using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class PlanningCollisionTests
{
    private static RobotModel Ur5e()
    {
        var preset = new RobotPreset
        {
            Manufacturer = RobotManufacturer.UniversalRobots,
            ModelName = "UR5e",
            AxisCount = 6,
            JointLimits = Enumerable.Range(0, 6).Select(_ => new JointLimit(-6.28, 6.28)).ToList()
        };
        return new RobotModel(preset);
    }

    [Fact]
    public void JointLinear_FailsWhenSceneWithoutChecker()
    {
        var robot = Ur5e();
        var scene = new CollisionScene(new[] { CollisionObject.Sphere("obs", new Frame(0.5, 0, 0.5), 0.1) });
        var result = new JointLinearPlanner().Plan(new PlanningRequest(
            robot,
            new JointState(new double[6]),
            new JointState(Enumerable.Repeat(0.2, 6).ToArray()),
            new PlanningOptions { CollisionScene = scene }));
        Assert.False(result.Success);
        Assert.Contains(result.Errors, e => e.Contains("ICollisionChecker", StringComparison.OrdinalIgnoreCase));
    }

    [Fact]
    public void JointLinear_ValidatesCollisionWhenCheckerSupplied()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", FindResources());
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var start = new JointState(new double[6]);
        var goal = new JointState(Enumerable.Repeat(0.1, 6).ToArray());
        var scene = new CollisionScene(new[] { CollisionObject.Sphere("far", new Frame(2, 2, 2), 0.05) });
        var result = new JointLinearPlanner().Plan(new PlanningRequest(
            robot, start, goal, new PlanningOptions { CollisionScene = scene, CollisionChecker = checker }));
        Assert.True(result.Success, string.Join("; ", result.Errors));
    }

    private static string FindResources()
    {
        var dir = AppContext.BaseDirectory;
        for (var i = 0; i < 8 && dir is not null; i++)
        {
            var candidate = Path.Combine(dir, "resources", "robots");
            if (Directory.Exists(candidate)) return candidate;
            dir = Directory.GetParent(dir)?.FullName;
        }
        throw new InvalidOperationException("resources/robots not found");
    }
}

public class TrajectoryRetimerTests
{
    [Fact]
    public void Retime_IncreasesDurationAndMonotonicTime()
    {
        var robot = new RobotModel(new RobotPreset
        {
            Manufacturer = RobotManufacturer.UniversalRobots,
            ModelName = "UR5e",
            AxisCount = 6,
            JointLimits = Enumerable.Range(0, 6).Select(_ => new JointLimit(-6.28, 6.28, maxVelocityRadiansPerSecond: 0.5)).ToList()
        });
        var geo = new Trajectory(robot, new[]
        {
            new TrajectoryPoint(0, new JointState(new double[6])),
            new TrajectoryPoint(0.01, new JointState(Enumerable.Repeat(1.0, 6).ToArray()))
        });
        var retimed = TrajectoryRetimer.Retime(geo);
        Assert.True(retimed.DurationSeconds > geo.DurationSeconds);
        var val = new TrajectoryValidator().Validate(retimed);
        Assert.True(val.IsValid, string.Join("; ", val.Errors));
    }

    [Fact]
    public void TotgLite_Algorithm_IsSupportedAndMonotonic()
    {
        var robot = new RobotModel(new RobotPreset
        {
            Manufacturer = RobotManufacturer.UniversalRobots,
            ModelName = "UR5e",
            AxisCount = 6,
            JointLimits = Enumerable.Range(0, 6).Select(_ => new JointLimit(-6.28, 6.28, maxVelocityRadiansPerSecond: 0.8)).ToList()
        });
        var geo = new Trajectory(robot, new[]
        {
            new TrajectoryPoint(0, new JointState(new double[6])),
            new TrajectoryPoint(0, new JointState(new[] { 0.2, -0.2, 0.2, -0.2, 0.1, -0.1 })),
            new TrajectoryPoint(0, new JointState(new[] { 0.3, -0.25, 0.25, -0.25, 0.15, -0.1 }))
        });

        var retimed = TrajectoryRetimer.Retime(geo, new TrajectoryRetimerOptions
        {
            Algorithm = RetimerAlgorithm.TotgLite
        });

        Assert.True(retimed.DurationSeconds > 0);
        for (var i = 1; i < retimed.Points.Count; i++)
            Assert.True(retimed.Points[i].TimeSeconds >= retimed.Points[i - 1].TimeSeconds);
    }
}
