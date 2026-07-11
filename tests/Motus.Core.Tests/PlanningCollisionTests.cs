using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
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

    [Fact]
    public void CartesianLin_CanSucceedWithSphereOnTcpLine_WhenLinkEnvelopesClear()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", FindResources());
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var extendedTool = new ToolDefinition("probe", new Frame(0, 0, 0.18, 1, 0, 0, 0)).ToToolFrame();
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var goalTcp = fk.ComputeTcp(goalJ, preset.BaseFrame, preset.ToolFrame);

        var linResult = new CartesianLinearPathPlanner(preset).PlanToResult(
            new CartesianPlanningRequest(robot, home, goalTcp, new PlanningOptions { MaxJointStepRadians = 0.05 }),
            new CartesianLinOptions(StepMeters: 0.005, ContinueOnIkFailure: false));
        Assert.True(linResult.Success, string.Join("; ", linResult.Errors));
        Assert.NotNull(linResult.Trajectory);

        CollisionScene? tcpOnlyScene = null;
        var points = linResult.Trajectory!.Points;
        for (var i = 1; i < points.Count; i++)
        {
            var tcpA = fk.ComputeTcp(points[i - 1].JointState, preset.BaseFrame, extendedTool).Tcp;
            var tcpB = fk.ComputeTcp(points[i].JointState, preset.BaseFrame, extendedTool).Tcp;
            var tcpMid = new Frame(
                (tcpA.X + tcpB.X) / 2,
                (tcpA.Y + tcpB.Y) / 2,
                (tcpA.Z + tcpB.Z) / 2);
            foreach (var radius in new[] { 0.01, 0.015, 0.02, 0.025, 0.03 })
            {
                var trial = new CollisionScene(new[] { CollisionObject.Sphere("tcp_only", tcpMid, radius) });
                if (PlanningCollision.ValidateTrajectory(linResult.Trajectory, trial, checker, 0.05) is null)
                {
                    tcpOnlyScene = trial;
                    break;
                }
            }

            if (tcpOnlyScene is not null) break;
        }

        Assert.NotNull(tcpOnlyScene);
    }

    [Fact]
    public void CartesianLin_FailsWhenSphereOverlapsLinkEnvelope()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", FindResources());
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var goalTcp = fk.ComputeTcp(goalJ, preset.BaseFrame, preset.ToolFrame);

        var linResult = new CartesianLinearPathPlanner(preset).PlanToResult(
            new CartesianPlanningRequest(robot, home, goalTcp, new PlanningOptions { MaxJointStepRadians = 0.05 }),
            new CartesianLinOptions(StepMeters: 0.005, ContinueOnIkFailure: false));
        Assert.True(linResult.Success);
        Assert.NotNull(linResult.Trajectory);

        CollisionScene? blocking = null;
        foreach (var pt in linResult.Trajectory!.Points)
        {
            if (!checker.IsCollisionFree(pt.JointState, new CollisionScene())) continue;
            var origins = fk.ComputeLinkOrigins(pt.JointState.Positions, preset.BaseFrame.Frame);
            foreach (var origin in origins)
            {
                var trial = new CollisionScene(new[] { CollisionObject.Sphere("block", origin, 0.12) });
                if (!checker.IsCollisionFree(pt.JointState, trial))
                {
                    blocking = trial;
                    break;
                }
            }

            if (blocking is not null) break;
        }

        Assert.NotNull(blocking);

        var result = new CartesianLinearPathPlanner(preset).PlanToResult(
            new CartesianPlanningRequest(robot, home, goalTcp, new PlanningOptions
            {
                CollisionScene = blocking,
                CollisionChecker = checker,
                MaxJointStepRadians = 0.05
            }, blocking),
            new CartesianLinOptions(StepMeters: 0.005, ContinueOnIkFailure: false));

        Assert.False(result.Success);
        Assert.Contains(result.Errors, e => e.Contains("Collision", StringComparison.OrdinalIgnoreCase));
    }

    [Fact]
    public void CartesianLin_FailsFastWhenGoalTcpInCollision()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", FindResources());
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var goalTcp = fk.ComputeTcp(goalJ, preset.BaseFrame, preset.ToolFrame);
        var scene = new CollisionScene(new[] { CollisionObject.Sphere("goal_block", goalTcp.Tcp, 0.08) });

        var sw = System.Diagnostics.Stopwatch.StartNew();
        var result = new CartesianLinearPathPlanner(preset).PlanToResult(
            new CartesianPlanningRequest(robot, home, goalTcp, new PlanningOptions
            {
                CollisionScene = scene,
                CollisionChecker = checker,
                MaxJointStepRadians = 0.05
            }, scene),
            new CartesianLinOptions(StepMeters: 0.005, ContinueOnIkFailure: false));
        sw.Stop();

        Assert.False(result.Success);
        Assert.Contains(result.Errors, e => e.Contains("Goal configuration is in collision", StringComparison.OrdinalIgnoreCase));
        Assert.True(sw.ElapsedMilliseconds < 500, $"Expected fast-fail, took {sw.ElapsedMilliseconds}ms");
    }

    [Fact]
    public void Rrt_FailsFastWhenGoalInCollision()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", FindResources());
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var start = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var goal = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var goalTcp = fk.ComputeTcp(goal, preset.BaseFrame, preset.ToolFrame);
        var scene = new CollisionScene(new[] { CollisionObject.Sphere("goal_block", goalTcp.Tcp, 0.08) });

        var sw = System.Diagnostics.Stopwatch.StartNew();
        var result = new RrtConnectPlanner(checker, new RrtConnectOptions { MaxIterations = 4000, RandomSeed = 42 })
            .Plan(new PlanningRequest(robot, start, goal, new PlanningOptions
            {
                CollisionScene = scene,
                CollisionChecker = checker,
                MaxJointStepRadians = 0.08
            }));
        sw.Stop();

        Assert.False(result.Success);
        Assert.Contains(result.Errors, e => e.Contains("Goal configuration is in collision", StringComparison.OrdinalIgnoreCase));
        Assert.True(sw.ElapsedMilliseconds < 500, $"Expected fast-fail, took {sw.ElapsedMilliseconds}ms");
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
