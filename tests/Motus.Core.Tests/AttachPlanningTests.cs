using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;

namespace Motus.Core.Tests;

public class PlanningContextTests
{
    [Fact]
    public void Attach_HidesSceneObstacle()
    {
        var preset = PresetLoader.LoadByModelName("UR5e");
        var robot = new RobotModel(preset);
        var box = CollisionObject.Box("workpiece", new Frame(0.5, 0, 0.3), 0.05, 0.05, 0.05);
        var table = CollisionObject.Box("table", Frame.Identity, 1, 1, 0.05);
        var scene = new CollisionScene(new[] { box, table });
        var ctx = PlanningContext.Create(robot, scene);
        var attached = ctx.Attach("workpiece", box, Frame.Identity);
        Assert.Single(attached.Scene.Objects);
        Assert.Equal("table", attached.Scene.Objects[0].Name);
        Assert.Single(attached.Attached);
    }

    [Fact]
    public void Detach_RestoresSceneObstacleAtWorldPose()
    {
        var preset = PresetLoader.LoadByModelName("UR5e");
        var robot = new RobotModel(preset);
        var box = CollisionObject.Box("workpiece", new Frame(0.5, 0, 0.3), 0.05, 0.05, 0.05);
        var scene = new CollisionScene(new[] { box });
        var worldPose = new Frame(0.55, 0.05, 0.35);
        var ctx = PlanningContext.Create(robot, scene)
            .Attach("workpiece", box, Frame.Identity)
            .Detach("workpiece", worldPose);

        Assert.Empty(ctx.Attached);
        var restored = Assert.Single(ctx.Scene.Objects);
        Assert.Equal("workpiece", restored.Name);
    }

    [Fact]
    public void AttachAwareChecker_WrapsSphereChecker()
    {
        var preset = PresetLoader.LoadByModelName("UR5e");
        var robot = new RobotModel(preset);
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var inner = new SphereCollisionChecker(preset);
        var attached = new AttachedBody("tool", Frame.Identity, CollisionObject.Sphere("tool", Frame.Identity, 0.04));
        var checker = new AttachAwareCollisionChecker(inner, fk, preset.BaseFrame, preset.ToolFrame, new[] { attached });
        var start = new JointState(new double[6]);
        Assert.True(checker.IsCollisionFree(start, new CollisionScene()));
    }

    [Fact]
    public void MotusCapabilities_Describe_IncludesAttach()
    {
        var desc = MotusCapabilities.Describe();
        Assert.Contains("attach", desc, StringComparison.OrdinalIgnoreCase);
    }
}

public class AttachPlanningTests
{
    [Fact]
    public void AttachAtTcp_RrtConnect_PathValidatesWithAttachedVolume()
    {
        var preset = PresetLoader.LoadByModelName("UR5e");
        var robot = new RobotModel(preset);
        var start = new JointState(new double[] { 0, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });
        var goal = new JointState(new double[] { 0.4, -1.0, 1.6, 0.1, 1.1, 0.1 });

        var table = CollisionObject.Box("table", new Frame(0.6, 0, -0.02), 0.3, 0.3, 0.02);
        var workpiece = CollisionObject.Box("workpiece", new Frame(0.6, 0, 0.08), 0.03, 0.03, 0.03);
        var scene = new CollisionScene(new CollisionObject[] { workpiece, table });

        var checker = CollisionCheckerFactory.Create(robot);
        Assert.True(checker.IsCollisionFree(start, scene), "start must be free before attach");

        var ctx = PlanningContext.Create(robot, scene)
            .Attach("workpiece", workpiece, new Frame(0, 0, -0.06));

        checker = CollisionCheckerFactory.Create(robot, attached: ctx.Attached);
        Assert.True(checker.IsCollisionFree(start, ctx.Scene), "start must be free with attached body");

        var opts = ctx.ToPlanningOptions(new PlanningOptions
        {
            CollisionChecker = checker,
            MaxJointStepRadians = 0.08
        });

        var planner = new RrtConnectPlanner(checker, new RrtConnectOptions { MaxIterations = 8000, RandomSeed = 11 });
        var result = planner.Plan(new PlanningRequest(robot, start, goal, opts));
        Assert.True(result.Success, string.Join("; ", result.Errors));

        var validator = new TrajectoryValidator();
        var val = validator.Validate(result.Trajectory!, new TrajectoryValidationOptions
        {
            CollisionChecker = checker,
            CollisionScene = ctx.Scene
        });
        Assert.True(val.IsValid, string.Join("; ", val.Errors));
    }

    [Fact]
    public void AttachedBody_MovesWithTcp()
    {
        var preset = PresetLoader.LoadByModelName("UR5e");
        var robot = new RobotModel(preset);
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var q0 = new JointState(new double[] { 0, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });
        var q1 = new JointState(new double[] { 0.3, -1.0, 1.5, 0.1, 1.2, 0.2 });
        var body = new AttachedBody("part", Frame.Identity, CollisionObject.Sphere("part", Frame.Identity, 0.05), "part");
        var checker = CollisionCheckerFactory.Create(robot, attached: new[] { body });
        var scene = new CollisionScene();
        Assert.True(checker.IsCollisionFree(q0, scene));
        Assert.True(checker.IsCollisionFree(q1, scene));
        var tcp0 = fk.ComputeTcp(q0, preset.BaseFrame, preset.ToolFrame);
        var tcp1 = fk.ComputeTcp(q1, preset.BaseFrame, preset.ToolFrame);
        Assert.True(Math.Abs(tcp0.Tcp.X - tcp1.Tcp.X) > 0.01);
    }

    /// <summary>Logic for GH example 10_pick_place.ghx — three programs with attach context swaps between plans.</summary>
    [Fact]
    public void Example10_PickAndPlace_Box()
    {
        var preset = PresetLoader.LoadByModelName("UR5e");
        var robot = new RobotModel(preset);
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var planner = new IndustrialMotionPlanner(preset);

        var home = new JointState(new double[] { 0, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });
        var workpiece = CollisionObject.Box("workpiece", new Frame(0.6, 0, 0.08), 0.03, 0.03, 0.03);
        var scene = new CollisionScene(new[] { workpiece });
        var tcpLocal = new Frame(0, 0, -0.06);

        var homeTcp = fk.ComputeTcp(home, preset.BaseFrame, preset.ToolFrame);
        var pickPose = homeTcp;
        var placePose = new CartesianPose(new Frame(
            homeTcp.Tcp.X - 0.05, homeTcp.Tcp.Y + 0.05, homeTcp.Tcp.Z,
            homeTcp.Tcp.Qw, homeTcp.Tcp.Qx, homeTcp.Tcp.Qy, homeTcp.Tcp.Qz));
        var retractPose = new CartesianPose(new Frame(
            homeTcp.Tcp.X - 0.05, homeTcp.Tcp.Y + 0.05, homeTcp.Tcp.Z + 0.10,
            homeTcp.Tcp.Qw, homeTcp.Tcp.Qx, homeTcp.Tcp.Qy, homeTcp.Tcp.Qz));

        var open = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.085 });
        var closed = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.0 });
        var caps = ToolCapabilities.Robotiq2F85;

        var ctx = PlanningContext.Create(robot, scene);
        var checker = CollisionCheckerFactory.Create(robot, attached: ctx.Attached);
        var opts = new PlanningOptions { CollisionChecker = checker, MaxJointStepRadians = 0.05 };

        var approach = planner.Plan(new MotionProgramRequest(
            robot,
            home,
            new MotionSegment[]
            {
                new LinSegment(pickPose, stepMeters: 0.005),
                new SetToolStateSegment(closed, durationSeconds: 0.1)
            },
            ctx.ToPlanningOptions(opts))
        {
            InitialToolState = open,
            ToolCapabilities = caps
        });
        Assert.True(approach.Success, string.Join("; ", approach.Errors));
        Assert.Contains(approach.Trajectory!.Points, p => p.ToolState?.GetValueOrDefault("width") == 0.0);

        var pickEnd = approach.Trajectory.Points[^1].JointState;
        ctx = ctx.Attach("workpiece", workpiece, tcpLocal);
        Assert.DoesNotContain(ctx.Scene.Objects, o => o.Name == "workpiece");
        checker = CollisionCheckerFactory.Create(robot, attached: ctx.Attached);

        var carry = planner.Plan(new MotionProgramRequest(
            robot,
            pickEnd,
            new MotionSegment[]
            {
                new LinSegment(placePose, stepMeters: 0.005),
                new SetToolStateSegment(open, durationSeconds: 0.1)
            },
            ctx.ToPlanningOptions(new PlanningOptions { CollisionChecker = checker, MaxJointStepRadians = 0.05 }))
        {
            InitialToolState = closed,
            ToolCapabilities = caps
        });
        Assert.True(carry.Success, string.Join("; ", carry.Errors));
        Assert.Contains(carry.Trajectory!.Points, p => p.ToolState?.GetValueOrDefault("width") == 0.085);

        var placeEnd = carry.Trajectory.Points[^1].JointState;
        var placeWorld = new Frame(homeTcp.Tcp.X - 0.05, homeTcp.Tcp.Y + 0.05, 0.08);
        ctx = ctx.Detach("workpiece", placeWorld);
        Assert.Contains(ctx.Scene.Objects, o => o.Name == "workpiece");
        checker = CollisionCheckerFactory.Create(robot, attached: ctx.Attached);

        var retract = planner.Plan(new MotionProgramRequest(
            robot,
            placeEnd,
            new MotionSegment[] { new LinSegment(retractPose, stepMeters: 0.005) },
            ctx.ToPlanningOptions(new PlanningOptions { CollisionChecker = checker, MaxJointStepRadians = 0.05 })));
        Assert.True(retract.Success, string.Join("; ", retract.Errors));
    }
}
