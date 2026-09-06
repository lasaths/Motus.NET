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

    /// <summary>Logic for GH example 10 — destack via PickPlaceCycle (attach/detach mid-program).</summary>
    [Fact]
    public void Example10_PickAndPlace_Box()
    {
        var preset = PresetLoader.LoadByModelName("UR5e");
        var robot = new RobotModel(preset, new RobotCollisionModel(
            new[] { new LinkCollisionGeometry(0, "base", CollisionObject.Sphere("base", Frame.Identity, 0.01)) },
            CollisionObject.Sphere("robotiq_2f85", new Frame(0, 0, -0.06), 0.015)));
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var planner = new IndustrialMotionPlanner(preset);

        var home = new JointState(new double[] { 0, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });
        var homeTcp = fk.ComputeTcp(home, preset.BaseFrame, preset.ToolFrame);
        var grasp = homeTcp;
        var place = new CartesianPose(new Frame(
            homeTcp.Tcp.X - 0.05, homeTcp.Tcp.Y + 0.05, homeTcp.Tcp.Z,
            homeTcp.Tcp.Qw, homeTcp.Tcp.Qx, homeTcp.Tcp.Qy, homeTcp.Tcp.Qz));

        // Brick in Motus TCP-local (0,0,-0.06) so AttachAwareChecker matches legacy Example10.
        var brickPose = Transforms.ToFrame(Transforms.Multiply(
            Transforms.FromFrame(grasp.Tcp),
            Transforms.FromFrame(new Frame(0, 0, -0.06))));
        var brick = CollisionObject.Box("b00", brickPose, 0.03, 0.03, 0.03);
        // GH example 10 plans against table only; brick enters the scene at Detach.
        var table = CollisionObject.Box("table", new Frame(homeTcp.Tcp.X, homeTcp.Tcp.Y, homeTcp.Tcp.Z - 0.15), 0.4, 0.4, 0.01);
        var scene = new CollisionScene(new[] { table });

        var open = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.085 });
        var close = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.04 });
        var caps = ToolCapabilities.Robotiq2F85;
        var checker = new RobotMeshCollisionChecker(robot);
        var opts = new PlanningOptions
        {
            CollisionScene = scene,
            CollisionChecker = checker,
            MaxJointStepRadians = 0.05
        };

        // Detach-at-place restores the brick into the gripper; without Touch, Tr is null.
        var noTouch = PickPlaceCycle.Expand(grasp, place, approachMeters: 0.05, open, close, brick);
        var failed = planner.Plan(new MotionProgramRequest(robot, home, noTouch, opts)
        {
            InitialToolState = open,
            ToolCapabilities = caps
        });
        Assert.False(failed.Success);
        Assert.Null(failed.Trajectory);

        // Explicit gripper contact is allowed only during grasp/release (GH Touch = robotiq_2f85).
        var segments = PickPlaceCycle.Expand(grasp, place, approachMeters: 0.05, open, close, brick,
            options: new PickPlaceOptions { TouchBodies = new[] { "robotiq_2f85" } });

        var result = planner.Plan(new MotionProgramRequest(robot, home, segments, opts)
        {
            InitialToolState = open,
            ToolCapabilities = caps
        });
        Assert.True(result.Success, string.Join("; ", result.Errors));
        Assert.Single(result.AttachSpans);
        Assert.Contains(result.Trajectory!.Points, p => p.ToolState?.GetValueOrDefault("width") == 0.04);
        Assert.Contains(result.Trajectory.Points, p => p.ToolState?.GetValueOrDefault("width") == 0.085);
        Assert.Equal("b00", result.AttachSpans[0].Bodies[0].Name);
    }

    [Fact]
    public void Destack_TwoBricks_AttachHides_DetachRestores_ExpandMany()
    {
        var preset = PresetLoader.LoadByModelName("UR5e");
        var robot = new RobotModel(preset);
        var open = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.085 });
        var close = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.04 });

        var grasp0 = new CartesianPose(new Frame(0.5, 0, 0.3));
        var place0 = new CartesianPose(new Frame(0.4, -0.1, 0.3));
        var grasp1 = new CartesianPose(new Frame(0.5, 0.05, 0.3));
        var place1 = new CartesianPose(new Frame(0.4, -0.1, 0.32));
        var b0 = CollisionObject.Box("b00", new Frame(0.5, 0, 0.28), 0.04, 0.02, 0.01);
        var b1 = CollisionObject.Box("b01", new Frame(0.5, 0.05, 0.28), 0.04, 0.02, 0.01);

        var segs = PickPlaceCycle.ExpandMany(
            new[] { grasp0, grasp1 },
            new[] { place0, place1 },
            new[] { b0, b1 },
            0.08,
            open,
            close);
        Assert.Equal(2, segs.OfType<AttachSegment>().Count());
        Assert.Equal(2, segs.OfType<DetachSegment>().Count());

        // Scene starts with both bricks; Attach hides grasped name; Detach restores at column pose.
        var ctx = PlanningContext.Create(robot, new CollisionScene(new[] { b0, b1 }));
        Assert.Equal(2, ctx.Scene.Objects.Count);

        var att0 = (AttachSegment)segs.First(s => s is AttachSegment);
        ctx = ctx.Attach(att0.Name, CollisionObject.Box(att0.Name, Frame.Identity, 0.04, 0.02, 0.01), att0.TcpLocal);
        Assert.DoesNotContain(ctx.Scene.Objects, o => o.Name == "b00");
        Assert.Contains(ctx.Scene.Objects, o => o.Name == "b01");
        Assert.Single(ctx.Attached);

        var det0 = (DetachSegment)segs.First(s => s is DetachSegment d && d.Name == "b00");
        ctx = ctx.Detach(det0.Name, det0.WorldPose);
        Assert.Contains(ctx.Scene.Objects, o => o.Name == "b00");
        Assert.Empty(ctx.Attached);
        var restored = ctx.Scene.Objects.First(o => o.Name == "b00");
        Assert.InRange(restored.Pose.X, place0.Tcp.X - 0.05, place0.Tcp.X + 0.05);
    }

    [Fact]
    public void PickPlaceCycle_Expand_EmitsAttachDetach()
    {
        var grasp = new CartesianPose(new Frame(0.5, 0, 0.2));
        var place = new CartesianPose(new Frame(0.4, 0.1, 0.2));
        var obj = CollisionObject.Box("brick", new Frame(0.5, 0, 0.18), 0.04, 0.02, 0.01);
        var open = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.085 });
        var close = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.04 });
        var segs = PickPlaceCycle.Expand(grasp, place, 0.08, open, close, obj);
        Assert.Contains(segs, s => s is AttachSegment a && a.Name == "brick" && a.Geometry.Name == "brick");
        Assert.Contains(segs, s => s is DetachSegment d && d.Name == "brick");
        Assert.Equal(10, segs.Count);
    }
}
