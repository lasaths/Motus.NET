using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>
/// Motus 2.1 ADR 0002 — sequential aerial hover → serial PickPlace (example 10 contracts).
/// </summary>
public class AerialArmPassOffTests
{
    [Fact]
    public void FreeFlyer_PlanStartToGoal_LikeExample11Hover()
    {
        // Mirrors GH 11_aerial_hover WorldXY Start/Goal (plate ≡ Motus identity).
        var aerial = AerialRobot();
        var start = new MobilityModel.HolonomicSE3(-0.5, 0.35, 0.55, 0, 0, 0);
        var goal = new MobilityModel.HolonomicSE3(0.45, -0.25, 1.05, 0, 0, 0);
        aerial = new RobotModel(
            new RobotPreset
            {
                Manufacturer = aerial.Preset.Manufacturer,
                ModelName = aerial.Preset.ModelName,
                Family = aerial.Preset.Family,
                AxisCount = 0,
                JointLimits = Array.Empty<JointLimit>(),
                BaseFrame = new BaseFrame(start.BaseFrame)
            },
            aerial.CollisionModel,
            aerial.JointNames);

        var result = new SamplingPlanner(aerial.Preset, new SamplingPlannerOptions
        {
            PreferManaged = true,
            MaxIterations = 2000,
            StepRadians = 0.2,
            ConnectThresholdRadians = 0.2,
            GoalBias = 1.0,
            RandomSeed = 11
        }).Plan(new PlanningRequest(
            aerial,
            new JointState(Array.Empty<double>()),
            new JointState(Array.Empty<double>()),
            new PlanningOptions
            {
                Mobility = goal,
                MobilityBoundsSE3 = MobilityBoundsSE3.HoverHandoff,
                CollisionChecker = FreeFlyerHullCollisionChecker.ForFreeFlyerBox(aerial.Preset.BaseFrame),
                MaxJointStepRadians = 0.05
            }));

        Assert.True(result.Success, string.Join("; ", result.Errors));
        Assert.NotNull(result.Trajectory!.Points[^1].BaseFrameOverride);
        var end = result.Trajectory.Points[^1].BaseFrameOverride!.Frame;
        Assert.True(MobilityModel.HolonomicSE3.TryFromFrame(end, out var endPose, out _), "end pose");
        Assert.InRange(endPose.X, goal.X - 0.05, goal.X + 0.05);
        Assert.InRange(endPose.Y, goal.Y - 0.05, goal.Y + 0.05);
        Assert.InRange(endPose.Z, goal.Z - 0.05, goal.Z + 0.05);
    }

    [Fact]
    public void HoverHandoff_Bounds_TighterThanDefault_RollPitch()
    {
        Assert.True(MobilityBoundsSE3.HoverHandoff.MaxPitchRadians < MobilityBoundsSE3.Default.MaxPitchRadians);
        Assert.True(MobilityBoundsSE3.HoverHandoff.MaxRollRadians < MobilityBoundsSE3.Default.MaxRollRadians);
        var ok = new MobilityModel.HolonomicSE3(0, 0, 1.0, 0, 0.2, 0);
        Assert.Null(MobilityBoundsSE3.HoverHandoff.Validate(ok, "hover"));
        var steep = new MobilityModel.HolonomicSE3(0, 0, 1.0, 0, 0.5, 0);
        Assert.Contains("pitch", MobilityBoundsSE3.HoverHandoff.Validate(steep, "hover")!, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void AerialStationHold_AppendsDwellAtLastSample()
    {
        var aerial = AerialRobot();
        var a = new MobilityModel.HolonomicSE3(0.1, 0, 0.8, 0, 0, 0);
        var traj = new Trajectory(aerial, new[]
        {
            new TrajectoryPoint(0.0, new JointState(Array.Empty<double>()), baseFrameOverride: new BaseFrame(a.BaseFrame)),
            new TrajectoryPoint(1.0, new JointState(Array.Empty<double>()), baseFrameOverride: new BaseFrame(a.BaseFrame))
        });
        var held = AerialStationHold.AppendHold(traj, 2.5);
        Assert.Equal(3, held.Points.Count);
        Assert.Equal(3.5, held.Points[^1].TimeSeconds, 3);
        Assert.Equal(held.Points[^2].BaseFrameOverride!.Frame.X, held.Points[^1].BaseFrameOverride!.Frame.X);
    }

    [Fact]
    public void FreeFlyer_WithAttached_PayloadCollidesWhileHullClear()
    {
        // Hull at z=0.5; payload hangs to z≈0.05 into a low wall; hull sphere stays clear of the wall.
        var payload = CollisionObject.Box("b00", Frame.Identity, 0.04, 0.04, 0.04);
        var attached = new AttachedBody("b00", new Frame(0, 0, -0.45), payload, "b00");
        var checker = FreeFlyerHullCollisionChecker.WithAttached(new[] { attached });
        var wall = CollisionObject.Box("wall", new Frame(1.0, 0, 0.05), 0.15, 0.15, 0.08);
        var scene = new CollisionScene(new[] { wall });
        var baseAt = new BaseFrame(new Frame(1.0, 0, 0.5));
        var empty = new JointState(Array.Empty<double>());

        Assert.True(FreeFlyerHullCollisionChecker.ForFreeFlyerBox()
            .IsCollisionFree(empty, scene, baseAt), "hull alone must clear the low wall");
        Assert.False(checker.IsCollisionFree(empty, scene, baseAt), "payload must hit the wall");
    }

    [Fact]
    public void AerialCarryAttach_HoverHandoff_Then_PickPlace_LikeExample10()
    {
        // Serial home TCP defines the shared handoff (arm reachability like Example10).
        var preset = PresetLoader.LoadByModelName("UR5e");
        var robot = new RobotModel(preset, new RobotCollisionModel(
            new[] { new LinkCollisionGeometry(0, "base", CollisionObject.Sphere("base", Frame.Identity, 0.01)) },
            CollisionObject.Sphere("robotiq_2f85", new Frame(0, 0, -0.06), 0.015)));
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var home = new JointState(new double[] { 0, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });
        var homeTcp = fk.ComputeTcp(home, preset.BaseFrame, preset.ToolFrame);
        var grasp = homeTcp;

        var brickPose = Transforms.ToFrame(Transforms.Multiply(
            Transforms.FromFrame(grasp.Tcp),
            Transforms.FromFrame(new Frame(0, 0, -0.06))));
        var brickGeom = CollisionObject.Box("b00", Frame.Identity, 0.03, 0.03, 0.03);
        // Base-local hang so world pose at handoff matches brickPose when body is 0.25 m above brick.
        var baseLocal = new Frame(0, 0, -0.25);
        var attached = new AttachedBody("b00", baseLocal, brickGeom, "b00");

        var aerial = AerialRobot();
        var handoff = new MobilityModel.HolonomicSE3(brickPose.X, brickPose.Y, brickPose.Z + 0.25, 0, 0, 0);
        // Aerial carry plans against empty scene (payload attached); table is arm-side only (example 10).
        var aerialScene = new CollisionScene();

        var aerialPlanner = new SamplingPlanner(aerial.Preset, new SamplingPlannerOptions
        {
            PreferManaged = true,
            MaxIterations = 2000,
            StepRadians = 0.2,
            ConnectThresholdRadians = 0.2,
            GoalBias = 1.0,
            RandomSeed = 42
        });
        var aerialResult = aerialPlanner.Plan(new PlanningRequest(
            aerial,
            new JointState(Array.Empty<double>()),
            new JointState(Array.Empty<double>()),
            new PlanningOptions
            {
                Mobility = handoff,
                MobilityBoundsSE3 = MobilityBoundsSE3.HoverHandoff,
                CollisionScene = aerialScene,
                CollisionChecker = FreeFlyerHullCollisionChecker.ForFreeFlyerBox(aerial.Preset.BaseFrame),
                AttachedBodies = new[] { attached },
                MaxJointStepRadians = 0.05
            }));
        Assert.True(aerialResult.Success, string.Join("; ", aerialResult.Errors));
        Assert.NotNull(aerialResult.Trajectory!.Points[^1].BaseFrameOverride);

        var held = AerialStationHold.AppendHold(aerialResult.Trajectory, 1.0);
        Assert.True(held.DurationSeconds > aerialResult.Trajectory.DurationSeconds);

        // World brick at handoff for arm PickPlace (ownership transfer).
        var brickWorld = CollisionObject.Box("b00", brickPose, 0.03, 0.03, 0.03);
        var place = new CartesianPose(new Frame(
            homeTcp.Tcp.X - 0.05, homeTcp.Tcp.Y + 0.05, homeTcp.Tcp.Z,
            homeTcp.Tcp.Qw, homeTcp.Tcp.Qx, homeTcp.Tcp.Qy, homeTcp.Tcp.Qz));
        var table = CollisionObject.Box("table", new Frame(homeTcp.Tcp.X, homeTcp.Tcp.Y, homeTcp.Tcp.Z - 0.15), 0.4, 0.4, 0.01);
        var armScene = new CollisionScene(new[] { table });
        var open = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.085 });
        var close = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.04 });

        var segments = PickPlaceCycle.Expand(grasp, place, approachMeters: 0.05, open, close, brickWorld,
            options: new PickPlaceOptions { TouchBodies = new[] { "robotiq_2f85" } });
        var armResult = new IndustrialMotionPlanner(preset).Plan(
            new MotionProgramRequest(robot, home, segments, new PlanningOptions
            {
                CollisionScene = armScene,
                CollisionChecker = new RobotMeshCollisionChecker(robot),
                MaxJointStepRadians = 0.05
            })
            {
                InitialToolState = open,
                ToolCapabilities = ToolCapabilities.Robotiq2F85
            });

        Assert.True(armResult.Success, string.Join("; ", armResult.Errors));
        Assert.Single(armResult.AttachSpans);
        Assert.Equal("b00", armResult.AttachSpans[0].Bodies[0].Name);
        Assert.Contains(armResult.Trajectory!.Points, p => p.ToolState?.GetValueOrDefault("width") == 0.04);
        Assert.Contains(armResult.Trajectory.Points, p => p.ToolState?.GetValueOrDefault("width") == 0.085);

        // Family honesty: aerial export is bodyPose, not MoveJ.
        var json = TrajectoryExport.ToJson(held);
        Assert.Contains("bodyPose", json, StringComparison.Ordinal);
        Assert.Contains("not_ur_movej", json, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void AerialHandoff_Then_ExpandMany_FinishesMiniTower_LikeExample10()
    {
        // Drone delivers the final brick; arm builds stack ×3 then picks from drone (tower ×4).
        var preset = PresetLoader.LoadByModelName("UR5e");
        var robot = new RobotModel(preset, new RobotCollisionModel(
            new[] { new LinkCollisionGeometry(0, "base", CollisionObject.Sphere("base", Frame.Identity, 0.01)) },
            CollisionObject.Sphere("robotiq_2f85", new Frame(0, 0, -0.06), 0.015)));
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var home = new JointState(new double[] { 0, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });
        var homeTcp = fk.ComputeTcp(home, preset.BaseFrame, preset.ToolFrame);
        var q = homeTcp.Tcp;

        CartesianPose GraspAt(double dx, double dy, double dz) =>
            new(new Frame(q.X + dx, q.Y + dy, q.Z + dz, q.Qw, q.Qx, q.Qy, q.Qz));

        // Grasps 0–2 = stack; grasp 3 = handoff (drone-delivered final brick).
        var grasps = new[]
        {
            GraspAt(0, 0.05, 0),
            GraspAt(0, 0, 0.025),
            GraspAt(0, 0.05, 0.025),
            GraspAt(0, 0, 0),
        };
        var places = new[]
        {
            GraspAt(-0.12, 0, 0),
            GraspAt(-0.12, 0, 0.025),
            GraspAt(-0.12, 0.05, 0),
            GraspAt(-0.12, 0.05, 0.025)
        };
        var bricks = new CollisionObject[4];
        for (var i = 0; i < 4; i++)
        {
            var g = grasps[i].Tcp;
            var pose = Transforms.ToFrame(Transforms.Multiply(
                Transforms.FromFrame(g),
                Transforms.FromFrame(new Frame(0, 0, -0.06))));
            bricks[i] = CollisionObject.Box($"b{i:00}", pose, 0.03, 0.03, 0.03);
        }

        var aerial = AerialRobot();
        var handoffBrick = bricks[^1];
        var handoff = new MobilityModel.HolonomicSE3(
            handoffBrick.Pose.X, handoffBrick.Pose.Y, handoffBrick.Pose.Z + 0.25, 0, 0, 0);
        var aerialResult = new SamplingPlanner(aerial.Preset, new SamplingPlannerOptions
        {
            PreferManaged = true,
            MaxIterations = 2000,
            StepRadians = 0.2,
            ConnectThresholdRadians = 0.2,
            GoalBias = 1.0,
            RandomSeed = 7
        }).Plan(new PlanningRequest(
            aerial,
            new JointState(Array.Empty<double>()),
            new JointState(Array.Empty<double>()),
            new PlanningOptions
            {
                Mobility = handoff,
                MobilityBoundsSE3 = MobilityBoundsSE3.HoverHandoff,
                CollisionChecker = FreeFlyerHullCollisionChecker.ForFreeFlyerBox(),
                MaxJointStepRadians = 0.05
            }));
        Assert.True(aerialResult.Success, string.Join("; ", aerialResult.Errors));

        var open = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.085 });
        var close = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.04 });
        var segs = PickPlaceCycle.ExpandMany(
            grasps, places, bricks, 0.05, open, close,
            options: new PickPlaceOptions { TouchBodies = new[] { "robotiq_2f85" } });
        Assert.Equal(4, segs.OfType<AttachSegment>().Count());

        var armResult = new IndustrialMotionPlanner(preset).Plan(
            new MotionProgramRequest(robot, home, segs, new PlanningOptions
            {
                // Example 10: plan ColScene empty — bricks are attach payloads, not scene obstacles.
                CollisionScene = new CollisionScene(),
                CollisionChecker = new RobotMeshCollisionChecker(robot),
                MaxJointStepRadians = 0.05
            })
            {
                InitialToolState = open,
                ToolCapabilities = ToolCapabilities.Robotiq2F85
            });
        Assert.True(armResult.Success, string.Join("; ", armResult.Errors));
        Assert.Equal(4, armResult.AttachSpans.Count);
        Assert.Contains(armResult.Trajectory!.Points, p => p.ToolState?.GetValueOrDefault("width") == 0.04);
        Assert.Contains(armResult.Trajectory.Points, p => p.ToolState?.GetValueOrDefault("width") == 0.085);
    }

    [Fact]
    public void GhExample11_HomeTcp_PickPlace_OneBrick_Ur10e()
    {
        var path = "/Users/lasaths/Documents/GitHub/Motus.Grasshopper/resources/robots/ur10e_robotiq/ur10e_robotiq.urdf";
        var urdf = UrdfRobotLoader.Load(path, new UrdfLoadOptions { BaseLink = "base_link", TipLink = "tool0", ModelName = "UR10e" });
        var robot = urdf.ToModel();
        // Classic UR elbow-up home (reachable LIN neighborhood) — not the GH -77/-31/-77 wrist set.
        var home = new JointState(new[] { 0.0, -Math.PI / 2, Math.PI / 2, 0.0, Math.PI / 2, 0.0 });
        var fk = KinematicsResolver.CreateFkSolver(robot.Preset);
        var homeTcp = fk.ComputeTcp(home, robot.Preset.BaseFrame, robot.Preset.ToolFrame);
        var grasp = homeTcp;
        var place = new CartesianPose(new Frame(
            homeTcp.Tcp.X - 0.05, homeTcp.Tcp.Y + 0.05, homeTcp.Tcp.Z,
            homeTcp.Tcp.Qw, homeTcp.Tcp.Qx, homeTcp.Tcp.Qy, homeTcp.Tcp.Qz));
        var brickPose = Transforms.ToFrame(Transforms.Multiply(
            Transforms.FromFrame(grasp.Tcp),
            Transforms.FromFrame(new Frame(0, 0, -0.06))));
        var brick = CollisionObject.Box("b00", brickPose, 0.04, 0.02, 0.01);
        var open = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.085 });
        var close = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.04 });
        var segs = PickPlaceCycle.Expand(grasp, place, 0.05, open, close, brick,
            options: new PickPlaceOptions { TouchBodies = new[] { "robotiq_2f85" } });
        var planner = new IndustrialMotionPlanner(robot.Preset);
        var result = planner.Plan(new MotionProgramRequest(robot, home, segs, new PlanningOptions
        {
            CollisionScene = new CollisionScene(),
            CollisionChecker = new RobotMeshCollisionChecker(robot),
            MaxJointStepRadians = 0.05
        })
        {
            InitialToolState = open,
            ToolCapabilities = ToolCapabilities.Robotiq2F85
        });
        Assert.True(result.Success, string.Join("; ", result.Errors));
    }

    [Fact]
    public void FreeFlyerUrdf_Load_BaseEqualsTip_AxisCountZero()
    {
        var path = Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory, "..", "..", "..", "..", "..",
            "Motus.NET", "tests", "fixtures", "aerial", "free_flyer_box.urdf"));
        if (!File.Exists(path))
            path = "/Users/lasaths/Documents/GitHub/Motus.NET/tests/fixtures/aerial/free_flyer_box.urdf";
        Assert.True(File.Exists(path), path);
        var urdf = UrdfRobotLoader.Load(path, new UrdfLoadOptions
        {
            BaseLink = "body",
            TipLink = "body",
            ModelName = "free_flyer_box"
        });
        Assert.Equal(0, urdf.Preset.AxisCount);
        Assert.Empty(urdf.JointNames);
        Assert.NotNull(urdf.CollisionModel);
        Assert.NotEmpty(urdf.CollisionModel!.Links);
    }

    private static RobotModel AerialRobot() =>
        new(new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = "free_flyer_passoff",
            Family = Units.AerialFamily,
            AxisCount = 0,
            JointLimits = Array.Empty<JointLimit>(),
            BaseFrame = BaseFrame.Identity
        });
}
