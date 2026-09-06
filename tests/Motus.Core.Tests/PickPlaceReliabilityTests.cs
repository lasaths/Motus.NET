using System.Globalization;
using System.Text.Json;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;

namespace Motus.Core.Tests;

public class PickPlaceReliabilityTests
{
    private static readonly JointState Home = new(new[] { 0d, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });
    private static readonly EndEffectorState Open = new(new Dictionary<string, double> { ["width"] = 0.085 });
    private static readonly EndEffectorState Closed = new(new Dictionary<string, double> { ["width"] = 0.04 });

    private static RobotModel Robot()
    {
        // Small explicit base and gripper volumes isolate contact policy from conservative arm envelopes.
        var collision = new RobotCollisionModel(new[] {
            new LinkCollisionGeometry(0, "base", CollisionObject.Sphere("base", Frame.Identity, 0.01))
        }, CollisionObject.Sphere("gripper", Frame.Identity, 0.015));
        return new RobotModel(PresetLoader.LoadByModelName("UR5e"), collision);
    }

    private static CartesianPose Offset(CartesianPose pose, double x, double y, double z) => new(new Frame(
        pose.Tcp.X + x, pose.Tcp.Y + y, pose.Tcp.Z + z,
        pose.Tcp.Qw, pose.Tcp.Qx, pose.Tcp.Qy, pose.Tcp.Qz));

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TwoCycles_ReleaseBeforeRetract_KeepPlacedObjectsInScene(bool sampling)
    {
        var robot = Robot();
        var fk = KinematicsResolver.CreateFkSolver(robot.Preset);
        var tcp = fk.ComputeTcp(Home, robot.Preset.BaseFrame, robot.Preset.ToolFrame);
        var grasps = new[] { Offset(tcp, 0, 0, -0.06), Offset(tcp, 0, 0.08, -0.06) };
        var places = new[] { Offset(tcp, -0.10, 0, -0.06), Offset(tcp, -0.10, 0.08, -0.06) };
        var bricks = grasps.Select((p, i) => CollisionObject.Box($"brick{i}", p.Tcp, 0.01, 0.01, 0.01)).ToArray();
        var table = CollisionObject.Box("table", new Frame(tcp.Tcp.X, tcp.Tcp.Y, tcp.Tcp.Z - 0.10), 0.3, 0.3, 0.01);
        var scene = new CollisionScene(bricks.Append(table).ToArray());
        var checker = new RecordingChecker(new RobotMeshCollisionChecker(robot));
        var segments = PickPlaceCycle.ExpandMany(grasps, places, bricks, 0.06, Open, Closed,
            options: new PickPlaceOptions { UseSamplingTransfers = sampling, TouchBodies = new[] { "gripper" } });
        var result = new IndustrialMotionPlanner(robot.Preset).Plan(new MotionProgramRequest(robot, Home, segments,
            new PlanningOptions { CollisionScene = scene, CollisionChecker = checker })
        {
            InitialToolState = Open,
            TransferPlannerFactory = c => SamplingPlanner.Create(c,
                new SamplingPlannerOptions { RandomSeed = 17, MaxIterations = 8000 })
        });
        Assert.True(result.Success, string.Join("; ", result.Errors));
        var trajectory = result.Trajectory!;
        Assert.Equal(2, trajectory.AttachSpans.Count);
        Assert.Same(trajectory.AttachSpans, result.AttachSpans);
        for (var cycle = 0; cycle < 2; cycle++)
        {
            var span = trajectory.AttachSpans[cycle];
            var release = Assert.IsType<Frame>(span.ReleaseWorldPose);
            Assert.Equal(places[cycle].Tcp.X, release.X, 6);
            var retract = trajectory.Points.Where(p => p.SegmentIndex == cycle * 10 + 9).ToArray();
            Assert.NotEmpty(retract);
            Assert.All(retract, p => Assert.True(p.TimeSeconds > span.EndSeconds));
            var atRelease = trajectory.Points.Last(p => p.TimeSeconds <= span.EndSeconds);
            var actualTcp = fk.ComputeTcp(atRelease.JointState, robot.Preset.BaseFrame, robot.Preset.ToolFrame);
            Assert.InRange(Math.Abs(actualTcp.Tcp.Z - places[cycle].Tcp.Z), 0, 1e-4);
        }
        Assert.Contains(checker.Scenes, s => s.Objects.Any(o => o.Name == "brick0" && Math.Abs(o.Pose.X - places[0].Tcp.X) < 1e-6)
            && !s.Objects.Any(o => o.Name == "brick1"));
        Assert.Contains(checker.Scenes, s => s.Objects.Count == 3 &&
            s.Objects.Where(o => o.Name.StartsWith("brick")).All(o => Math.Abs(o.Pose.X - places[0].Tcp.X) < 1e-6));
        Assert.DoesNotContain(checker.Scenes, s => s.IsPairAllowed("gripper", "brick0") && s.IsPairAllowed("gripper", "brick1"));
        Assert.Equal(grasps[0].Tcp.X, scene.Objects[0].Pose.X); // Input scene stays immutable.
    }

    [Fact]
    public void ContactException_DoesNotHideOtherObstaclesOrPersistToNextSegment()
    {
        var robot = Robot();
        var fk = KinematicsResolver.CreateFkSolver(robot.Preset);
        var pose = fk.ComputeTcp(Home, robot.Preset.BaseFrame, robot.Preset.ToolFrame);
        var brick = CollisionObject.Box("brick", pose.Tcp, 0.01, 0.01, 0.01);
        var checker = new RobotMeshCollisionChecker(robot);
        var contact = new[] { ("gripper", "brick") };
        var segments = new MotionSegment[] {
            new LinSegment(pose) { AllowedCollisionPairs = contact }, new LinSegment(pose)
        };
        var result = new IndustrialMotionPlanner(robot.Preset).Plan(new MotionProgramRequest(robot, Home, segments,
            new PlanningOptions { CollisionScene = new CollisionScene(new[] { brick }), CollisionChecker = checker }));
        Assert.False(result.Success); // Second segment no longer permits contact.
        var other = CollisionObject.Box("other", pose.Tcp, 0.01, 0.01, 0.01);
        result = new IndustrialMotionPlanner(robot.Preset).Plan(new MotionProgramRequest(robot, Home, segments.Take(1).ToArray(),
            new PlanningOptions { CollisionScene = new CollisionScene(new[] { brick, other }), CollisionChecker = checker }));
        Assert.False(result.Success);
        Assert.Contains(result.Messages, m => m.Code == PlanningMessageCodes.EndpointCollision);
    }

    [Fact]
    public void Transfer_RoutesAroundObstacle_WithAttachedBody()
    {
        var robot = Robot();
        var fk = KinematicsResolver.CreateFkSolver(robot.Preset);
        var goal = new JointState(new[] { 0.5, -1.2, 1.4, 0.2, 1.3, 0d });
        var midpoint = new JointState(Home.Positions.Zip(goal.Positions, (a, b) => (a + b) / 2).ToArray());
        var blocker = CollisionObject.Sphere("blocker", fk.ComputeTcp(midpoint, robot.Preset.BaseFrame, robot.Preset.ToolFrame).Tcp, 0.025);
        var scene = new CollisionScene(new[] { blocker });
        var robotOnly = new RobotMeshCollisionChecker(robot);
        var body = new AttachedBody("part", new Frame(0, 0, -0.025), CollisionObject.Sphere("part", Frame.Identity, 0.01));
        var checker = new AttachAwareCollisionChecker(robotOnly, fk, robot.Preset.BaseFrame, robot.Preset.ToolFrame, new[] { body });
        Assert.False(checker.SegmentCollisionFree(Home, goal, scene, 0.01));
        var result = new IndustrialMotionPlanner(robot.Preset).Plan(new MotionProgramRequest(robot, Home,
            new[] { new TransferSegment(fk.ComputeTcp(goal, robot.Preset.BaseFrame, robot.Preset.ToolFrame)) },
            new PlanningOptions { CollisionScene = scene, CollisionChecker = robotOnly, AttachedBodies = new[] { body } })
        {
            TransferPlannerFactory = c => SamplingPlanner.Create(c,
                new SamplingPlannerOptions { RandomSeed = 31, MaxIterations = 15000 })
        });
        Assert.True(result.Success, string.Join("; ", result.Errors));
        var points = result.Trajectory!.Points;
        Assert.All(points, p => Assert.True(checker.IsCollisionFree(p.JointState, scene)));
        for (var i = 1; i < points.Count; i++)
            Assert.True(checker.SegmentCollisionFree(points[i - 1].JointState, points[i].JointState, scene, 0.01));
        Assert.Single(result.AttachSpans);
        Assert.Null(result.AttachSpans[0].ReleaseWorldPose);
    }

    [Theory]
    [InlineData(RetimerAlgorithm.Totg)]
    [InlineData(RetimerAlgorithm.TotgLite)]
    [InlineData(RetimerAlgorithm.Bottleneck)]
    [InlineData(RetimerAlgorithm.SegmentTrapezoid)]
    public void Export_RetimesOnce_PreservesDwellsAndAttachmentClock(RetimerAlgorithm algorithm)
    {
        var robot = Robot();
        var q1 = new JointState(Home.Positions.Select(q => q + 0.1).ToArray());
        var q2 = new JointState(Home.Positions.Select(q => q + 0.2).ToArray());
        var body = new AttachedBody("part", Frame.Identity, CollisionObject.Box("part", Frame.Identity, 0.01, 0.02, 0.03), "part");
        var trajectory = new Trajectory(robot, new[] {
            new TrajectoryPoint(0, Home),
            new TrajectoryPoint(0.1, q1, MotionPrimitiveType.Lin, 0),
            new TrajectoryPoint(0.4, q1, MotionPrimitiveType.Set, 1, toolState: Closed),
            new TrajectoryPoint(0.5, q2, MotionPrimitiveType.Lin, 3),
            new TrajectoryPoint(1.2, q2, MotionPrimitiveType.Wait, 4),
            new TrajectoryPoint(1.4, Home, MotionPrimitiveType.Lin, 6)
        }, new[] { new AttachTimeSpan(0.4, 1.2, new[] { body }, new Frame(0.3, 0.4, 0.5)) });
        var options = new TrajectoryExportOptions { Retime = true, Retimer = new TrajectoryRetimerOptions { Algorithm = algorithm } };
        var previous = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("de-DE");
            var export = TrajectoryExport.Export(trajectory, options);
            var points = export.Trajectory.Points;
            Assert.Equal(0.3, points[2].TimeSeconds - points[1].TimeSeconds, 8);
            Assert.Equal(0.7, points[4].TimeSeconds - points[3].TimeSeconds, 8);
            var span = Assert.Single(export.Trajectory.AttachSpans);
            Assert.Equal(points[2].TimeSeconds, span.StartSeconds);
            Assert.Equal(points[4].TimeSeconds, span.EndSeconds);
            using var json = JsonDocument.Parse(export.Json);
            Assert.Equal(export.Trajectory.DurationSeconds, json.RootElement.GetProperty("durationSeconds").GetDouble());
            Assert.Equal(span.EndSeconds, json.RootElement.GetProperty("attachSpans")[0].GetProperty("endSeconds").GetDouble());
            Assert.Equal(TrajectoryExport.ToJson(trajectory, options), export.Json);
            Assert.Equal(TrajectoryExport.ToCsv(trajectory, options), export.Csv);
            var lines = export.Csv.Split('\n', StringSplitOptions.RemoveEmptyEntries);
            Assert.Contains("attachment_spans_json", lines[0]);
            Assert.Contains("\"\"startSeconds\"\"", lines[1]);
            Assert.Equal(points[^1].TimeSeconds.ToString("F6", CultureInfo.InvariantCulture), lines[^1].Split(',')[0]);
            Assert.Contains("\"\"width\"\"", export.Csv);
        }
        finally { CultureInfo.CurrentCulture = previous; }
    }

    private sealed class RecordingChecker(ICollisionChecker inner) : ICollisionChecker
    {
        public List<CollisionScene> Scenes { get; } = new();
        public bool IsCollisionFree(JointState state, CollisionScene scene)
        {
            Scenes.Add(scene);
            return inner.IsCollisionFree(state, scene);
        }
        public bool SegmentCollisionFree(JointState from, JointState to, CollisionScene scene, double step)
        {
            Scenes.Add(scene);
            return inner.SegmentCollisionFree(from, to, scene, step);
        }
    }
}
