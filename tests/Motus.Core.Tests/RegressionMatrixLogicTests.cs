using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>
/// CI-runnable Motus.NET logic for Motus.Grasshopper docs/regression-matrix.md rows that
/// cannot exercise Rhino GUI on GitHub-hosted runners. Rhino wiring still needs local verify-qa.
/// </summary>
public class RegressionMatrixLogicTests
{
    private static readonly JointState UrHome =
        new(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });

    private static string ResourcesRoot =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

    [Fact]
    public void Serial_Ur5e_PlaneLin_JointLinear_AndCollisionRrt()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var goalTcp = fk.ComputeTcp(goalJ, preset.BaseFrame, preset.ToolFrame);

        var lin = new CartesianLinearPathPlanner(preset).PlanToResult(
            new CartesianPlanningRequest(robot, UrHome, goalTcp, new PlanningOptions { MaxJointStepRadians = 0.05 }),
            new CartesianLinOptions(StepMeters: 0.005, ContinueOnIkFailure: false));
        Assert.True(lin.Success, string.Join("; ", lin.Errors));

        var joint = new JointLinearPlanner().Plan(new PlanningRequest(robot, UrHome, goalJ));
        Assert.True(joint.Success, string.Join("; ", joint.Errors));

        var checker = new SphereCollisionChecker(preset);
        CollisionScene? blocking = null;
        foreach (var pt in lin.Trajectory!.Points)
        {
            var origins = fk.ComputeLinkOrigins(pt.JointState.Positions, preset.BaseFrame.Frame);
            foreach (var origin in origins)
            {
                var trial = new CollisionScene(new[] { CollisionObject.Sphere("block", origin, 0.12) });
                if (checker.IsCollisionFree(UrHome, trial)
                    && checker.IsCollisionFree(goalJ, trial)
                    && !checker.IsCollisionFree(pt.JointState, trial))
                {
                    blocking = trial;
                    break;
                }
            }
            if (blocking is not null) break;
        }
        Assert.NotNull(blocking);

        var blocked = new CartesianLinearPathPlanner(preset).PlanToResult(
            new CartesianPlanningRequest(robot, UrHome, goalTcp,
                new PlanningOptions { CollisionScene = blocking, CollisionChecker = checker, MaxJointStepRadians = 0.05 },
                blocking),
            new CartesianLinOptions(StepMeters: 0.005, ContinueOnIkFailure: false));
        Assert.False(blocked.Success);

        var reach = new CartesianGoalSolver().TryReach(
            robot, goalTcp, CartesianGoalSolver.EnumerateDefaultSeeds(UrHome, robot));
        Assert.True(reach.Success, string.Join("; ", reach.Errors));
        var rrt = SamplingPlanner.Create(checker, new SamplingPlannerOptions
        {
            PlannerId = SamplingPlannerId.RrtConnect,
            MaxIterations = 12000,
            MaxPlanTimeSeconds = 30,
            RandomSeed = 42,
            GoalBias = 0.08,
            StepRadians = 0.12
        }).Plan(new PlanningRequest(robot, UrHome, reach.Solution!,
            new PlanningOptions { CollisionScene = blocking, CollisionChecker = checker, MaxJointStepRadians = 0.05 }));
        Assert.True(rrt.Success, string.Join("; ", rrt.Errors));
    }

    [Fact]
    public void Stewart_TcpLin_CollisionNamed_AndExportMetersNotMoveJ()
    {
        var robot = StewartRobot.CreateClassic();
        Assert.True(Units.IsStewart(robot.Model.Preset));
        Assert.All(robot.Model.Preset.JointLimits, l => Assert.Equal(JointCoordinateUnit.Meters, l.Unit));

        var mid = 0.5 * (robot.Platform.StrokeLimits[0].Min + robot.Platform.StrokeLimits[0].Max);
        var start = new CartesianPose(new Frame(0, 0, mid));
        var goal = new CartesianPose(new Frame(0.015, 0, mid));
        var free = robot.PathPlanner.PlanToResult(start, goal, stepMeters: 0.005);
        Assert.True(free.Success, string.Join("; ", free.Errors));

        var scene = new CollisionScene(new[]
        {
            CollisionObject.Sphere("tcp_block", new Frame(0.0075, 0, mid), 0.01)
        });
        var collided = robot.PathPlanner.PlanToResult(
            start, goal, stepMeters: 0.005,
            planningOptions: new PlanningOptions
            {
                CollisionScene = scene,
                CollisionChecker = new StewartCollisionChecker(robot.Platform),
                MaxJointStepRadians = 0.02
            });
        Assert.False(collided.Success);
        Assert.Contains(collided.Messages, m =>
            m.Code == PlanningMessageCodes.EndpointCollision ||
            m.Code == PlanningMessageCodes.PathCollision);

        // Leg-length RRT stays in meters (GH Waypoints/Export warn; JSON units say meters).
        var home = robot.Platform.HomeLengths();
        var ik = robot.InverseKinematics.TrySolveDetailed(goal);
        Assert.True(ik.Success, ik.ToString());
        var rrt = new SamplingPlanner(robot.Model.Preset, new SamplingPlannerOptions
        {
            PreferManaged = true,
            MaxIterations = 5000,
            StepRadians = 0.04,
            ConnectThresholdRadians = 0.04,
            GoalBias = 0.5,
            RandomSeed = 7
        }).Plan(new PlanningRequest(robot.Model, home, ik.JointState!));
        Assert.True(rrt.Success, string.Join("; ", rrt.Errors));
        Assert.All(rrt.Trajectory!.Points, p =>
            Assert.All(p.JointState.Positions, q => Assert.True(double.IsFinite(q))));

        var json = TrajectoryExport.ToJson(free.Trajectory!);
        Assert.Contains("\"jointCoordinates\": \"meters\"", json, StringComparison.Ordinal);
        Assert.Contains("\"legLengths\": \"meters\"", json, StringComparison.Ordinal);
        Assert.Contains("stewart", json, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void HolonomicSE2_JointGoalsRouteThroughMobilityPlanningOptions()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var goalJ = new JointState(new[] { 0.3, -1.4, 1.4, -1.5, -1.5708, 0.0 });
        var targetBase = new MobilityModel.HolonomicSE2(0.45, -0.2, 0.35);
        var result = SamplingPlanner.Create(checker, new SamplingPlannerOptions
        {
            PreferManaged = true,
            MaxIterations = 8000,
            RandomSeed = 11,
            StepRadians = 0.1,
            GoalBias = 0.15
        }).Plan(new PlanningRequest(robot, UrHome, goalJ,
            new PlanningOptions { Mobility = targetBase, MaxJointStepRadians = 0.05 }));
        Assert.True(result.Success, string.Join("; ", result.Errors));
        Assert.Contains(result.Warnings, w =>
            w.Contains("HolonomicSE2", StringComparison.OrdinalIgnoreCase) ||
            w.Contains(MobilityMethodRefs.LaVallePlanningAlgorithmsUrl, StringComparison.Ordinal));
    }

    [Fact]
    public void Tool_CustomWidthSchema_AndRobotiqCap()
    {
        var custom = ToolCapabilities.WidthSchema(0, 0.12, 0.12);
        Assert.Contains(custom.Parameters, p => p.Name == "width" && Math.Abs(p.Max - 0.12) < 1e-12);
        Assert.Same(ToolCapabilities.Robotiq2F85, ToolCapabilities.Robotiq2F85);

        var binding = ToolParameterBinding.WidthBinding("j_jaw", openWidthMeters: 0.12, closedDriverValue: 1.0);
        var q = new double[2];
        var n = ToolParameterBinding.ApplyInto(
            custom, new EndEffectorState(new Dictionary<string, double> { ["width"] = 0 }),
            new[] { "other", "j_jaw" }, q, new[] { binding });
        Assert.Equal(1, n);
        Assert.Equal(1.0, q[1], 9);
    }

    [Fact]
    public void CustomSerial_FarPlane_NamesIkFailure()
    {
        var tree = SerialKinematicTrees.FromLengths(new[] { 0.3, 0.3, 0.2, 0.15, 0.1, 0.08 }, rail: false);
        var tip = tree.ExtractSerialTip("base_link", "tool0");
        var limits = new List<JointLimit>(tip.Chain.Joints.Length);
        foreach (var name in tip.JointNames)
        {
            var j = tree.Joints.First(jj => string.Equals(jj.Name, name, StringComparison.OrdinalIgnoreCase));
            limits.Add(new JointLimit(j.Lower, j.Upper, Math.PI, Math.PI * 2));
        }

        var preset = new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = "serial_arm",
            Family = "serial",
            AxisCount = tip.Chain.Joints.Length,
            JointLimits = limits,
            BaseFrame = BaseFrame.Identity,
            ToolFrame = ToolFrame.Identity,
        };
        var robot = new RobotModel(preset);
        var home = new JointState(new double[preset.AxisCount]);
        var far = new CartesianPose(new Frame(50, 0, 0));
        var result = new CartesianGoalSolver().TryReach(
            robot, far, CartesianGoalSolver.EnumerateDefaultSeeds(home, robot), tip.Chain);
        Assert.False(result.Success);
        Assert.Contains(result.Errors, e =>
            e.Contains("IK NoConvergence", StringComparison.Ordinal) ||
            e.Contains("IK SingularJacobian", StringComparison.Ordinal) ||
            e.Contains("IK InvalidInput", StringComparison.Ordinal));
    }

    [Fact]
    public void Legged_PlanBodyPath_FullDriverGait()
    {
        var mech = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12).ToMechanism();
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.4, 0, 0) };
        var plan = LeggedGait.PlanBodyPath(mech, path);
        Assert.True(plan.Success, string.Join("; ", plan.Errors));
        Assert.NotNull(plan.Trajectory);
        Assert.Equal(18, plan.Trajectory!.Robot.Preset.AxisCount);
        Assert.True(Units.IsLegged(plan.Trajectory.Robot.Preset));
        Assert.Contains(plan.Warnings, w => w.Contains(LeggedGait.PlanBodyPathHonestyWarning, StringComparison.Ordinal));
    }

    [Fact]
    public void Export_RetimeBool_DefaultsRetimerToTotgLite()
    {
        var opts = new TrajectoryExportOptions { Retime = true };
        Assert.True(opts.Retime);
        Assert.Null(opts.Retimer);

        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var traj = new JointLinearPlanner().Plan(
            new PlanningRequest(robot, UrHome, new JointState(new[] { 0.2, -1.4, 1.4, -1.5, -1.5708, 0.0 }))).Trajectory!;

        var defaulted = TrajectoryExport.Prepare(traj, opts);
        var explicitLite = TrajectoryExport.Prepare(traj, new TrajectoryExportOptions
        {
            Retime = true,
            Retimer = new TrajectoryRetimerOptions { Algorithm = RetimerAlgorithm.TotgLite }
        });
        Assert.Equal(explicitLite.DurationSeconds, defaulted.DurationSeconds, 9);
        Assert.True(defaulted.DurationSeconds >= traj.DurationSeconds);
    }

    [Fact]
    public void Example10_PickPlace_TouchContract_OpenCloseWidths()
    {
        var preset = PresetLoader.LoadByModelName("UR5e");
        var robot = new RobotModel(preset, new RobotCollisionModel(
            new[] { new LinkCollisionGeometry(0, "base", CollisionObject.Sphere("base", Frame.Identity, 0.01)) },
            CollisionObject.Sphere("robotiq_2f85", new Frame(0, 0, -0.06), 0.015)));
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var planner = new IndustrialMotionPlanner(preset);
        var home = new JointState(new double[] { 0, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });
        var homeTcp = fk.ComputeTcp(home, preset.BaseFrame, preset.ToolFrame);
        var place = new CartesianPose(new Frame(
            homeTcp.Tcp.X - 0.05, homeTcp.Tcp.Y + 0.05, homeTcp.Tcp.Z,
            homeTcp.Tcp.Qw, homeTcp.Tcp.Qx, homeTcp.Tcp.Qy, homeTcp.Tcp.Qz));
        var brickPose = Transforms.ToFrame(Transforms.Multiply(
            Transforms.FromFrame(homeTcp.Tcp),
            Transforms.FromFrame(new Frame(0, 0, -0.06))));
        var brick = CollisionObject.Box("b00", brickPose, 0.03, 0.03, 0.03);
        var table = CollisionObject.Box("table", new Frame(homeTcp.Tcp.X, homeTcp.Tcp.Y, homeTcp.Tcp.Z - 0.15), 0.4, 0.4, 0.01);
        var open = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.085 });
        var close = new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.04 });
        var opts = new PlanningOptions
        {
            CollisionScene = new CollisionScene(new[] { table }),
            CollisionChecker = new RobotMeshCollisionChecker(robot),
            MaxJointStepRadians = 0.05
        };
        var segments = PickPlaceCycle.Expand(homeTcp, place, approachMeters: 0.05, open, close, brick,
            options: new PickPlaceOptions { TouchBodies = new[] { "robotiq_2f85" } });
        var result = planner.Plan(new MotionProgramRequest(robot, home, segments, opts)
        {
            InitialToolState = open,
            ToolCapabilities = ToolCapabilities.Robotiq2F85
        });
        Assert.True(result.Success, string.Join("; ", result.Errors));
        Assert.Single(result.AttachSpans);
        Assert.Contains(result.Trajectory!.Points, p => p.ToolState?.GetValueOrDefault("width") == 0.085);
        Assert.Contains(result.Trajectory.Points, p => p.ToolState?.GetValueOrDefault("width") == 0.04);
    }
}
