using System.Diagnostics;
using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;
using Xunit.Abstractions;

namespace Motus.Core.Tests;

/// <summary>
/// Wall-clock plan times for every implemented planner × the motion types it actually handles.
/// Sampling IDs with no backend in this build are reported as skip, not fail.
/// </summary>
public sealed class PlannerMotionSpeedTests
{
    private const double DeterministicBudgetMs = 2_000;
    private const double SamplingBudgetMs = 5_000;
    private const double LeggedBudgetMs = 8_000;

    private static string ResourcesRoot =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

    private readonly ITestOutputHelper _output;

    public PlannerMotionSpeedTests(ITestOutputHelper output) => _output = output;

    [Fact]
    public void Report_PlanMs_EveryImplementedPlannerAndMotion()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var fk = new DhForwardKinematics(preset);
        var start = new JointState([0.0, -0.5, 1.0, -1.0, 0.0, 0.0]);
        var ptpGoal = new JointState([0.35, -0.7, 1.15, -1.15, 0.12, 0.2]);
        var startTcp = fk.ComputeTcp(start, preset.BaseFrame, preset.ToolFrame);
        var linGoal = Offset(startTcp, 0.04, 0, 0);
        var circVia = Offset(linGoal, 0.012, 0.008, 0);
        var circGoal = Offset(linGoal, 0, 0.016, 0);

        // JIT so the first timed row is not a cold-start lie.
        _ = new JointLinearPlanner().Plan(new PlanningRequest(robot, start, ptpGoal));
        _ = new CartesianLinearPathPlanner(preset).PlanToResult(
            new CartesianPlanningRequest(robot, start, linGoal, new PlanningOptions()), 0.005);

        var rows = new List<Row>();

        rows.Add(Time("JointLinearPlanner", "ptp", DeterministicBudgetMs, required: true, () =>
            new JointLinearPlanner().Plan(new PlanningRequest(robot, start, ptpGoal))));

        rows.Add(Time("CartesianLinearPlanner", "lin-ik-ptp", DeterministicBudgetMs, required: true, () =>
            new CartesianLinearPlanner(preset).Plan(
                new CartesianPlanningRequest(robot, start, linGoal, new PlanningOptions()))));

        rows.Add(Time("CartesianLinearPathPlanner", "lin", DeterministicBudgetMs, required: true, () =>
            new CartesianLinearPathPlanner(preset).PlanToResult(
                new CartesianPlanningRequest(robot, start, linGoal, new PlanningOptions()), 0.005)));

        var industrial = new IndustrialMotionPlanner(preset);
        rows.Add(Time("IndustrialMotionPlanner", "ptp", DeterministicBudgetMs, required: true, () =>
            industrial.Plan(new MotionProgramRequest(robot, start, [new PtpSegment(ptpGoal)]))));
        rows.Add(Time("IndustrialMotionPlanner", "lin", DeterministicBudgetMs, required: true, () =>
            industrial.Plan(new MotionProgramRequest(robot, start, [new LinSegment(linGoal, stepMeters: 0.005)]))));
        rows.Add(Time("IndustrialMotionPlanner", "circ", DeterministicBudgetMs, required: true, () =>
            industrial.Plan(new MotionProgramRequest(robot, start, [new CircSegment(circVia, circGoal, arcSamples: 12)]))));
        rows.Add(Time("IndustrialMotionPlanner", "set", DeterministicBudgetMs, required: true, () =>
            industrial.Plan(new MotionProgramRequest(
                robot,
                start,
                [new SetToolStateSegment(new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.0 }), 0.1)])
                { ToolCapabilities = ToolCapabilities.Robotiq2F85 })));
        rows.Add(Time("IndustrialMotionPlanner", "wait", DeterministicBudgetMs, required: true, () =>
            industrial.Plan(new MotionProgramRequest(robot, start, [new WaitSegment(0.1)]))));

        var sampleReq = new PlanningRequest(robot, start, ptpGoal);
        foreach (var id in Enum.GetValues<SamplingPlannerId>())
        {
            var desc = SamplingPlannerRegistry.Resolve(id);
            var name = desc?.ShortName ?? id.ToString();
            if (desc is null || (!desc.NativeSupported && !desc.ManagedSupported))
            {
                rows.Add(Row.Skip($"Sampling:{name}", "ptp", desc?.UnavailableReason ?? "not registered"));
                continue;
            }

            var preferManaged = desc.ManagedSupported && !desc.NativeSupported;
            rows.Add(Time($"Sampling:{name}", "ptp", SamplingBudgetMs, required: id == SamplingPlannerId.RrtConnect, () =>
                SamplingPlanner.Create(preset, new SamplingPlannerOptions
                {
                    PlannerId = id,
                    MaxIterations = 2000,
                    MaxPlanTimeSeconds = 0.5,
                    RandomSeed = 42,
                    PreferManaged = preferManaged
                }).Plan(sampleReq)));
        }

        var stewart = StewartRobot.CreateClassic();
        var mid = 0.5 * (stewart.Platform.StrokeLimits[0].Min + stewart.Platform.StrokeLimits[0].Max);
        rows.Add(Time("StewartCartesianPathPlanner", "lin", DeterministicBudgetMs, required: true, () =>
            stewart.PathPlanner.PlanToResult(
                new CartesianPose(new Frame(0, 0, mid)),
                new CartesianPose(new Frame(0.015, 0, mid)),
                stepMeters: 0.005)));

        var mech = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12).ToMechanism();
        var gaitPath = new[] { new Vec3(0, 0, 0), new Vec3(0.4, 0, 0) };
        rows.Add(Time("LeggedGait.PlanBodyPath", "gait", LeggedBudgetMs, required: true, () =>
            LeggedGait.PlanBodyPath(mech, gaitPath)));

        _output.WriteLine($"{"planner",-32} {"motion",-12} {"ms",8} {"pts",6}  status");
        foreach (var row in rows)
            _output.WriteLine($"{row.Planner,-32} {row.Motion,-12} {row.Ms,8:F1} {row.Points,6}  {row.Status}");

        foreach (var row in rows.Where(r => r.Required))
        {
            Assert.True(row.Ok, $"{row.Planner}/{row.Motion}: {row.Status}");
            Assert.True(row.Ms < row.BudgetMs, $"{row.Planner}/{row.Motion} {row.Ms:F1}ms exceeded {row.BudgetMs:F0}ms");
        }
    }

    private static CartesianPose Offset(CartesianPose pose, double dx, double dy, double dz) =>
        new(new Frame(
            pose.Tcp.X + dx, pose.Tcp.Y + dy, pose.Tcp.Z + dz,
            pose.Tcp.Qw, pose.Tcp.Qx, pose.Tcp.Qy, pose.Tcp.Qz));

    private static Row Time(string planner, string motion, double budgetMs, bool required, Func<PlanningResult> plan)
    {
        var sw = Stopwatch.StartNew();
        PlanningResult result;
        try
        {
            result = plan();
        }
        catch (Exception ex)
        {
            sw.Stop();
            return new Row(planner, motion, sw.Elapsed.TotalMilliseconds, 0, required, budgetMs, false,
                $"exception: {ex.GetType().Name}: {ex.Message}");
        }

        sw.Stop();
        var pts = result.Trajectory?.Points.Count ?? 0;
        if (!result.Success)
            return new Row(planner, motion, sw.Elapsed.TotalMilliseconds, pts, required, budgetMs, false,
                result.Errors.Count > 0 ? string.Join("; ", result.Errors) : "failed");
        return new Row(planner, motion, sw.Elapsed.TotalMilliseconds, pts, required, budgetMs, true, "ok");
    }

    private readonly record struct Row(
        string Planner,
        string Motion,
        double Ms,
        int Points,
        bool Required,
        double BudgetMs,
        bool Ok,
        string Status)
    {
        public static Row Skip(string planner, string motion, string reason) =>
            new(planner, motion, 0, 0, false, 0, true, $"skip: {reason}");
    }
}
