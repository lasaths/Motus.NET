using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;
using Xunit.Abstractions;

namespace Motus.Core.Tests;

/// <summary>
/// Headless "Plan speed" benchmark: a UR5e reach with a cluster of obstacles, run through
/// CartesianLinearPathPlanner (LIN) and the managed RRT-Connect fallback. Uses
/// <see cref="PlanningDiagnostics"/> to report wall-clock time plus collision-check / IK-attempt /
/// RRT-iteration counts, so future optimizations have a before/after baseline without needing
/// Rhino. Not a strict pass/fail perf gate (CI hardware varies) — asserts only sane upper bounds
/// to catch gross regressions, and prints the numbers for manual before/after comparison.
/// </summary>
public class PlanningSpeedBenchmarkTests
{
    private readonly ITestOutputHelper _output;
    public PlanningSpeedBenchmarkTests(ITestOutputHelper output) => _output = output;
    private const double CollisionStepRadians = 0.08;

    private static RobotModel Ur5e() => new(PresetLoader.LoadByModelName("UR5e", FindResources()));

    private static CollisionScene LooseObstacleScene()
    {
        // A loose cluster near the reach path — obstacles the planner must route around,
        // not ones that immediately fail endpoints (that would fast-fail and measure nothing).
        var objects = new List<CollisionObject>();
        for (var i = 0; i < 20; i++)
        {
            var angle = i * (Math.PI * 2 / 20);
            var x = 0.35 + 0.05 * Math.Cos(angle);
            var y = 0.05 * Math.Sin(angle);
            var z = 0.3 + 0.02 * i;
            objects.Add(CollisionObject.Box($"obs{i:D2}", new Frame(x, y, z), 0.03, 0.03, 0.03));
        }
        return new CollisionScene(objects);
    }

    private static RrtBenchmarkScenario DenseRrtScenario(RobotPreset preset)
    {
        var start = new JointState(new double[6]);
        var goal = new JointState(new[] { 0.6, -0.6, 0.6, -0.6, -0.6, 0.3 });
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var objects = new List<CollisionObject>();

        var alphas = new[] { 0.45, 0.60 };
        for (var i = 0; i < alphas.Length; i++)
        {
            var tcp = fk.ComputeTcp(Interpolate(start, goal, alphas[i]), preset.BaseFrame, preset.ToolFrame).Tcp;
            var x = tcp.X - 0.02 * i;
            var zLevels = new[] { -0.035, 0.035 };
            var yOffsets = new[] { -0.10, -0.05, 0.0, 0.05, 0.10 };
            for (var zIndex = 0; zIndex < zLevels.Length; zIndex++)
            for (var yIndex = 0; yIndex < yOffsets.Length; yIndex++)
            {
                objects.Add(CollisionObject.Box(
                    $"wall{i:D2}_{zIndex}_{yIndex}",
                    new Frame(x, tcp.Y + yOffsets[yIndex], tcp.Z + zLevels[zIndex]),
                    0.022, 0.018, 0.018));
            }
        }

        return new RrtBenchmarkScenario(start, goal, new CollisionScene(objects));
    }

    [Fact]
    public void Lin_Benchmark_ReportsCountsAndStaysWithinSaneBounds()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", FindResources());
        var robot = new RobotModel(preset);
        var scene = LooseObstacleScene();
        var checker = new SphereCollisionChecker(preset);
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var goalJ = new JointState(new[] { 0.6, -1.2, 1.0, -1.4, -1.5708, 0.0 });
        var goalTcp = fk.ComputeTcp(goalJ, preset.BaseFrame, preset.ToolFrame);

        PlanningDiagnostics.Enabled = true;
        PlanningDiagnostics.Reset();
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var result = new CartesianLinearPathPlanner(preset).PlanToResult(
            new CartesianPlanningRequest(robot, home, goalTcp, new PlanningOptions
            {
                CollisionScene = scene,
                CollisionChecker = checker,
                MaxJointStepRadians = 0.05
            }, scene),
            new CartesianLinOptions(StepMeters: 0.005, ContinueOnIkFailure: true));
        sw.Stop();
        var (checks, segChecks, ikAttempts, _) = Snapshot();
        PlanningDiagnostics.Enabled = false;

        _output.WriteLine(
            $"LIN: {sw.ElapsedMilliseconds}ms, success={result.Success}, " +
            $"collisionChecks={checks}, segmentChecks={segChecks}, ikAttempts={ikAttempts}");

        // Sane upper bound, not a tight perf gate — catches gross regressions (e.g. an
        // accidental O(n^2) reintroduction), not hardware-dependent micro-variance.
        Assert.True(sw.ElapsedMilliseconds < 5000, $"LIN benchmark took {sw.ElapsedMilliseconds}ms (expected < 5000ms)");
    }

    [Theory]
    [InlineData(7)]
    [InlineData(42)]
    [InlineData(314)]
    public void Rrt_Benchmark_DenseScene_ReportsCountsAndStaysWithinSaneBounds(int seed)
    {
        var robot = Ur5e();
        var preset = robot.Preset;
        var scenario = DenseRrtScenario(preset);
        var checker = new SphereCollisionChecker(preset);
        var defaultOptions = ResolveBenchmarkOptions(seed);

        Assert.True(checker.IsCollisionFree(scenario.Start, scenario.Scene), "benchmark start should be collision-free");
        Assert.True(checker.IsCollisionFree(scenario.Goal, scenario.Scene), "benchmark goal should be collision-free");
        Assert.False(
            checker.SegmentCollisionFree(scenario.Start, scenario.Goal, scenario.Scene, CollisionStepRadians),
            "dense benchmark should block the straight-line joint segment so RRT must grow trees");

        PlanningDiagnostics.Enabled = true;
        PlanningDiagnostics.Reset();
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var result = new RrtConnectPlanner(checker, new RrtConnectOptions
        {
            MaxIterations = defaultOptions.MaxIterations,
            StepRadians = defaultOptions.StepRadians,
            GoalBias = defaultOptions.GoalBias,
            ConnectThresholdRadians = defaultOptions.ConnectThresholdRadians,
            RandomSeed = seed,
            PreferManaged = true // exercise ManagedRrtConnect deterministically in this headless benchmark
        }).Plan(new PlanningRequest(robot, scenario.Start, scenario.Goal, new PlanningOptions
        {
            CollisionScene = scenario.Scene,
            CollisionChecker = checker,
            MaxJointStepRadians = CollisionStepRadians
        }));
        sw.Stop();
        var (checks, segChecks, _, rrtIterations) = Snapshot();
        PlanningDiagnostics.Enabled = false;

        _output.WriteLine(
            $"RRT-Connect dense(seed={seed}, maxIter={defaultOptions.MaxIterations}, step={defaultOptions.StepRadians:R}, connect={defaultOptions.ConnectThresholdRadians:R}): " +
            $"{sw.ElapsedMilliseconds}ms, success={result.Success}, collisionChecks={checks}, " +
            $"segmentChecks={segChecks}, rrtIterations={rrtIterations}, waypoints={result.Trajectory?.Points.Count ?? 0}");

        Assert.True(result.Success, string.Join("; ", result.Errors));
        Assert.True(sw.ElapsedMilliseconds < 5000, $"RRT benchmark took {sw.ElapsedMilliseconds}ms (expected < 5000ms)");
    }

    private static (long checks, long segChecks, long ikAttempts, long rrtIterations) Snapshot() =>
        (PlanningDiagnostics.CollisionChecks, PlanningDiagnostics.SegmentChecks,
            PlanningDiagnostics.IkAttempts, PlanningDiagnostics.RrtIterations);

    private static RrtConnectOptions ResolveBenchmarkOptions(int seed)
    {
        var defaults = new RrtConnectOptions();
        return new RrtConnectOptions
        {
            MaxIterations = ReadEnv("MOTUS_RRT_BENCH_MAX_ITER", defaults.MaxIterations),
            StepRadians = ReadEnv("MOTUS_RRT_BENCH_STEP", defaults.StepRadians),
            GoalBias = ReadEnv("MOTUS_RRT_BENCH_GOAL_BIAS", defaults.GoalBias),
            ConnectThresholdRadians = ReadEnv("MOTUS_RRT_BENCH_CONNECT", defaults.ConnectThresholdRadians),
            RandomSeed = seed,
            PreferManaged = true
        };
    }

    private static int ReadEnv(string key, int fallback) =>
        int.TryParse(Environment.GetEnvironmentVariable(key), out var value) ? value : fallback;

    private static double ReadEnv(string key, double fallback) =>
        double.TryParse(Environment.GetEnvironmentVariable(key), out var value) ? value : fallback;

    private static JointState Interpolate(JointState start, JointState goal, double alpha)
    {
        var q = new double[start.AxisCount];
        for (var i = 0; i < q.Length; i++)
            q[i] = start.Positions[i] + alpha * (goal.Positions[i] - start.Positions[i]);
        return new JointState(q);
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

    private sealed record RrtBenchmarkScenario(JointState Start, JointState Goal, CollisionScene Scene);
}
