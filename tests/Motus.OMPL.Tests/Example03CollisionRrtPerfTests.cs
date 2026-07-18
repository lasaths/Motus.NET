using System.Diagnostics;
using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;
using Xunit.Abstractions;

namespace Motus.OMPL.Tests;

/// <summary>
/// Perf + planning regression from Motus.Grasshopper <c>examples/03_collision_rrt.ghx</c>
/// and <c>scripts/generate-examples.mjs</c> (<c>graph03</c>, <c>GOAL_JOINTS</c>, RRT Settings, ColSphere).
/// <para>
/// Measured before/after managed mesh rewrite (UR10e Robotiq STL links, same start/goal/RRT knobs):
/// RRT ~915ms / ~1620MB → ~15ms / ~3.5MB; IsCollisionFree ~2080µs &amp; ~4MB/check → ~19µs &amp; ~12KB/check.
/// </para>
/// </summary>
public sealed class Example03CollisionRrtPerfTests
{
    private readonly ITestOutputHelper _output;

    public Example03CollisionRrtPerfTests(ITestOutputHelper output) => _output = output;

    // Motus Joint State persistent values in 03_collision_rrt.ghx
    private static readonly double[] GoalJoints = [1.2, -1.0, 1.2, -1.6, -1.5708, 0.0];

    // Motus UR10e Robotiq home (HomePoseLookup ready pose)
    private static readonly double[] StartJoints =
    [
        0.0,
        -Math.PI / 2,
        Math.PI / 2,
        -Math.PI / 2,
        0.0,
        0.0
    ];

    // generate-examples.mjs ColSphere defaults (graph03)
    private static readonly Frame CanonicalSphereCenter = new(0.35, 0.15, 0.35);
    private const double CanonicalSphereRadius = 0.12;

    // Saved 03_collision_rrt.ghx Radius persistent (Center comes from Populate 3D — not fixed)
    private const double SavedGhxSphereRadius = 0.1;

    // Motus RRT Settings in 03_collision_rrt.ghx / generate-examples defaults
    private const int MaxIter = 4000;
    private const double TimeLimitSeconds = 30;
    private const double GoalBias = 0.08;
    private const double StepRadians = 0.12;

    [Fact]
    public void Example03_RrtConnect_PlansAroundBlockingSphere()
    {
        var robot = LoadUr10eRobotiq();
        var checker = CollisionCheckerFactory.Create(robot);
        var start = new JointState(StartJoints);
        var goal = new JointState(GoalJoints);
        var scene = ResolveBlockingScene(checker, robot, start, goal);

        _output.WriteLine(
            $"Scene: {DescribeScene(scene)} links={robot.CollisionModel?.Links.Count ?? 0} " +
            $"checker={checker.GetType().Name}");

        var opts = new PlanningOptions
        {
            CollisionScene = scene,
            CollisionChecker = checker,
            MaxJointStepRadians = StepRadians
        };
        var planner = SamplingPlanner.Create(robot.Preset, new SamplingPlannerOptions
        {
            PlannerId = SamplingPlannerId.RrtConnect,
            MaxIterations = MaxIter,
            MaxPlanTimeSeconds = TimeLimitSeconds,
            GoalBias = GoalBias,
            StepRadians = StepRadians,
            RandomSeed = 11,
            PreferManaged = true
        });

        var before = GC.GetAllocatedBytesForCurrentThread();
        var sw = Stopwatch.StartNew();
        var result = planner.Plan(new PlanningRequest(robot, start, goal, opts));
        sw.Stop();
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        _output.WriteLine(
            $"Example03 RRT: success={result.Success} ms={sw.ElapsedMilliseconds} " +
            $"allocMB={allocated / (1024.0 * 1024.0):F2} waypoints={result.Trajectory?.Points.Count ?? 0} " +
            $"errors={string.Join("; ", result.Errors)}");

        Assert.True(result.Success, string.Join("; ", result.Errors));
        Assert.NotNull(result.Trajectory);
        Assert.True(result.Trajectory!.Points.Count >= 2);
        Assert.True(sw.Elapsed.TotalSeconds < TimeLimitSeconds);
    }

    [Fact]
    public void Example03_IsCollisionFree_HotPathMetrics()
    {
        var robot = LoadUr10eRobotiq();
        var checker = CollisionCheckerFactory.Create(robot);
        var start = new JointState(StartJoints);
        var goal = new JointState(GoalJoints);
        var scene = ResolveBlockingScene(checker, robot, start, goal);

        var samples = new List<JointState>(24);
        for (var i = 0; i < 20; i++)
            samples.Add(Interpolate(start, goal, i / 19.0));
        samples.Add(start);
        samples.Add(goal);

        for (var i = 0; i < 40; i++)
            foreach (var q in samples)
                checker.IsCollisionFree(q, scene);

        const int n = 500;
        var before = GC.GetAllocatedBytesForCurrentThread();
        var sw = Stopwatch.StartNew();
        var hits = 0;
        for (var i = 0; i < n; i++)
        {
            if (!checker.IsCollisionFree(samples[i % samples.Count], scene))
                hits++;
        }
        sw.Stop();
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        var usPerCheck = sw.Elapsed.TotalMilliseconds * 1000.0 / n;
        var bytesPerCheck = allocated / (double)n;
        _output.WriteLine(
            $"Example03 hot path: n={n} elapsedMs={sw.ElapsedMilliseconds} " +
            $"usPerCheck={usPerCheck:F1} allocBytes={allocated} bytesPerCheck={bytesPerCheck:F0} " +
            $"inCollisionSamples={hits} scene={DescribeScene(scene)}");

        Assert.True(hits > 0, "expected some interpolated samples in collision with blocking sphere");
        Assert.True(usPerCheck < 5_000, $"IsCollisionFree too slow: {usPerCheck:F1} µs/check");
        Assert.True(bytesPerCheck < 200_000, $"IsCollisionFree alloc too high: {bytesPerCheck:F0} B/check");
    }

    [Fact]
    public void Example03_SegmentAndCheck_Throughput()
    {
        var robot = LoadUr10eRobotiq();
        var checker = CollisionCheckerFactory.Create(robot);
        var start = new JointState(StartJoints);
        var goal = new JointState(GoalJoints);
        var scene = ResolveBlockingScene(checker, robot, start, goal);

        for (var i = 0; i < 3; i++)
            checker.SegmentCollisionFree(start, goal, scene, StepRadians);

        const int n = 25;
        var before = GC.GetAllocatedBytesForCurrentThread();
        var sw = Stopwatch.StartNew();
        var blocked = 0;
        for (var i = 0; i < n; i++)
        {
            if (!checker.SegmentCollisionFree(start, goal, scene, StepRadians))
                blocked++;
        }
        sw.Stop();
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        _output.WriteLine(
            $"Example03 segment: n={n} ms={sw.ElapsedMilliseconds} " +
            $"msPerSegment={sw.Elapsed.TotalMilliseconds / n:F2} allocMB={allocated / (1024.0 * 1024.0):F2} " +
            $"blocked={blocked}/{n}");

        Assert.Equal(n, blocked);
    }

    /// <summary>
    /// Prefer canonical ColSphere from generate-examples when it blocks the GH start→goal path;
    /// otherwise place a same-named sphere on the TCP path (saved .ghx uses Populate 3D for Center).
    /// </summary>
    private CollisionScene ResolveBlockingScene(
        ICollisionChecker checker, RobotModel robot, JointState start, JointState goal)
    {
        foreach (var radius in new[] { CanonicalSphereRadius, SavedGhxSphereRadius, 0.15, 0.2 })
        {
            var trial = new CollisionScene(new[]
            {
                CollisionObject.Sphere("sphere", CanonicalSphereCenter, radius)
            });
            if (IsBlocking(checker, start, goal, trial))
            {
                _output.WriteLine($"Using canonical ColSphere center with r={radius}");
                return trial;
            }
        }

        var fk = KinematicsResolver.CreateFkSolver(robot.Preset);
        for (var s = 1; s <= 12; s++)
        {
            var alpha = s / 13.0;
            var midQ = Interpolate(start, goal, alpha);
            var tcp = fk.ComputeTcp(midQ, robot.Preset.BaseFrame, robot.Preset.ToolFrame).Tcp;
            foreach (var radius in new[] { SavedGhxSphereRadius, CanonicalSphereRadius, 0.08, 0.06 })
            {
                var trial = new CollisionScene(new[]
                {
                    CollisionObject.Sphere("sphere", tcp, radius)
                });
                if (IsBlocking(checker, start, goal, trial))
                {
                    _output.WriteLine(
                        $"Canonical ColSphere does not block UR10e path; " +
                        $"using TCP-path sphere at ({tcp.X:F3},{tcp.Y:F3},{tcp.Z:F3}) r={radius} (α={alpha:F2})");
                    return trial;
                }
            }
        }

        throw new InvalidOperationException(
            "Could not place a blocking ColSphere for Example03 start/goal — check URDF collision load.");
    }

    private static bool IsBlocking(
        ICollisionChecker checker, JointState start, JointState goal, CollisionScene scene) =>
        checker.IsCollisionFree(start, scene)
        && checker.IsCollisionFree(goal, scene)
        && !checker.SegmentCollisionFree(start, goal, scene, StepRadians);

    private static string DescribeScene(CollisionScene scene)
    {
        var o = scene.Objects[0];
        return $"{o.Name} shape={o.Shape} pose=({o.Pose.X:F3},{o.Pose.Y:F3},{o.Pose.Z:F3}) r={o.ExtentX:F3}";
    }

    private static RobotModel LoadUr10eRobotiq()
    {
        foreach (var path in CandidateUrdfPaths())
        {
            if (!File.Exists(path)) continue;
            var urdf = UrdfRobotLoader.Load(path, new UrdfLoadOptions
            {
                BaseLink = "base_link",
                TipLink = "tool0",
                ModelName = "UR10e"
            });
            var model = urdf.ToModel();
            if (model.CollisionModel is { Links.Count: > 0 })
                return model;
        }

        var resources = Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));
        return PresetLoader.LoadRobotModelByName("UR10e", resources);
    }

    private static IEnumerable<string> CandidateUrdfPaths()
    {
        // Sibling Motus.Grasshopper bundle (Motus UR10e Robotiq component)
        yield return Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory, "..", "..", "..", "..", "..", "..",
            "Motus.Grasshopper", "resources", "robots", "ur10e_robotiq", "ur10e_robotiq.urdf"));
        yield return Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory, "..", "..", "..", "..", "..", "..",
            "Motus.Grasshopper", "examples", "ur10e", "ur10e_robotiq.urdf"));
        yield return Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory, "..", "..", "..", "..", "..", "tests", "fixtures", "ur10e", "ur10e.urdf"));
    }

    private static JointState Interpolate(JointState a, JointState b, double t)
    {
        var q = new double[a.AxisCount];
        for (var i = 0; i < q.Length; i++)
            q[i] = a.Positions[i] + t * (b.Positions[i] - a.Positions[i]);
        return new JointState(q);
    }
}
