using BenchmarkDotNet.Attributes;
using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;

namespace Motus.Benchmarks;

/// <summary>Compare registered planners on UR5e free-space and mesh obstacle scenes.</summary>
[MemoryDiagnoser]
public class PlannerComparisonBenchmarks
{
    private RobotPreset _preset = null!;
    private RobotModel _robot = null!;
    private JointState _start = null!;
    private JointState _goal = null!;
    private SphereCollisionChecker _checker = null!;
    private CollisionScene _scene = null!;
    private PlanningRequest _freeRequest = null!;
    private PlanningRequest _obstacleRequest = null!;

    [GlobalSetup]
    public void Setup()
    {
        _preset = BenchmarkFixture.LoadUr5e();
        _robot = new RobotModel(_preset);
        _start = BenchmarkFixture.Home;
        _goal = BenchmarkFixture.Goal;
        _checker = new SphereCollisionChecker(_preset);
        _scene = new CollisionScene(new[]
        {
            CollisionObject.Sphere("bench_sphere", new Frame(0.45, 0, 0.35, 1, 0, 0, 0), 0.12)
        });
        _freeRequest = new PlanningRequest(_robot, _start, _goal);
        _obstacleRequest = new PlanningRequest(_robot, _start, _goal, new PlanningOptions
        {
            CollisionScene = _scene,
            CollisionChecker = _checker,
            MaxJointStepRadians = 0.08
        });
    }

    [Benchmark]
    public bool RrtConnect_FreeSpace() => Plan(SamplingPlannerId.RrtConnect, _freeRequest);

    [Benchmark]
    public bool RrtConnect_Obstacle() => Plan(SamplingPlannerId.RrtConnect, _obstacleRequest);

    [Benchmark]
    public bool RrtStar_FreeSpace() => Plan(SamplingPlannerId.RrtStar, _freeRequest);

    [Benchmark]
    public bool Lbkpiece_Obstacle() => Plan(SamplingPlannerId.Lbkpiece, _obstacleRequest);

    private bool Plan(SamplingPlannerId id, PlanningRequest request)
    {
        var opts = new SamplingPlannerOptions
        {
            PlannerId = id,
            MaxIterations = 2500,
            RandomSeed = 42,
            PreferManaged = id == SamplingPlannerId.RrtConnect
        };
        return SamplingPlanner.Create(_checker, opts).Plan(request).Success;
    }
}
