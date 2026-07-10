using BenchmarkDotNet.Attributes;
using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;

namespace Motus.Benchmarks;

[MemoryDiagnoser]
public class OmplBenchmarks
{
    private RobotPreset _preset = null!;
    private RobotModel _robot = null!;
    private JointState _start = null!;
    private JointState _goal = null!;
    private PlanningRequest _request = null!;
    private SphereCollisionChecker _checker = null!;
    private CollisionScene _emptyScene = null!;

    [GlobalSetup]
    public void Setup()
    {
        _preset = BenchmarkFixture.LoadUr5e();
        _robot = new RobotModel(_preset);
        _start = BenchmarkFixture.Home;
        _goal = BenchmarkFixture.Goal;
        _request = new PlanningRequest(_robot, _start, _goal);
        _checker = new SphereCollisionChecker(_preset);
        _emptyScene = new CollisionScene();
    }

    [Benchmark]
    public bool RrtConnect_3kIterations()
    {
        var result = new RrtConnectPlanner(_preset, new RrtConnectOptions { MaxIterations = 3000, RandomSeed = 42 }).Plan(_request);
        return result.Success;
    }

    [Benchmark]
    public bool SphereCollisionCheck() => _checker.IsCollisionFree(_start, _emptyScene);

    [Benchmark]
    public bool SphereSegmentCheck() =>
        _checker.SegmentCollisionFree(_start, _goal, _emptyScene, stepRadians: 0.05);
}
