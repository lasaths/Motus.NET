using System.Diagnostics;
using BenchmarkDotNet.Attributes;
using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;

namespace Motus.Benchmarks;

/// <summary>Separates collision-check cost from RRT tree growth on mesh checker paths.</summary>
[MemoryDiagnoser]
public class PlanningProfileBenchmarks
{
    private RobotPreset _preset = null!;
    private RobotModel _robot = null!;
    private JointState _start = null!;
    private JointState _goal = null!;
    private RobotMeshCollisionChecker _meshChecker = null!;
    private CollisionScene _scene = null!;
    private PlanningRequest _request = null!;

    [GlobalSetup]
    public void Setup()
    {
        _preset = BenchmarkFixture.LoadUr5e();
        _robot = new RobotModel(_preset);
        _start = BenchmarkFixture.Home;
        _goal = BenchmarkFixture.Goal;
        _meshChecker = new RobotMeshCollisionChecker(_robot);
        _scene = new CollisionScene(new[]
        {
            CollisionObject.Sphere("bench_sphere", new Frame(0.45, 0, 0.35, 1, 0, 0, 0), 0.12)
        });
        _request = new PlanningRequest(_robot, _start, _goal, new PlanningOptions
        {
            CollisionScene = _scene,
            CollisionChecker = _meshChecker,
            MaxJointStepRadians = 0.08
        });
    }

    [Benchmark]
    public bool MeshStateCollisionCheck() => _meshChecker.IsCollisionFree(_start, _scene);

    [Benchmark]
    public bool MeshSegmentCollisionCheck() =>
        _meshChecker.SegmentCollisionFree(_start, _goal, _scene, stepRadians: 0.08);

    [Benchmark]
    public bool ManagedRrtConnect_2kPreferManaged()
    {
        var result = SamplingPlanner.Create(_meshChecker, new SamplingPlannerOptions
        {
            PlannerId = SamplingPlannerId.RrtConnect,
            MaxIterations = 2000,
            RandomSeed = 42,
            PreferManaged = true
        }).Plan(_request);
        return result.Success;
    }

    [Benchmark]
    public PlanningProfileReport ProfileManagedRrtOnce()
    {
        var opts = new SamplingPlannerOptions
        {
            PlannerId = SamplingPlannerId.RrtConnect,
            MaxIterations = 1500,
            RandomSeed = 7,
            PreferManaged = true,
            StepRadians = 0.12
        };
        var sw = Stopwatch.StartNew();
        var ccOnly = 0L;
        var ccSw = new Stopwatch();
        var probe = new ProfilingCollisionChecker(_meshChecker, () =>
        {
            if (!ccSw.IsRunning) ccSw.Start();
        }, () =>
        {
            ccOnly += ccSw.ElapsedTicks;
            ccSw.Reset();
        });
        var req = new PlanningRequest(_robot, _start, _goal, new PlanningOptions
        {
            CollisionScene = _scene,
            CollisionChecker = probe,
            MaxJointStepRadians = 0.08
        });
        var result = SamplingPlanner.Create(probe, opts).Plan(req);
        sw.Stop();
        return new PlanningProfileReport(result.Success, sw.Elapsed.TotalMilliseconds, ccOnly / (double)Stopwatch.Frequency * 1000.0);
    }

    public readonly record struct PlanningProfileReport(bool Success, double TotalMs, double CollisionMs);

    private sealed class ProfilingCollisionChecker : ICollisionChecker
    {
        private readonly ICollisionChecker _inner;
        private readonly Action _enter;
        private readonly Action _leave;

        public ProfilingCollisionChecker(ICollisionChecker inner, Action enter, Action leave)
        {
            _inner = inner;
            _enter = enter;
            _leave = leave;
        }

        public bool IsCollisionFree(JointState state, CollisionScene scene)
        {
            _enter();
            try { return _inner.IsCollisionFree(state, scene); }
            finally { _leave(); }
        }

        public bool SegmentCollisionFree(JointState from, JointState to, CollisionScene scene, double stepRadians)
        {
            _enter();
            try { return _inner.SegmentCollisionFree(from, to, scene, stepRadians); }
            finally { _leave(); }
        }
    }
}
