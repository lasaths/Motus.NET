using BenchmarkDotNet.Attributes;
using Motus.Core;
using Motus.Geometry;

namespace Motus.Benchmarks;

[MemoryDiagnoser]
public class KinematicsBenchmarks
{
    private RobotPreset _preset = null!;
    private DhForwardKinematics _fk = null!;
    private UrInverseKinematics _ik = null!;
    private CartesianLinearPathPlanner _lin = null!;
    private RobotModel _robot = null!;
    private JointState _home = null!;
    private CartesianPose _goalPose = null!;
    private CartesianPose _ikTarget = null!;

    [GlobalSetup]
    public void Setup()
    {
        _preset = BenchmarkFixture.LoadUr5e();
        _fk = new DhForwardKinematics(_preset);
        _ik = new UrInverseKinematics(_preset);
        _lin = new CartesianLinearPathPlanner(_preset);
        _robot = new RobotModel(_preset);
        _home = BenchmarkFixture.Home;
        _goalPose = BenchmarkFixture.LinGoal(_fk, _preset);
        _ikTarget = _fk.ComputeTcp(BenchmarkFixture.Goal, _preset.BaseFrame, _preset.ToolFrame);
    }

    [Benchmark(Baseline = true)]
    public double[] FkFlangeTransform() => _fk.ComputeFlangeTransform(_home.Positions);

    [Benchmark]
    public double[] FkTcpTransform() =>
        _fk.ComputeTcpTransform(_home.Positions, _preset.BaseFrame.Frame, _preset.ToolFrame.Frame);

    [Benchmark]
    public bool UrIkSolve() => _ik.TrySolve(_ikTarget, _home, out _);

    [Benchmark]
    public int LinPlanShort()
    {
        var result = _lin.PlanToResult(new CartesianPlanningRequest(_robot, _home, _goalPose, new PlanningOptions()));
        return result.Trajectory?.Points.Count ?? -1;
    }
}
