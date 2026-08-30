using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// IK for UR robots loaded from URDF. Universal Robots URDF and bundled DH profiles
/// share joint values but differ by a 180° base Z rotation in TCP pose; analytic UR IK
/// runs in DH frame then solutions are verified with the URDF FK chain.
/// </summary>
internal sealed class UrdfUniversalRobotsKinematics : IInverseKinematics
{
    // URDF base_link vs Motus DH preset: same joints, TCP xy negated (Rz(pi)).
    private static readonly double[] UrdfToDhBase = Transforms.FromRpy(0, 0, 0, 0, 0, Math.PI);

    private readonly UrInverseKinematics _dhIk;
    private readonly IFkSolver _urdfFk;
    private readonly NumericalInverseKinematics _numerical;
    private readonly BaseFrame _base;
    private readonly ToolFrame _tool;
    private readonly double[] _baseM;
    private readonly double[] _toolM;

    public UrdfUniversalRobotsKinematics(RobotPreset preset, SerialJointChain chain)
    {
        _dhIk = new UrInverseKinematics(preset);
        _urdfFk = KinematicsResolver.CreateFkSolver(preset, chain);
        _numerical = new NumericalInverseKinematics(_urdfFk, preset);
        _base = preset.BaseFrame;
        _tool = preset.ToolFrame;
        _baseM = Transforms.FromFrame(_base.Frame);
        _toolM = Transforms.FromFrame(_tool.Frame);
    }

    public bool TrySolve(CartesianPose target, JointState seed, out JointState solution)
    {
        var dhTarget = ToDhFrame(target);
        if (_dhIk.TrySolve(dhTarget, seed, out solution) && UrdfTcpMatches(target, solution))
            return true;

        foreach (var trySeed in ExtraSeeds(seed))
        {
            if (_dhIk.TrySolve(dhTarget, trySeed, out solution) && UrdfTcpMatches(target, solution))
                return true;
        }

        return _numerical.TrySolve(target, seed, out solution);
    }

    public bool TrySolveNear(CartesianPose target, JointState seed, out JointState solution)
    {
        var dhTarget = ToDhFrame(target);
        if (_dhIk.TrySolveNear(dhTarget, seed, out solution) && UrdfTcpMatches(target, solution))
            return true;
        return _numerical.TrySolveNear(target, seed, out solution);
    }

    private static IEnumerable<JointState> ExtraSeeds(JointState seed)
    {
        yield return new JointState(new[] { 0.0, -1.4, 1.4, -1.4, -1.5708, 0.0 });
        yield return new JointState(new double[seed.AxisCount]);
    }

    private static CartesianPose ToDhFrame(CartesianPose urdf) =>
        new(Transforms.ToFrame(Transforms.Multiply(UrdfToDhBase, Transforms.FromFrame(urdf.Tcp))));

    private bool UrdfTcpMatches(CartesianPose target, JointState joints) =>
        Transforms.TcpMatches(
            Transforms.TcpFromJoints(_urdfFk, joints.Positions, _baseM, _toolM),
            target.Tcp,
            5e-3,
            0.05);
}
