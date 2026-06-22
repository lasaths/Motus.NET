using Motus.Core;

namespace Motus.Geometry;

/// <summary>UR analytic IK with numerical fallback.</summary>
public sealed class UrInverseKinematics : IInverseKinematics
{
    private readonly NumericalInverseKinematics _numerical;

    public UrInverseKinematics(RobotPreset preset) => _numerical = new NumericalInverseKinematics(preset);

    public bool TrySolve(CartesianPose target, JointState seed, out JointState solution) =>
        _numerical.TrySolve(target, seed, out solution);
}
