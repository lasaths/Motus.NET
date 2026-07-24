using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Analytic Stewart IK: platform pose → unique leg-length vector (meters).
/// Implements <see cref="IInverseKinematics"/> for planner injection; prefer
/// <see cref="TrySolveDetailed"/> for Status / reason codes.
/// </summary>
public sealed class StewartInverseKinematics : IInverseKinematics
{
    private readonly StewartPlatform _platform;

    public StewartInverseKinematics(StewartPlatform platform) =>
        _platform = platform ?? throw new ArgumentNullException(nameof(platform));

    public bool TrySolve(CartesianPose target, JointState seed, out JointState solution)
    {
        var result = TrySolveDetailed(target);
        if (result.Success && result.JointState is not null)
        {
            solution = result.JointState;
            return true;
        }
        solution = seed;
        return false;
    }

    public KinematicsSolveResult TrySolveDetailed(CartesianPose target)
    {
        if (target is null)
            return KinematicsSolveResult.Fail(KinematicsReason.InvalidInput, "Target pose is null.");
        var f = target.Tcp;
        if (!double.IsFinite(f.X) || !double.IsFinite(f.Y) || !double.IsFinite(f.Z) ||
            !double.IsFinite(f.Qw) || !double.IsFinite(f.Qx) || !double.IsFinite(f.Qy) || !double.IsFinite(f.Qz))
            return KinematicsSolveResult.Fail(KinematicsReason.InvalidInput, "Target pose contains non-finite values.");

        Span<double> L = stackalloc double[StewartPlatform.LegCount];
        _platform.LegLengthAtPose(f, L);
        var q = new double[StewartPlatform.LegCount];
        for (var i = 0; i < StewartPlatform.LegCount; i++)
        {
            if (!double.IsFinite(L[i]))
                return KinematicsSolveResult.Fail(KinematicsReason.InvalidInput, $"Leg {i + 1} length is non-finite.");
            if (!_platform.StrokeLimits[i].Contains(L[i]))
            {
                return KinematicsSolveResult.Fail(
                    KinematicsReason.StrokeLimit,
                    $"Leg {i + 1} length {L[i]:F6} m outside [{_platform.StrokeLimits[i].Min:F6}, {_platform.StrokeLimits[i].Max:F6}] m.");
            }
            q[i] = L[i];
        }

        return KinematicsSolveResult.OkJoints(new JointState(q));
    }
}
