using Motus.Core;

namespace Motus.Geometry;

/// <summary>UR preset IK: analytic closed-form when possible, numerical fallback.</summary>
public sealed class UrInverseKinematics : IInverseKinematics
{
    private readonly IFkSolver _fk;
    private readonly KinematicsChain _chain;
    private readonly NumericalInverseKinematics? _numerical;
    private readonly BaseFrame _base;
    private readonly ToolFrame _tool;
    private readonly IReadOnlyList<JointLimit> _limits;
    private readonly double[] _baseM;
    private readonly double[] _toolM;

    public UrInverseKinematics(RobotPreset preset, SerialJointChain? verifyChain = null)
        : this(
            KinematicsProfiles.GetRequired(preset),
            verifyChain is not null
                ? KinematicsResolver.CreateFkSolver(preset, verifyChain)
                : KinematicsResolver.CreateFkSolver(preset),
            preset,
            verifyChain is null)
    {
    }

    private UrInverseKinematics(KinematicsChain chain, IFkSolver fk, RobotPreset preset, bool allowNumericalFallback)
    {
        _chain = chain;
        _fk = fk;
        _numerical = allowNumericalFallback ? new NumericalInverseKinematics(fk, preset) : null;
        _base = preset.BaseFrame;
        _tool = preset.ToolFrame;
        _limits = preset.JointLimits;
        _baseM = Transforms.FromFrame(_base.Frame);
        _toolM = Transforms.FromFrame(_tool.Frame);
    }

    public bool TrySolve(CartesianPose target, JointState seed, out JointState solution)
    {
        var targetM = Transforms.FromFrame(target.Tcp);
        var flangeM = FlangeTarget(targetM);

        JointState? best = null;
        var bestDelta = double.MaxValue;
        foreach (var candidate in UrAnalyticInverseKinematics.EnumerateSolutions(_chain, flangeM, _limits))
        {
            if (!Verify(target, candidate)) continue;
            var delta = MaxJointDelta(seed, candidate);
            if (delta < bestDelta)
            {
                bestDelta = delta;
                best = candidate;
            }
        }

        // Near wrist singularity analytic IK often returns a far wrist branch. Prefer a
        // numerical refine from the seed when it stays continuous and still verifies.
        if (_numerical is not null &&
            (best is null || bestDelta > 1.0) &&
            _numerical.TrySolveNear(target, seed, out var numerical))
        {
            numerical = UnwrapNear(seed, numerical);
            if (Verify(target, numerical))
            {
                var numDelta = MaxJointDelta(seed, numerical);
                if (best is null || numDelta < bestDelta)
                {
                    solution = numerical;
                    return true;
                }
            }
        }

        if (best is not null)
        {
            solution = UnwrapNear(seed, best);
            return true;
        }

        if (_numerical is not null && TryNumericalFromSeeds(target, seed, flangeM, out solution))
            return true;

        solution = seed;
        return false;
    }

    /// <summary>
    /// Closest verified analytic solution to <paramref name="seed"/>, then seed-only numerical.
    /// Skips the multi-start numerical hunt used by <see cref="TrySolve"/>.
    /// </summary>
    public bool TrySolveNear(CartesianPose target, JointState seed, out JointState solution)
    {
        var targetM = Transforms.FromFrame(target.Tcp);
        var flangeM = FlangeTarget(targetM);

        JointState? best = null;
        var bestDelta = double.MaxValue;
        foreach (var candidate in UrAnalyticInverseKinematics.EnumerateSolutions(_chain, flangeM, _limits))
        {
            if (!Verify(target, candidate)) continue;
            var delta = MaxJointDelta(seed, candidate);
            if (delta >= bestDelta) continue;
            bestDelta = delta;
            best = candidate;
        }

        if (best is not null && bestDelta <= 2.0)
        {
            solution = UnwrapNear(seed, best);
            return true;
        }

        if (_numerical is not null &&
            _numerical.TrySolveNear(target, seed, out var numerical))
        {
            numerical = UnwrapNear(seed, numerical);
            if (Verify(target, numerical))
            {
                solution = numerical;
                return true;
            }
        }

        solution = seed;
        return false;
    }

    private double[] FlangeTarget(double[] targetM) =>
        IsFlangeTool(_tool) ? targetM : Transforms.Multiply(targetM, Transforms.Inverse(_toolM));

    private bool TryNumericalFromSeeds(CartesianPose target, JointState seed, double[] flangeM, out JointState solution)
    {
        foreach (var trySeed in NumericalSeeds(seed, flangeM))
        {
            if (_numerical!.TrySolve(target, trySeed, out var raw))
            {
                solution = UnwrapNear(seed, raw);
                return true;
            }
        }

        solution = seed;
        return false;
    }

    private IEnumerable<JointState> NumericalSeeds(JointState seed, double[] flangeM)
    {
        yield return seed;
        if (IsNearZero(seed))
            yield return new JointState(new[] { 0.0, 0.01, -0.01, 0.01, 0.0, 0.0 });
        foreach (var candidate in UrAnalyticInverseKinematics.EnumerateSolutions(_chain, flangeM, _limits))
            yield return candidate;
    }

    private static bool IsNearZero(JointState seed)
    {
        for (var i = 0; i < seed.AxisCount; i++)
            if (Math.Abs(seed.Positions[i]) > 1e-6) return false;
        return true;
    }

    private static double MaxJointDelta(JointState a, JointState b)
    {
        var max = 0.0;
        for (var i = 0; i < a.AxisCount; i++)
        {
            var d = b.Positions[i] - a.Positions[i];
            while (d > Math.PI) d -= 2 * Math.PI;
            while (d < -Math.PI) d += 2 * Math.PI;
            max = Math.Max(max, Math.Abs(d));
        }
        return max;
    }

    private JointState UnwrapNear(JointState reference, JointState raw)
    {
        var q = new double[raw.AxisCount];
        for (var i = 0; i < q.Length; i++)
        {
            var v = raw.Positions[i];
            while (v - reference.Positions[i] > Math.PI) v -= 2 * Math.PI;
            while (v - reference.Positions[i] < -Math.PI) v += 2 * Math.PI;
            if (!_limits[i].Contains(v))
                v = raw.Positions[i];
            q[i] = v;
        }
        return new JointState(q);
    }

    private bool Verify(CartesianPose target, JointState candidate) =>
        Transforms.TcpMatches(
            Transforms.TcpFromJoints(_fk, candidate.Positions, _baseM, _toolM),
            target.Tcp,
            5e-3,
            0.05);

    private static bool IsFlangeTool(ToolFrame tool) =>
        Math.Abs(tool.Frame.X) < 1e-6 && Math.Abs(tool.Frame.Y) < 1e-6 && Math.Abs(tool.Frame.Z) < 1e-6
        && Math.Abs(tool.Frame.Qw - 1) < 1e-6;
}
