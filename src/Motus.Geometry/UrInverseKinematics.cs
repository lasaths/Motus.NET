using Motus.Core;

namespace Motus.Geometry;

/// <summary>UR preset IK: analytic closed-form when possible, numerical fallback.</summary>
public sealed class UrInverseKinematics : IInverseKinematics
{
    private readonly IFkSolver _fk;
    private readonly KinematicsChain _chain;
    private readonly NumericalInverseKinematics _numerical;
    private readonly BaseFrame _base;
    private readonly ToolFrame _tool;
    private readonly IReadOnlyList<JointLimit> _limits;

    public UrInverseKinematics(RobotPreset preset)
        : this(KinematicsProfiles.GetRequired(preset), KinematicsResolver.CreateFkSolver(preset), preset) { }

    private UrInverseKinematics(KinematicsChain chain, IFkSolver fk, RobotPreset preset)
    {
        _chain = chain;
        _fk = fk;
        _numerical = new NumericalInverseKinematics(fk, preset);
        _base = preset.BaseFrame;
        _tool = preset.ToolFrame;
        _limits = preset.JointLimits;
    }

    public bool TrySolve(CartesianPose target, JointState seed, out JointState solution)
    {
        if (IsFlangeTool(_tool))
        {
            var targetM = Transforms.FromFrame(target.Tcp);
            JointState? best = null;
            var bestDelta = double.MaxValue;
            foreach (var candidate in UrAnalyticInverseKinematics.EnumerateSolutions(_chain, targetM, _limits))
            {
                if (!Verify(target, candidate)) continue;
                var delta = MaxJointDelta(seed, candidate);
                if (delta < bestDelta)
                {
                    bestDelta = delta;
                    best = candidate;
                }
            }
            if (best is not null)
            {
                solution = UnwrapNear(seed, best);
                return true;
            }

            return TryNumericalFromSeeds(target, seed, targetM, out solution);
        }

        return _numerical.TrySolve(target, seed, out solution);
    }

    private bool TryNumericalFromSeeds(CartesianPose target, JointState seed, double[] targetM, out JointState solution)
    {
        foreach (var trySeed in NumericalSeeds(seed, targetM))
        {
            if (_numerical.TrySolve(target, trySeed, out solution))
                return true;
        }

        solution = seed;
        return false;
    }

    private IEnumerable<JointState> NumericalSeeds(JointState seed, double[] targetM)
    {
        yield return seed;
        if (IsNearZero(seed))
            yield return new JointState(new[] { 0.0, 0.01, -0.01, 0.01, 0.0, 0.0 });
        foreach (var candidate in UrAnalyticInverseKinematics.EnumerateSolutions(_chain, targetM, _limits))
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

    private static JointState UnwrapNear(JointState reference, JointState raw)
    {
        var q = new double[raw.AxisCount];
        for (var i = 0; i < q.Length; i++)
        {
            var v = raw.Positions[i];
            while (v - reference.Positions[i] > Math.PI) v -= 2 * Math.PI;
            while (v - reference.Positions[i] < -Math.PI) v += 2 * Math.PI;
            q[i] = v;
        }
        return new JointState(q);
    }

    private bool Verify(CartesianPose target, JointState candidate)
    {
        var actual = _fk.ComputeTcp(candidate, _base, _tool);
        var dx = actual.Tcp.X - target.Tcp.X;
        var dy = actual.Tcp.Y - target.Tcp.Y;
        var dz = actual.Tcp.Z - target.Tcp.Z;
        if (Math.Sqrt(dx * dx + dy * dy + dz * dz) >= 5e-3) return false;

        var dot = Math.Abs(
            actual.Tcp.Qw * target.Tcp.Qw +
            actual.Tcp.Qx * target.Tcp.Qx +
            actual.Tcp.Qy * target.Tcp.Qy +
            actual.Tcp.Qz * target.Tcp.Qz);
        var oriErr = 2 * Math.Acos(Math.Clamp(dot, -1, 1));
        return oriErr < 0.05;
    }

    private static bool IsFlangeTool(ToolFrame tool) =>
        Math.Abs(tool.Frame.X) < 1e-6 && Math.Abs(tool.Frame.Y) < 1e-6 && Math.Abs(tool.Frame.Z) < 1e-6
        && Math.Abs(tool.Frame.Qw - 1) < 1e-6;
}
