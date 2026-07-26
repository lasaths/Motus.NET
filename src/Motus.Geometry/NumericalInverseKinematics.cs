using Motus.Core;

namespace Motus.Geometry;

/// <summary>Damped least-squares IK for any serial chain (DH or URDF).</summary>
public sealed class NumericalInverseKinematics : IInverseKinematics
{
    private readonly IFkSolver _fk;
    private readonly BaseFrame _base;
    private readonly ToolFrame _tool;
    private readonly IReadOnlyList<JointLimit> _limits;
    private readonly double[] _baseM;
    private readonly double[] _toolM;
    private readonly double[] _toolInv;
    private readonly NumericalIkOptions _opts;
    private readonly ProductOfExponentials? _poe;

    /// <summary>Last failure reason from <see cref="TrySolveDetailed"/> / internal solve (null on success).</summary>
    public string? LastFailureReason { get; private set; }

    /// <summary>Iterations used by the last internal solve attempt.</summary>
    public int LastIterations { get; private set; }

    public NumericalInverseKinematics(RobotPreset preset, NumericalIkOptions? options = null)
        : this(KinematicsResolver.CreateFkSolver(preset), preset, options, poe: null) { }

    public NumericalInverseKinematics(IFkSolver fk, RobotPreset preset, NumericalIkOptions? options = null)
        : this(fk, preset, options, poe: null) { }

    public NumericalInverseKinematics(
        IFkSolver fk,
        RobotPreset preset,
        SerialJointChain serialChain,
        NumericalIkOptions? options = null)
        : this(fk, preset, options, ProductOfExponentials.FromSerialChain(serialChain)) { }

    public NumericalInverseKinematics(
        IFkSolver fk,
        RobotPreset preset,
        NumericalIkOptions? options,
        ProductOfExponentials? poe)
    {
        _fk = fk;
        _base = preset.BaseFrame;
        _tool = preset.ToolFrame;
        _limits = preset.JointLimits;
        _baseM = Transforms.FromFrame(_base.Frame);
        _toolM = Transforms.FromFrame(_tool.Frame);
        _toolInv = Transforms.Inverse(_toolM);
        _opts = options ?? NumericalIkOptions.Default;
        _poe = poe;
    }

    public bool TrySolve(CartesianPose target, JointState seed, out JointState solution)
    {
        var detailed = TrySolveDetailed(target, seed);
        solution = detailed.Solution;
        return detailed.Success;
    }

    public NumericalIkResult TrySolveDetailed(CartesianPose target, JointState seed)
    {
        LastFailureReason = null;
        LastIterations = 0;

        if (!IsFinitePose(target) || !IsFiniteJoints(seed) || seed.AxisCount != _limits.Count)
        {
            LastFailureReason = NumericalIkFailureReasons.InvalidInput;
            return new NumericalIkResult(false, seed, LastFailureReason, 0);
        }

        JointState? best = null;
        var bestDelta = double.MaxValue;
        string? bestFail = null;
        var bestIters = 0;

        void Consider(JointState trySeed)
        {
            var r = TrySolveInternalDetailed(target, trySeed);
            if (!r.Success)
            {
                bestFail ??= r.FailureReason;
                bestIters = Math.Max(bestIters, r.Iterations);
                return;
            }
            var delta = MaxJointDelta(seed, r.Solution);
            if (delta < bestDelta)
            {
                bestDelta = delta;
                best = r.Solution;
                bestFail = null;
                bestIters = r.Iterations;
            }
        }

        Consider(seed);
        if (best is not null && bestDelta < 0.35)
        {
            LastIterations = bestIters;
            return new NumericalIkResult(true, best, null, bestIters);
        }

        Consider(PerturbSeed(seed, 1.0));
        Consider(PerturbSeed(seed, -1.0));

        var rng = new Random(17);
        for (var attempt = 0; attempt < 10; attempt++)
        {
            var q = new double[_limits.Count];
            for (var i = 0; i < q.Length; i++)
            {
                var lim = _limits[i];
                q[i] = lim.MinRadians + rng.NextDouble() * (lim.MaxRadians - lim.MinRadians);
            }
            Consider(new JointState(q));
        }

        if (best is not null)
        {
            LastIterations = bestIters;
            return new NumericalIkResult(true, best, null, bestIters);
        }

        LastFailureReason = bestFail ?? NumericalIkFailureReasons.NoConvergence;
        LastIterations = bestIters;
        return new NumericalIkResult(false, seed, LastFailureReason, bestIters);
    }

    /// <summary>Refine from <paramref name="seed"/> only — used by LIN to stay on the current branch.</summary>
    public bool TrySolveNear(CartesianPose target, JointState seed, out JointState solution)
    {
        var r = TrySolveInternalDetailed(target, seed);
        solution = r.Solution;
        LastFailureReason = r.FailureReason;
        LastIterations = r.Iterations;
        return r.Success;
    }

    private static double MaxJointDelta(JointState a, JointState b)
    {
        var max = 0.0;
        for (var i = 0; i < a.AxisCount; i++)
            max = Math.Max(max, Math.Abs(b.Positions[i] - a.Positions[i]));
        return max;
    }

    private JointState PerturbSeed(JointState seed, double fractionTowardLimits)
    {
        var q = new double[_limits.Count];
        for (var i = 0; i < q.Length; i++)
        {
            var lim = _limits[i];
            var span = lim.MaxRadians - lim.MinRadians;
            var offset = span * 0.1 * fractionTowardLimits;
            q[i] = Math.Clamp(seed.Positions[i] + offset, lim.MinRadians, lim.MaxRadians);
        }
        return new JointState(q);
    }

    private NumericalIkResult TrySolveInternalDetailed(CartesianPose target, JointState seed)
    {
        var q = (double[])seed.Positions.Clone();
        var targetM = Transforms.FromFrame(target.Tcp);

        var maxIter = Math.Max(1, _opts.MaxIterations);
        var posTol = _opts.PositionToleranceMeters;
        var rotTol = _opts.OmegaToleranceRadians;
        var finalPosTol = _opts.FinalPositionToleranceMeters;
        var finalRotTol = _opts.FinalOmegaToleranceRadians;
        double lambda = 0.05;
        const double minLambda = 0.001;
        const double maxLambda = 0.5;

        for (var iter = 0; iter < maxIter; iter++)
        {
            var current = Transforms.TcpFromJoints(_fk, q, _baseM, _toolM);
            var posErr = PositionError(current, targetM);
            var rotErr = RotationError(current, targetM);

            if (posErr < posTol && rotErr < rotTol)
            {
                var sol = new JointState(Clamp(q));
                return new NumericalIkResult(true, sol, null, iter + 1);
            }

            if (iter > 50 && (posErr > 0.5 || rotErr > 1.0))
                lambda = Math.Min(lambda + 0.1, maxLambda);

            double[,] j;
            double[] e;
            if (_poe is not null)
            {
                // MR §6.2.2 body NR: [Vb] = log(T⁻¹ Tsd), θ ← θ + Jb⁺ Vb
                if (!TryBodyTwistError(current, targetM, out e))
                {
                    var bad = new JointState(Clamp(q));
                    return new NumericalIkResult(false, bad, NumericalIkFailureReasons.InvalidInput, iter + 1);
                }
                j = BodyJacobianAtTcp(q);
                var cond = PoEJacobian.EstimateConditionJjT(j);
                if (!double.IsFinite(cond) || cond > 1e10)
                {
                    var singularSol = new JointState(Clamp(q));
                    return new NumericalIkResult(false, singularSol, NumericalIkFailureReasons.SingularJacobian, iter + 1);
                }
            }
            else
            {
                j = FiniteDifferenceJacobian(q, targetM);
                if (IsJacobianSingular(j))
                {
                    var singularSol = new JointState(Clamp(q));
                    return new NumericalIkResult(false, singularSol, NumericalIkFailureReasons.SingularJacobian, iter + 1);
                }
                e = PoseErrorVector(current, targetM);
            }

            var currentLambda = iter < 20 ? Math.Max(lambda, 0.1) : lambda;
            var dq = SolveDls(j, e, currentLambda);
            var stepScale = posErr > 0.1 ? 0.8 : 1.0;

            for (var i = 0; i < q.Length; i++)
                q[i] += dq[i] * stepScale;

            ClampInPlace(q);

            if (iter % 20 == 0)
            {
                if (posErr < 0.05) lambda = Math.Max(minLambda, lambda * 0.9);
                else lambda = Math.Min(maxLambda, lambda * 1.1);
            }
        }

        var solution = new JointState(Clamp(q));
        var finalCheck = Transforms.TcpFromJoints(_fk, solution.Positions, _baseM, _toolM);
        var ok = PositionError(finalCheck, targetM) < finalPosTol
            && RotationError(finalCheck, targetM) < finalRotTol;
        return new NumericalIkResult(ok, solution, ok ? null : NumericalIkFailureReasons.NoConvergence, maxIter);
    }

    private double[,] BodyJacobianAtTcp(double[] q)
    {
        var jbFlange = PoEJacobian.JacobianBody(_poe!, q);
        // Fixed tool: Vb_tcp = Ad_{T_tool⁻¹} Vb_flange
        var adToolInv = ScrewMath.Adjoint(_toolInv);
        var n = q.Length;
        var jb = new double[6, n];
        for (var c = 0; c < n; c++)
        {
            var col = new double[6];
            for (var r = 0; r < 6; r++) col[r] = jbFlange[r, c];
            var mapped = ScrewMath.AdjointMultiply(adToolInv, col);
            for (var r = 0; r < 6; r++) jb[r, c] = mapped[r];
        }
        return jb;
    }

    private static bool TryBodyTwistError(double[] currentTcp, double[] targetTcp, out double[] vb)
    {
        vb = new double[6];
        var tErr = Transforms.Multiply(Transforms.Inverse(currentTcp), targetTcp);
        if (!ScrewMath.TryMatrixLog6(tErr, out var s, out var theta))
            return false;
        // se(3) coords = S · θ
        for (var i = 0; i < 6; i++)
            vb[i] = s[i] * theta;
        return true;
    }

    private double[,] FiniteDifferenceJacobian(double[] q, double[] targetM)
    {
        var n = q.Length;
        var j = new double[6, n];
        const double h = 1e-5;
        var baseM = Transforms.TcpFromJoints(_fk, q, _baseM, _toolM);
        var e0 = PoseErrorVector(baseM, targetM);
        for (var i = 0; i < n; i++)
        {
            q[i] += h;
            var perturbed = Transforms.TcpFromJoints(_fk, q, _baseM, _toolM);
            var ei = PoseErrorVector(perturbed, targetM);
            for (var r = 0; r < 6; r++)
                j[r, i] = (ei[r] - e0[r]) / h;
            q[i] -= h;
        }
        return j;
    }

    private static bool IsJacobianSingular(double[,] j)
    {
        // Rough: if all entries near zero, DLS will not move.
        var max = 0.0;
        var rows = j.GetLength(0);
        var cols = j.GetLength(1);
        for (var r = 0; r < rows; r++)
        for (var c = 0; c < cols; c++)
            max = Math.Max(max, Math.Abs(j[r, c]));
        return max < 1e-14;
    }

    private double[] Clamp(double[] q)
    {
        var r = (double[])q.Clone();
        ClampInPlace(r);
        return r;
    }

    private void ClampInPlace(double[] q)
    {
        for (var i = 0; i < q.Length; i++)
            q[i] = Math.Clamp(q[i], _limits[i].MinRadians, _limits[i].MaxRadians);
    }

    private static bool IsFinitePose(CartesianPose pose) =>
        double.IsFinite(pose.Tcp.X) && double.IsFinite(pose.Tcp.Y) && double.IsFinite(pose.Tcp.Z)
        && double.IsFinite(pose.Tcp.Qw) && double.IsFinite(pose.Tcp.Qx)
        && double.IsFinite(pose.Tcp.Qy) && double.IsFinite(pose.Tcp.Qz);

    private static bool IsFiniteJoints(JointState s)
    {
        for (var i = 0; i < s.AxisCount; i++)
            if (!double.IsFinite(s.Positions[i])) return false;
        return true;
    }

    private static double[] SolveDls(double[,] j, double[] e, double lambda)
    {
        var n = j.GetLength(1);
        var jtJ = new double[n, n];
        var jtE = new double[n];
        for (var i = 0; i < n; i++)
        {
            for (var k = 0; k < n; k++)
            {
                var sum = 0.0;
                for (var r = 0; r < 6; r++)
                    sum += j[r, i] * j[r, k];
                jtJ[i, k] = sum;
            }
            jtJ[i, i] += lambda * lambda;
            var sumE = 0.0;
            for (var r = 0; r < 6; r++)
                sumE += j[r, i] * e[r];
            jtE[i] = sumE;
        }
        return SolveSymmetric(jtJ, jtE);
    }

    private static double[] SolveSymmetric(double[,] a, double[] b)
    {
        var n = b.Length;
        var x = new double[n];
        var m = new double[n, n + 1];
        for (var i = 0; i < n; i++)
        {
            for (var j = 0; j < n; j++)
                m[i, j] = a[i, j];
            m[i, n] = b[i];
        }
        for (var col = 0; col < n; col++)
        {
            var pivot = col;
            for (var row = col + 1; row < n; row++)
                if (Math.Abs(m[row, col]) > Math.Abs(m[pivot, col])) pivot = row;
            for (var j = 0; j <= n; j++)
                (m[col, j], m[pivot, j]) = (m[pivot, j], m[col, j]);
            var div = m[col, col];
            if (Math.Abs(div) < 1e-12) continue;
            for (var j = col; j <= n; j++) m[col, j] /= div;
            for (var row = 0; row < n; row++)
            {
                if (row == col) continue;
                var factor = m[row, col];
                for (var j = col; j <= n; j++)
                    m[row, j] -= factor * m[col, j];
            }
        }
        for (var i = 0; i < n; i++) x[i] = m[i, n];
        return x;
    }

    private static double PositionError(double[] current, double[] target)
    {
        var dx = target[3] - current[3];
        var dy = target[7] - current[7];
        var dz = target[11] - current[11];
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    private static double RotationError(double[] current, double[] target)
    {
        var rErr = PoseErrorVector(current, target);
        return Math.Sqrt(rErr[3] * rErr[3] + rErr[4] * rErr[4] + rErr[5] * rErr[5]);
    }

    private static double[] PoseErrorVector(double[] current, double[] target)
    {
        var dx = target[3] - current[3];
        var dy = target[7] - current[7];
        var dz = target[11] - current[11];
        var rc = SubMatrix3(current);
        var rt = SubMatrix3(target);
        var rErr = Multiply3(Transpose3(rt), rc);
        var ax = rErr[2, 1] - rErr[1, 2];
        var ay = rErr[0, 2] - rErr[2, 0];
        var az = rErr[1, 0] - rErr[0, 1];
        return [dx, dy, dz, ax, ay, az];
    }

    private static double[,] SubMatrix3(double[] m) => new double[,]
    {
        { m[0], m[1], m[2] },
        { m[4], m[5], m[6] },
        { m[8], m[9], m[10] }
    };

    private static double[,] Transpose3(double[,] m) => new double[,]
    {
        { m[0, 0], m[1, 0], m[2, 0] },
        { m[0, 1], m[1, 1], m[2, 1] },
        { m[0, 2], m[1, 2], m[2, 2] }
    };

    private static double[,] Multiply3(double[,] a, double[,] b)
    {
        var r = new double[3, 3];
        for (var i = 0; i < 3; i++)
            for (var j = 0; j < 3; j++)
                for (var k = 0; k < 3; k++)
                    r[i, j] += a[i, k] * b[k, j];
        return r;
    }
}
