using Motus.Core;

namespace Motus.Geometry;

/// <summary>Damped least-squares IK using the preset DH chain.</summary>
public sealed class NumericalInverseKinematics : IInverseKinematics
{
    private readonly DhForwardKinematics _fk;
    private readonly BaseFrame _base;
    private readonly ToolFrame _tool;
    private readonly IReadOnlyList<JointLimit> _limits;

    public NumericalInverseKinematics(RobotPreset preset)
    {
        _fk = new DhForwardKinematics(preset);
        _base = preset.BaseFrame;
        _tool = preset.ToolFrame;
        _limits = preset.JointLimits;
    }

    public bool TrySolve(CartesianPose target, JointState seed, out JointState solution)
    {
        if (TrySolveInternal(target, seed, out solution))
            return true;

        // ponytail: a few random restarts before giving up
        var rng = new Random(17);
        for (var attempt = 0; attempt < 6; attempt++)
        {
            var q = new double[_limits.Count];
            for (var i = 0; i < q.Length; i++)
            {
                var lim = _limits[i];
                q[i] = lim.MinRadians + rng.NextDouble() * (lim.MaxRadians - lim.MinRadians);
            }
            if (TrySolveInternal(target, new JointState(q), out solution))
                return true;
        }

        solution = seed;
        return false;
    }

    private bool TrySolveInternal(CartesianPose target, JointState seed, out JointState solution)
    {
        var q = (double[])seed.Positions.Clone();
        var targetM = Transforms.FromFrame(target.Tcp);
        const int maxIter = 200;
        const double posTol = 2e-4;
        const double rotTol = 2e-3;

        for (var iter = 0; iter < maxIter; iter++)
        {
            var current = _fk.ComputeTcpTransform(q, _base.Frame, _tool.Frame);
            var posErr = PositionError(current, targetM);
            var rotErr = RotationError(current, targetM);
            if (posErr < posTol && rotErr < rotTol)
            {
                solution = new JointState(Clamp(q));
                return true;
            }

            var j = Jacobian(q, targetM);
            var e = PoseErrorVector(current, targetM);
            var dq = SolveDls(j, e, 0.08);
            for (var i = 0; i < q.Length; i++)
                q[i] += dq[i];
            ClampInPlace(q);
        }

        solution = new JointState(Clamp(q));
        return false;
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

    private double[,] Jacobian(double[] q, double[] targetM)
    {
        var n = q.Length;
        var j = new double[6, n];
        const double h = 1e-5;
        var baseM = _fk.ComputeTcpTransform(q, _base.Frame, _tool.Frame);
        var e0 = PoseErrorVector(baseM, targetM);
        for (var i = 0; i < n; i++)
        {
            var qp = (double[])q.Clone();
            qp[i] += h;
            var perturbed = _fk.ComputeTcpTransform(qp, _base.Frame, _tool.Frame);
            var ei = PoseErrorVector(perturbed, targetM);
            for (var r = 0; r < 6; r++)
                j[r, i] = (ei[r] - e0[r]) / h;
        }
        return j;
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
