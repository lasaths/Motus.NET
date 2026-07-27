using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Numerical Stewart FK: leg lengths → platform pose via Newton on residual
/// r_i = ‖R·P_i + t − B_i‖ − L_i, state = [x,y,z,roll,pitch,yaw].
/// Documented defaults: ADR 0003 / <see cref="StewartSolverOptions"/>.
/// </summary>
public sealed class StewartForwardKinematics
{
    private readonly StewartPlatform _platform;
    private readonly StewartSolverOptions _opts;

    public StewartForwardKinematics(StewartPlatform platform)
    {
        _platform = platform ?? throw new ArgumentNullException(nameof(platform));
        _opts = platform.SolverOptions;
    }

    public KinematicsSolveResult TrySolve(JointState lengths, CartesianPose? seedPose = null)
    {
        if (lengths is null || lengths.AxisCount != StewartPlatform.LegCount)
            return KinematicsSolveResult.Fail(KinematicsReason.InvalidInput, $"Need {StewartPlatform.LegCount} leg lengths.");
        for (var i = 0; i < StewartPlatform.LegCount; i++)
        {
            if (!double.IsFinite(lengths.Positions[i]))
                return KinematicsSolveResult.Fail(KinematicsReason.InvalidInput, $"Leg {i + 1} length is non-finite.");
            if (!_platform.StrokeLimits[i].Contains(lengths.Positions[i]))
                return KinematicsSolveResult.Fail(
                    KinematicsReason.StrokeLimit,
                    $"Leg {i + 1} length {lengths.Positions[i]:F6} m outside stroke.");
        }

        var seed = seedPose?.Tcp ?? GuessSeedFromLengths(lengths);
        var x = PoseToState(seed);
        var residual = new double[6];
        var jacobian = new double[6 * 6];
        var delta = new double[6];

        for (var iter = 0; iter < _opts.FkMaxIterations; iter++)
        {
            FillResidual(x, lengths.Positions, residual);
            var residualNorm = Norm(residual);
            if (residualNorm < _opts.FkPositionTolMeters)
            {
                var pose = StateToPose(x);
                return KinematicsSolveResult.OkPose(new CartesianPose(pose), $"FK converged in {iter} iterations.");
            }

            FillJacobianFiniteDiff(x, lengths.Positions, jacobian);
            // Mixed-unit FD Jacobian (m vs rad) makes ‖J‖∞·‖J⁻¹‖∞ huge (~1e10) even when
            // Newton is healthy — do not gate on EstimateCondition; trust the linear solve.
            if (!TrySolveLinear6(jacobian, residual, delta))
            {
                return KinematicsSolveResult.Fail(
                    KinematicsReason.Singular,
                    $"Jacobian solve failed at FK iteration {iter}.");
            }

            // Newton: x <- x - J^{-1} r
            for (var i = 0; i < 6; i++)
                x[i] -= delta[i];

            if (!AllFinite(x))
                return KinematicsSolveResult.Fail(KinematicsReason.FkDiverge, "FK state became non-finite.");
        }

        FillResidual(x, lengths.Positions, residual);
        return KinematicsSolveResult.Fail(
            KinematicsReason.FkDiverge,
            $"FK did not converge within {_opts.FkMaxIterations} iterations (residual={Norm(residual):E3} m).");
    }

    private Frame GuessSeedFromLengths(JointState lengths)
    {
        var avg = 0.0;
        for (var i = 0; i < StewartPlatform.LegCount; i++)
            avg += lengths.Positions[i];
        avg /= StewartPlatform.LegCount;
        // Height ≈ mid-stroke for classic hex (avg L is radial-biased high, not Z).
        var mid = 0.5 * (_platform.StrokeLimits[0].Min + _platform.StrokeLimits[0].Max);
        var z = mid > 0 ? mid : avg;
        return new Frame(0, 0, z);
    }

    private void FillResidual(double[] state, double[] L, double[] residual)
    {
        var pose = StateToPose(state);
        Span<double> computed = stackalloc double[6];
        _platform.LegLengthAtPose(pose, computed);
        for (var i = 0; i < 6; i++)
            residual[i] = computed[i] - L[i];
    }

    private void FillJacobianFiniteDiff(double[] state, double[] L, double[] J)
    {
        var r0 = new double[6];
        var r1 = new double[6];
        FillResidual(state, L, r0);
        var stepPos = _opts.FiniteDiffStepMeters;
        var stepAng = _opts.FiniteDiffStepRadians;
        for (var col = 0; col < 6; col++)
        {
            var step = col < 3 ? stepPos : stepAng;
            var saved = state[col];
            state[col] = saved + step;
            FillResidual(state, L, r1);
            state[col] = saved;
            for (var row = 0; row < 6; row++)
                J[row * 6 + col] = (r1[row] - r0[row]) / step;
        }
    }

    private static double[] PoseToState(Frame f)
    {
        var (roll, pitch, yaw) = QuatToRpy(f.Qw, f.Qx, f.Qy, f.Qz);
        return [f.X, f.Y, f.Z, roll, pitch, yaw];
    }

    private static Frame StateToPose(double[] s)
    {
        var m = Transforms.FromRpy(s[0], s[1], s[2], s[3], s[4], s[5]);
        return Transforms.ToFrame(m);
    }

    private static (double roll, double pitch, double yaw) QuatToRpy(double w, double x, double y, double z)
    {
        var q = Transforms.NormalizeQuat(w, x, y, z);
        w = q.w; x = q.x; y = q.y; z = q.z;
        // ZYX yaw-pitch-roll
        var sinr = 2 * (w * x + y * z);
        var cosr = 1 - 2 * (x * x + y * y);
        var roll = Math.Atan2(sinr, cosr);
        var sinp = 2 * (w * y - z * x);
        var pitch = Math.Abs(sinp) >= 1 ? Math.CopySign(Math.PI / 2, sinp) : Math.Asin(sinp);
        var siny = 2 * (w * z + x * y);
        var cosy = 1 - 2 * (y * y + z * z);
        var yaw = Math.Atan2(siny, cosy);
        return (roll, pitch, yaw);
    }

    private static double Norm(double[] v)
    {
        var s = 0.0;
        for (var i = 0; i < v.Length; i++) s += v[i] * v[i];
        return Math.Sqrt(s);
    }

    private static bool AllFinite(double[] v)
    {
        for (var i = 0; i < v.Length; i++)
            if (!double.IsFinite(v[i])) return false;
        return true;
    }

    /// <summary>Gaussian elimination with partial pivoting for 6x6; returns false if singular.</summary>
    private static bool TrySolveLinear6(double[] aIn, double[] bIn, double[] x)
    {
        var a = new double[36];
        Array.Copy(aIn, a, 36);
        var b = new double[6];
        Array.Copy(bIn, b, 6);
        for (var col = 0; col < 6; col++)
        {
            var pivot = col;
            var max = Math.Abs(a[col * 6 + col]);
            for (var row = col + 1; row < 6; row++)
            {
                var v = Math.Abs(a[row * 6 + col]);
                if (v > max) { max = v; pivot = row; }
            }
            if (max < 1e-14) return false;
            if (pivot != col)
            {
                for (var k = 0; k < 6; k++)
                    (a[col * 6 + k], a[pivot * 6 + k]) = (a[pivot * 6 + k], a[col * 6 + k]);
                (b[col], b[pivot]) = (b[pivot], b[col]);
            }
            var diag = a[col * 6 + col];
            for (var row = col + 1; row < 6; row++)
            {
                var f = a[row * 6 + col] / diag;
                b[row] -= f * b[col];
                for (var k = col; k < 6; k++)
                    a[row * 6 + k] -= f * a[col * 6 + k];
            }
        }
        for (var i = 5; i >= 0; i--)
        {
            var sum = b[i];
            for (var k = i + 1; k < 6; k++)
                sum -= a[i * 6 + k] * x[k];
            x[i] = sum / a[i * 6 + i];
        }
        return true;
    }

}
