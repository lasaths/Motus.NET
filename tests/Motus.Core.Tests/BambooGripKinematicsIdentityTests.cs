using Motus.Construction;
using K = Motus.Construction.BambooGripKinematics;

namespace Motus.Core.Tests;

/// <summary>
/// The allocation-free grip IK must give bit-identical solutions to the earlier allocating code (kept below as the
/// reference, verbatim), so reach maps, scene checks and plans do not change.
/// </summary>
public class BambooGripKinematicsIdentityTests
{
    [Fact]
    public void SolveGrip_BitIdenticalToReference()
    {
        var rng = new Random(11);
        var lim = BambooIcdMotion.LimitsDeg;
        var solved = 0;
        for (var c = 0; c < 120; c++)
        {
            var bg = c % 2 == 0 ? K.Gripper.L : K.Gripper.R;
            var bw = K.StakeClampFrame(rng.NextDouble() - 0.5, 0.1 + 0.3 * rng.NextDouble(), rng.NextDouble() - 0.5, 360 * rng.NextDouble());
            var q = new double[5];
            for (var k = 0; k < 5; k++) q[k] = lim[k].Min + (lim[k].Max - lim[k].Min) * rng.NextDouble();
            var g = K.Free(bg, bw, q);
            // Every third target is pushed off the reachable pose (likely no solution).
            var off = c % 3 == 2 ? 0.05 : 0.0;
            var t = new K.GripTarget(g[3] + off, g[7], g[11], g[2], g[6], g[10], "x");
            var backoff = c % 4 == 1 ? 0.02 : 0;
            var axisSign = (c % 5) switch { 1 => 1, 2 => -1, _ => 0 };
            var seeds = c % 7 == 0 ? new[] { q } : null;
            var got = K.SolveGrip(bg, bw, t, seeds, 8, backoff, 7 + c, axisSign);
            var want = Reference.SolveGrip(bg, bw, t, seeds, 8, backoff, 7 + c, axisSign);
            Assert.Equal(want.Count, got.Count);
            for (var i = 0; i < want.Count; i++)
                for (var k = 0; k < 5; k++)
                    Assert.Equal(BitConverter.DoubleToInt64Bits(want[i][k]), BitConverter.DoubleToInt64Bits(got[i][k]));
            if (got.Count > 0) solved++;
        }
        Assert.True(solved > 30, $"only {solved} cases solved");
    }

    /// <summary>The pre-speedup SolveGrip (allocating), verbatim.</summary>
    static class Reference
    {
        const double Deg = Math.PI / 180.0;

        public static List<double[]> SolveGrip(K.Gripper baseGripper, double[] baseWorld, K.GripTarget target,
            IEnumerable<double[]>? seeds, int randomSeeds, double backoff, int rngSeed, int axisSign)
        {
            var all = new List<double[]>();
            if (seeds is not null) all.AddRange(seeds.Select(s => s.ToArray()));
            var rng = new Random(rngSeed);
            for (var i = 0; i < randomSeeds * K.SeedScale; i++)
            {
                var q = new double[5];
                for (var k = 0; k < 5; k++)
                {
                    var (lo, hi) = BambooIcdMotion.LimitsDeg[k];
                    q[k] = lo + (hi - lo) * rng.NextDouble();
                }
                all.Add(q);
            }
            var sols = new List<double[]>();
            foreach (var seed in all)
            {
                var q = Refine(baseGripper, baseWorld, target, seed, backoff, axisSign);
                if (q is null) continue;
                if (sols.Any(s => K.JointDist(s, q) < 2.0)) continue;
                sols.Add(q);
            }
            return sols;
        }

        static double[] ErrorVec(K.Gripper bg, double[] bw, K.GripTarget t, double[] q, double backoff, int axisSign)
        {
            var g = K.Free(bg, bw, q);
            if (backoff != 0) g = K.Mul(g, K.Trans(0, -backoff, 0));
            var zx = g[2]; var zy = g[6]; var zz = g[10];
            var dot = zx * t.Ax + zy * t.Ay + zz * t.Az;
            var sgn = axisSign != 0 ? axisSign : dot >= 0 ? 1.0 : -1.0;
            var ux = t.Ax * sgn; var uy = t.Ay * sgn; var uz = t.Az * sgn;
            const double w = 0.1;
            return new[] { t.X - g[3], t.Y - g[7], t.Z - g[11], w * (zy * uz - zz * uy), w * (zz * ux - zx * uz), w * (zx * uy - zy * ux) };
        }

        static double[]? Refine(K.Gripper bg, double[] bw, K.GripTarget t, double[] seed, double backoff, int axisSign)
        {
            var q = seed.ToArray();
            for (var k = 0; k < 5; k++) q[k] = Math.Clamp(q[k], BambooIcdMotion.LimitsDeg[k].Min, BambooIcdMotion.LimitsDeg[k].Max);
            var lambda = 0.01;
            var e = ErrorVec(bg, bw, t, q, backoff, axisSign);
            var en = Norm(e);
            for (var it = 0; it < 120; it++)
            {
                var (pe, ae) = K.GripResidual(bg, bw, t, q, backoff);
                if (pe < 1e-7 && ae < 1e-5) break;
                var J = new double[6, 5];
                const double h = 1e-6;
                for (var k = 0; k < 5; k++)
                {
                    var qp = q.ToArray(); qp[k] += h / Deg;
                    var ep = ErrorVec(bg, bw, t, qp, backoff, axisSign);
                    for (var r = 0; r < 6; r++) J[r, k] = -(ep[r] - e[r]) / h;
                }
                var A = new double[5, 5]; var b = new double[5];
                for (var i = 0; i < 5; i++)
                {
                    for (var j = 0; j < 5; j++)
                    {
                        var s = 0.0;
                        for (var r = 0; r < 6; r++) s += J[r, i] * J[r, j];
                        A[i, j] = s + (i == j ? lambda * lambda : 0);
                    }
                    var sb = 0.0;
                    for (var r = 0; r < 6; r++) sb += J[r, i] * e[r];
                    b[i] = sb;
                }
                var dq = Solve5(A, b);
                if (dq is null) return null;
                var maxStep = 0.0;
                for (var k = 0; k < 5; k++) maxStep = Math.Max(maxStep, Math.Abs(dq[k]));
                var scale = maxStep > 0.35 ? 0.35 / maxStep : 1.0;
                var qn = q.ToArray();
                for (var k = 0; k < 5; k++)
                    qn[k] = Math.Clamp(q[k] + dq[k] * scale / Deg, BambooIcdMotion.LimitsDeg[k].Min, BambooIcdMotion.LimitsDeg[k].Max);
                var en2 = ErrorVec(bg, bw, t, qn, backoff, axisSign);
                var nn = Norm(en2);
                if (nn < en) { q = qn; e = en2; en = nn; lambda = Math.Max(1e-5, lambda * 0.5); }
                else { lambda *= 4; if (lambda > 1e3) break; }
            }
            var (p, a) = K.GripResidual(bg, bw, t, q, backoff);
            if (p > K.GripPosTolM || a > K.GripAxisTolDeg) return null;
            if (axisSign != 0)
            {
                var g = K.Free(bg, bw, q);
                if ((g[2] * t.Ax + g[6] * t.Ay + g[10] * t.Az) * axisSign <= 0) return null;
            }
            return K.WithinLimits(q) ? q : null;
        }

        static double Norm(double[] v) { var s = 0.0; foreach (var x in v) s += x * x; return Math.Sqrt(s); }

        static double[]? Solve5(double[,] A, double[] b)
        {
            const int n = 5;
            var M = new double[n, n + 1];
            for (var i = 0; i < n; i++) { for (var j = 0; j < n; j++) M[i, j] = A[i, j]; M[i, n] = b[i]; }
            for (var c = 0; c < n; c++)
            {
                var piv = c;
                for (var r = c + 1; r < n; r++) if (Math.Abs(M[r, c]) > Math.Abs(M[piv, c])) piv = r;
                if (Math.Abs(M[piv, c]) < 1e-14) return null;
                if (piv != c) for (var j = 0; j <= n; j++) (M[c, j], M[piv, j]) = (M[piv, j], M[c, j]);
                for (var r = 0; r < n; r++)
                {
                    if (r == c) continue;
                    var f = M[r, c] / M[c, c];
                    for (var j = c; j <= n; j++) M[r, j] -= f * M[c, j];
                }
            }
            var x = new double[n];
            for (var i = 0; i < n; i++) x[i] = M[i, n] / M[i, i];
            return x;
        }
    }
}
