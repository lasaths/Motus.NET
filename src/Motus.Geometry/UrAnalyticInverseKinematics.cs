using Motus.Core;

namespace Motus.Geometry;

/// <summary>Closed-form 6-DOF IK for Universal Robots DH chains (matches KinematicsProfiles UR template).</summary>
internal static class UrAnalyticInverseKinematics
{
    private const double ZeroThresh = 1e-8;

    public static bool TrySolve(
        KinematicsChain chain,
        double[] targetTcp,
        IReadOnlyList<JointLimit> limits,
        JointState seed,
        out JointState solution)
    {
        solution = seed;
        if (chain.Links.Length != 6) return false;

        JointState? best = null;
        var bestDist = double.MaxValue;
        foreach (var q in ComputeSolutions(chain, targetTcp))
        {
            if (!WithinLimits(q, limits)) continue;
            var dist = JointDistance(q, seed.Positions);
            if (dist < bestDist)
            {
                bestDist = dist;
                best = new JointState(q);
            }
        }

        if (best is null) return false;
        solution = best;
        return true;
    }

    public static IEnumerable<JointState> EnumerateSolutions(
        KinematicsChain chain,
        double[] targetTcp,
        IReadOnlyList<JointLimit> limits)
    {
        if (chain.Links.Length != 6) yield break;

        foreach (var q in ComputeSolutions(chain, targetTcp))
        {
            if (WithinLimits(q, limits))
                yield return new JointState(q);
        }
    }

    // Hawkins row-major IK (ros-industrial/universal_robot#300), matches DhForwardKinematics layout.
    private static IEnumerable<double[]> ComputeSolutions(KinematicsChain chain, double[] t)
    {
        var d1 = chain.Links[0].D;
        var a2 = chain.Links[1].A;
        var a3 = chain.Links[2].A;
        var d4 = chain.Links[3].D;
        var d5 = chain.Links[4].D;
        var d6 = chain.Links[5].D;

        var q1 = new double[2];
        {
            var a = t[3] - d6 * t[2];
            var b = t[7] - d6 * t[6];
            var r = a * a + b * b;
            if (r < 1e-12) yield break;
            if (d4 * d4 > r + 1e-9) yield break;

            var arccos = Math.Acos(Math.Clamp(d4 / Math.Sqrt(r), -1, 1));
            var arctan = Math.Atan2(b, a);
            q1[0] = NormalizeAngle(arctan + arccos + Math.PI / 2);
            q1[1] = NormalizeAngle(arctan - arccos + Math.PI / 2);
        }

        var q5 = new double[2, 2];
        for (var i = 0; i < 2; i++)
        {
            var numer = t[3] * Math.Sin(q1[i]) - t[7] * Math.Cos(q1[i]) - d4;
            var arccos = Math.Acos(Math.Clamp(numer / d6, -1, 1));
            q5[i, 0] = arccos;
            q5[i, 1] = -arccos;
        }

        for (var i = 0; i < 2; i++)
        {
            var c1 = Math.Cos(q1[i]);
            var s1 = Math.Sin(q1[i]);
            for (var j = 0; j < 2; j++)
            {
                var c5 = Math.Cos(q5[i, j]);
                var s5 = Math.Sin(q5[i, j]);

                double q6;
                if (Math.Abs(s5) < ZeroThresh)
                    q6 = 0;
                else
                {
                    q6 = Math.Atan2(
                        Math.Sign(s5) * -(t[1] * s1 - t[5] * c1),
                        Math.Sign(s5) * (t[0] * s1 - t[4] * c1));
                    q6 = NormalizeAngle(q6);
                }

                var c6 = Math.Cos(q6);
                var s6 = Math.Sin(q6);

                var x04x = -s5 * (t[2] * c1 + t[6] * s1)
                    - c5 * (s6 * (t[1] * c1 + t[5] * s1) - c6 * (t[0] * c1 + t[4] * s1));
                var x04y = c5 * (t[8] * c6 - t[9] * s6) - t[10] * s5;
                var p13x = d5 * (s6 * (t[0] * c1 + t[4] * s1) + c6 * (t[1] * c1 + t[5] * s1))
                    - d6 * (t[2] * c1 + t[6] * s1) + t[3] * c1 + t[7] * s1;
                var p13y = t[11] - d1 - d6 * t[10] + d5 * (t[9] * c6 + t[8] * s6);

                var c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2 * a2 * a3);
                if (Math.Abs(c3) > 1.0001) continue;
                c3 = Math.Clamp(c3, -1, 1);

                var q3 = new[] { Math.Acos(c3), -Math.Acos(c3) };
                var s3 = Math.Sin(q3[0]);
                var denom = a2 * a2 + a3 * a3 + 2 * a2 * a3 * c3;
                var bigA = a2 + a3 * c3;
                var bigB = a3 * s3;
                var q2 = new[]
                {
                    Math.Atan2((bigA * p13y - bigB * p13x) / denom, (bigA * p13x + bigB * p13y) / denom),
                    Math.Atan2((bigA * p13y + bigB * p13x) / denom, (bigA * p13x - bigB * p13y) / denom),
                };

                for (var k = 0; k < 2; k++)
                {
                    var c23 = Math.Cos(q2[k] + q3[k]);
                    var s23 = Math.Sin(q2[k] + q3[k]);
                    var q4 = Math.Atan2(c23 * x04y - s23 * x04x, x04x * c23 + x04y * s23);
                    yield return NormalizeJoints([q1[i], q2[k], q3[k], q4, q5[i, j], q6]);
                }
            }
        }
    }

    private static double NormalizeAngle(double angle)
    {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private static double[] NormalizeJoints(double[] q)
    {
        for (var i = 0; i < q.Length; i++)
            q[i] = NormalizeAngle(q[i]);
        return q;
    }

    private static bool WithinLimits(double[] q, IReadOnlyList<JointLimit> limits)
    {
        for (var i = 0; i < q.Length; i++)
            if (!limits[i].Contains(q[i])) return false;
        return true;
    }

    private static double JointDistance(double[] a, IReadOnlyList<double> b)
    {
        var max = 0.0;
        for (var i = 0; i < a.Length; i++)
        {
            var d = a[i] - b[i];
            while (d > Math.PI) d -= 2 * Math.PI;
            while (d < -Math.PI) d += 2 * Math.PI;
            max = Math.Max(max, Math.Abs(d));
        }
        return max;
    }
}
