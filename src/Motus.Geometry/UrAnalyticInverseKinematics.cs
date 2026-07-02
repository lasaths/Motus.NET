using Motus.Core;

namespace Motus.Geometry;

/// <summary>Closed-form 6-DOF IK for Universal Robots DH chains (matches KinematicsProfiles UR template).</summary>
internal static class UrAnalyticInverseKinematics
{
    public static bool TrySolve(
        KinematicsChain chain,
        double[] targetTcp,
        IReadOnlyList<JointLimit> limits,
        JointState seed,
        out JointState solution)
    {
        solution = seed;
        if (chain.Links.Length != 6) return false;

        var d1 = chain.Links[0].D;
        var a2 = -chain.Links[1].A;
        var a3 = -chain.Links[2].A;
        var d4 = chain.Links[3].D;
        var d5 = chain.Links[4].D;
        var d6 = chain.Links[5].D;

        var candidates = ComputeSolutions(targetTcp, d1, a2, a3, d4, d5, d6);
        JointState? best = null;
        var bestDist = double.MaxValue;

        foreach (var q in candidates)
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

        var d1 = chain.Links[0].D;
        var a2 = -chain.Links[1].A;
        var a3 = -chain.Links[2].A;
        var d4 = chain.Links[3].D;
        var d5 = chain.Links[4].D;
        var d6 = chain.Links[5].D;

        foreach (var q in ComputeSolutions(targetTcp, d1, a2, a3, d4, d5, d6))
        {
            if (WithinLimits(q, limits))
                yield return new JointState(q);
        }
    }

    private static List<double[]> ComputeSolutions(double[] t, double d1, double a2, double a3, double d4, double d5, double d6)
    {
        var results = new List<double[]>();
        var ox = t[3]; var oy = t[7]; var oz = t[11];
        var ax = t[2]; var ay = t[6]; var az = t[10];

        var wx = ox - d6 * ax;
        var wy = oy - d6 * ay;
        var wz = oz - d6 * az;

        var r = Math.Sqrt(wx * wx + wy * wy);
        if (r < 1e-9 || Math.Abs(d4) > r + 1e-9) return results;

        var phi = Math.Acos(Math.Clamp(d4 / r, -1, 1));
        var psi = Math.Atan2(wy, wx);
        var q1Candidates = new[] { psi + phi + Math.PI / 2, psi - phi - Math.PI / 2 };

        foreach (var q1 in q1Candidates)
        {
            var c1 = Math.Cos(q1); var s1 = Math.Sin(q1);
            var c5Candidates = ComputeQ5(c1, s1, wx, wy, wz, d1, d4, d5);
            foreach (var (q5, c5, s5) in c5Candidates)
            {
                var q6 = Math.Atan2(-t[1] * s1 + t[5] * c1, t[0] * s1 - t[4] * c1);
                if (Math.Abs(s5) < 1e-8)
                    q6 = seedQ6FromSeed(q1, q5, t);

                var wrist = WristCenterTransform(c1, s1, wx, wy, wz, d1);
                var (q2, q3, q4) = SolveArm(wrist, a2, a3, d4, q5, c5, s5, c1, s1);
                if (double.IsNaN(q2)) continue;

                results.Add(NormalizeJoints([q1, q2, q3, q4, q5, q6]));
                results.Add(NormalizeJoints([q1, q2, q3, q4 + Math.PI, q5, q6 + Math.PI]));
            }
        }

        return results;
    }

    private static double seedQ6FromSeed(double q1, double q5, double[] t) =>
        Math.Atan2(t[6], t[10]);

    private static List<(double q5, double c5, double s5)> ComputeQ5(
        double c1, double s1, double wx, double wy, double wz, double d1, double d4, double d5)
    {
        var num = wx * s1 - wy * c1 - d4;
        var den = d5;
        if (Math.Abs(den) < 1e-12) return new List<(double, double, double)>();

        var val = Math.Clamp(num / den, -1, 1);
        var q5a = Math.Acos(val);
        var q5b = -q5a;
        return new List<(double, double, double)>
        {
            (q5a, Math.Cos(q5a), Math.Sin(q5a)),
            (q5b, Math.Cos(q5b), Math.Sin(q5b))
        };
    }

    private static double[] WristCenterTransform(double c1, double s1, double wx, double wy, double wz, double d1)
    {
        var x = c1 * wx + s1 * wy;
        var y = -s1 * wx + c1 * wy;
        var z = wz - d1;
        return [x, y, z];
    }

    private static (double q2, double q3, double q4) SolveArm(
        double[] p, double a2, double a3, double d4, double q5, double c5, double s5, double c1, double s1)
    {
        var x = p[0]; var z = p[2];
        var distSq = x * x + z * z;
        var cos3 = (distSq - a2 * a2 - a3 * a3) / (2 * a2 * a3);
        if (cos3 < -1.0001 || cos3 > 1.0001) return (double.NaN, double.NaN, double.NaN);

        var q3Options = new[] { Math.Acos(Math.Clamp(cos3, -1, 1)), -Math.Acos(Math.Clamp(cos3, -1, 1)) };
        foreach (var q3 in q3Options)
        {
            var k1 = a2 + a3 * Math.Cos(q3);
            var k2 = a3 * Math.Sin(q3);
            var q2 = Math.Atan2(z, x) - Math.Atan2(k2, k1);
            var q4 = Math.Atan2(-Math.Sin(q3), Math.Cos(q3) - 1) - q2 - q3;
            return (q2, q3, q4);
        }

        return (double.NaN, double.NaN, double.NaN);
    }

    private static double[] NormalizeJoints(double[] q)
    {
        for (var i = 0; i < q.Length; i++)
        {
            while (q[i] > Math.PI) q[i] -= 2 * Math.PI;
            while (q[i] < -Math.PI) q[i] += 2 * Math.PI;
        }
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
            max = Math.Max(max, Math.Abs(a[i] - b[i]));
        return max;
    }
}
