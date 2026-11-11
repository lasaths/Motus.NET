namespace Motus.Core.Tests;

/// <summary>Generic segment certifier with a plain planar 2-link arm (links are line segments) and one box.</summary>
public class SegmentCertifierTests
{
    const double L1 = 0.5, L2 = 0.5;

    sealed class TwoLinkArm(double[] qa, double[] qb, double[] boxMin, double[] boxMax) : ICertifiableSegment
    {
        public double SweepBound()
        {
            var (e, t) = Points(qa);
            var axis = new[] { new[] { 0.0, 0, 0 }, e };
            var spheres = new[] { (1, Mid(new[] { 0.0, 0, 0 }, e), L1 / 2), (2, Mid(e, t), L2 / 2) };
            return SegmentCertifier.ChainSweepBound(axis, spheres, new[] { qb[0] - qa[0], qb[1] - qa[1] });
        }

        public bool IsClear(double t, double growM)
        {
            var q = new[] { qa[0] + (qb[0] - qa[0]) * t, qa[1] + (qb[1] - qa[1]) * t };
            var (e, tip) = Points(q);
            var lo = boxMin.Select(v => v - growM).ToArray();
            var hi = boxMax.Select(v => v + growM).ToArray();
            return !Hits(new[] { 0.0, 0, 0 }, e, lo, hi) && !Hits(e, tip, lo, hi);
        }
    }

    static (double[] Elbow, double[] Tip) Points(double[] q)
    {
        var e = new[] { L1 * Math.Cos(q[0]), L1 * Math.Sin(q[0]), 0 };
        return (e, new[] { e[0] + L2 * Math.Cos(q[0] + q[1]), e[1] + L2 * Math.Sin(q[0] + q[1]), 0 });
    }

    static double[] Mid(double[] a, double[] b) => new[] { (a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2 };

    /// <summary>Exact line segment ↔ axis-aligned box test (slab method).</summary>
    static bool Hits(double[] p, double[] r, double[] lo, double[] hi)
    {
        double t0 = 0, t1 = 1;
        for (var k = 0; k < 3; k++)
        {
            var d = r[k] - p[k];
            if (Math.Abs(d) < 1e-15) { if (p[k] < lo[k] || p[k] > hi[k]) return false; continue; }
            var a = (lo[k] - p[k]) / d; var b = (hi[k] - p[k]) / d;
            if (a > b) (a, b) = (b, a);
            t0 = Math.Max(t0, a); t1 = Math.Min(t1, b);
            if (t0 > t1) return false;
        }
        return true;
    }

    [Fact]
    public void Certify_RejectsSegmentThroughBox_AndCertifiesClearSegment()
    {
        var qa = new[] { 0.0, 0 };
        var qb = new[] { Math.PI / 2, 0 };
        var boxMin = new[] { 0.55, 0.55, -0.05 };
        var boxMax = new[] { 0.65, 0.65, 0.05 };
        var through = new TwoLinkArm(qa, qb, boxMin, boxMax);
        Assert.True(through.IsClear(0, 0) && through.IsClear(1, 0)); // both samples are clear
        Assert.False(SegmentCertifier.Certify(through, 18, 20000, out var hitChecks));
        Assert.True(hitChecks > 0);

        var clear = new TwoLinkArm(qa, qb, new[] { 1.5, 1.5, -0.05 }, new[] { 1.6, 1.6, 0.05 });
        Assert.True(SegmentCertifier.Certify(clear, 18, 20000, out var clearChecks));
        Assert.InRange(clearChecks, 1, 20000);
        Assert.Equal(Math.PI / 2 * (L1 + L2), clear.SweepBound(), 12); // tip moves at most |Δq|·(L1 + L2)
    }
}
