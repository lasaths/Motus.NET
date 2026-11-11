namespace Motus.Core;

/// <summary>
/// One joint-space segment, seen by <see cref="SegmentCertifier.Certify"/>. The robot gives the motion bound and the
/// clear check. Segment fraction t runs from 0 (start pose) to 1 (end pose).
/// </summary>
public interface ICertifiableSegment
{
    /// <summary>Motion bound ε (m): no robot point moves more than ε·|t − s| between fractions t and s.</summary>
    double SweepBound();

    /// <summary>True when the pose at fraction <paramref name="t"/> is clear of every obstacle grown by <paramref name="growM"/> (exact test).</summary>
    bool IsClear(double t, double growM);
}

/// <summary>
/// Proves a joint-space segment contact-free between its samples by conservative advancement in threshold form
/// (Schwarzer, Saha &amp; Latombe 2005). An interval [a, b] is clear when the pose at its midpoint is clear of every
/// obstacle grown by ε·(b − a)/2. Otherwise the interval is bisected.
/// </summary>
public static class SegmentCertifier
{
    /// <summary>Smallest obstacle growth step (m). Growths are rounded up to 1e-4·2^k so grown obstacles can be reused.</summary>
    public const double GrowQuantumM = 1e-4;

    /// <summary>
    /// True when the segment is proven free of contact between its end poses (the ends are the caller's samples).
    /// Bisection stops at <paramref name="maxDepth"/> levels or <paramref name="maxChecks"/> clear checks. Hitting either
    /// leaves the segment uncertified (false), never accepted. An interval that needs a growth above
    /// <paramref name="maxGrowM"/> is bisected without a check.
    /// </summary>
    public static bool Certify(ICertifiableSegment segment, int maxDepth, int maxChecks, out int checks, double maxGrowM = 0.25)
    {
        checks = 0;
        var eps = segment.SweepBound();
        if (eps == 0) return true;
        if (double.IsInfinity(eps) || double.IsNaN(eps)) return false;
        var stack = new Stack<(double A, double B, int Depth)>();
        stack.Push((0, 1, 0));
        while (stack.Count > 0)
        {
            var (a, b, depth) = stack.Pop();
            var grow = Quantize(eps * (b - a) / 2);
            var clear = false;
            if (grow <= maxGrowM)
            {
                if (++checks > maxChecks) return false;
                clear = segment.IsClear((a + b) / 2, grow);
            }
            if (clear) continue;
            if (depth >= maxDepth) return false;
            var mid = (a + b) / 2;
            stack.Push((mid, b, depth + 1));
            stack.Push((a, mid, depth + 1));
        }
        return true;
    }

    /// <summary>Growth rounded up to 1e-4·2^k m (reuses grown obstacles across intervals).</summary>
    public static double Quantize(double d)
    {
        if (d <= GrowQuantumM) return GrowQuantumM;
        return GrowQuantumM * Math.Pow(2, Math.Ceiling(Math.Log2(d / GrowQuantumM) - 1e-12));
    }

    /// <summary>
    /// Motion bound ε (m) of a serial chain of revolute joints. <paramref name="axis"/>[j] is a point on joint j's axis
    /// (n points). Body i (0..n) sits between joint i − 1 and joint i: body 0 is the base link, body n is the tip link.
    /// <paramref name="spheres"/> are bounding spheres tagged with their body. With <paramref name="fixedAtTip"/> false,
    /// body 0 is fixed and joint j turns bodies i &gt; j; else body n is fixed and joint j turns bodies i ≤ j.
    /// Joint j adds |Δq_j|·(chain of axis points from j to the body's pivot + reach of the body from its pivot).
    /// All distances are between points fixed in one body, so ε holds for every configuration of the segment.
    /// </summary>
    public static double ChainSweepBound(IReadOnlyList<double[]> axis, IEnumerable<(int Body, double[] Center, double Radius)> spheres,
        IReadOnlyList<double> deltaRad, bool fixedAtTip = false)
    {
        var n = axis.Count;
        var reach = new double[n + 1];
        Array.Fill(reach, -1);
        foreach (var (i, c, r) in spheres)
        {
            var pivot = fixedAtTip ? i : i - 1;
            if (pivot < 0 || pivot >= n) continue; // a body that never moves
            reach[i] = Math.Max(reach[i], Dist(c, axis[pivot]) + r);
        }
        var worst = 0.0;
        for (var i = 0; i <= n; i++)
        {
            if (reach[i] < 0) continue;
            var delta = 0.0;
            for (var j = 0; j < n; j++)
            {
                if (fixedAtTip ? i - 1 >= j : i - 1 < j) continue;
                var chain = 0.0;
                var (lo, hi) = fixedAtTip ? (i, j) : (j, i - 1);
                for (var k = lo; k < hi; k++) chain += Dist(axis[k], axis[k + 1]);
                delta += Math.Abs(deltaRad[j]) * (chain + reach[i]);
            }
            worst = Math.Max(worst, delta);
        }
        return worst;
    }

    static double Dist(double[] a, double[] b) => Math.Sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}
