namespace Motus.Geometry;

/// <summary>
/// Workspace reach sampling over driver joint limits.
/// Always capped by <c>maxSamples</c> — never a full joint-grid product.
/// </summary>
public static class ReachSampling
{
    /// <summary>
    /// Fills interleaved xyz TCP samples into caller-owned <paramref name="xyz"/> (length &gt;= 3 * maxSamples).
    /// Uses a Halton sequence over driver limits (stratified low-discrepancy), capped at <paramref name="maxSamples"/>.
    /// Returns the number of samples written.
    /// </summary>
    public static int FillTcpPointsInto(
        TreeForwardKinematics fk,
        int tipLinkIndex,
        IReadOnlyList<double> lower,
        IReadOnlyList<double> upper,
        double[] xyz,
        int maxSamples)
    {
        if (maxSamples < 0)
            throw new ArgumentOutOfRangeException(nameof(maxSamples));
        if (fk.DriverCount != lower.Count || fk.DriverCount != upper.Count)
            throw new ArgumentException("lower/upper must match driver count.");
        if (tipLinkIndex < 0 || tipLinkIndex >= fk.LinkCount)
            throw new ArgumentOutOfRangeException(nameof(tipLinkIndex));
        if (xyz.Length < maxSamples * 3)
            throw new ArgumentException($"xyz length must be >= {maxSamples * 3}.");

        if (maxSamples == 0)
            return 0;

        var n = fk.DriverCount;
        var q = new double[n];
        var mats = new double[fk.LinkCount][];
        for (var i = 0; i < mats.Length; i++)
            mats[i] = new double[16];

        // ponytail: Halton over [0,1]^n mapped to limits — O(maxSamples), never ∏ bins
        var primes = n <= s_primes.Length ? s_primes : ExtendPrimes(n);
        for (var s = 0; s < maxSamples; s++)
        {
            for (var d = 0; d < n; d++)
            {
                var u = Halton(s + 1, primes[d]);
                q[d] = lower[d] + u * (upper[d] - lower[d]);
            }

            fk.ComputeTipTranslationInto(q, tipLinkIndex, mats, out var x, out var y, out var z);
            var o = s * 3;
            xyz[o] = x;
            xyz[o + 1] = y;
            xyz[o + 2] = z;
        }

        return maxSamples;
    }

    private static readonly int[] s_primes =
    [
        2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53
    ];

    private static int[] ExtendPrimes(int n)
    {
        var p = new int[n];
        Array.Copy(s_primes, p, s_primes.Length);
        var v = s_primes[^1] + 1;
        for (var i = s_primes.Length; i < n; i++)
        {
            while (!IsPrime(v)) v++;
            p[i] = v++;
        }
        return p;
    }

    private static bool IsPrime(int v)
    {
        if (v < 2) return false;
        for (var i = 2; i * i <= v; i++)
            if (v % i == 0) return false;
        return true;
    }

    private static double Halton(int index, int basePrime)
    {
        var f = 1.0;
        var r = 0.0;
        var i = index;
        while (i > 0)
        {
            f /= basePrime;
            r += f * (i % basePrime);
            i /= basePrime;
        }
        return r;
    }
}
