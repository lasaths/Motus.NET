namespace Motus.Geometry;

/// <summary>Analytic space/body Jacobians for PoE chains (Lynch &amp; Park §5.1 / §5.6).</summary>
public static class PoEJacobian
{
    /// <summary>Space Jacobian Js(θ): columns Ad(e^[S1]θ1…e^[S_{i-1}]θ_{i-1}) S_i.</summary>
    public static double[,] JacobianSpace(IReadOnlyList<double[]> spaceScrews, IReadOnlyList<double> theta)
    {
        var n = spaceScrews.Count;
        if (theta.Count != n)
            throw new ArgumentException($"Expected {n} joints, got {theta.Count}.");

        var js = new double[6, n];
        WriteColumn(js, 0, spaceScrews[0]);
        var t = Transforms.Identity();
        for (var i = 1; i < n; i++)
        {
            t = Transforms.Multiply(t, ScrewMath.MatrixExp6(spaceScrews[i - 1], theta[i - 1]));
            WriteColumn(js, i, ScrewMath.AdjointMultiply(ScrewMath.Adjoint(t), spaceScrews[i]));
        }
        return js;
    }

    /// <summary>Body Jacobian Jb(θ): columns Ad(e^-[Bn]θn…e^-[B_{i+1}]θ_{i+1}) B_i.</summary>
    public static double[,] JacobianBody(IReadOnlyList<double[]> bodyScrews, IReadOnlyList<double> theta)
    {
        var n = bodyScrews.Count;
        if (theta.Count != n)
            throw new ArgumentException($"Expected {n} joints, got {theta.Count}.");

        var jb = new double[6, n];
        WriteColumn(jb, n - 1, bodyScrews[n - 1]);
        var t = Transforms.Identity();
        for (var i = n - 2; i >= 0; i--)
        {
            t = Transforms.Multiply(ScrewMath.MatrixExp6(bodyScrews[i + 1], -theta[i + 1]), t);
            WriteColumn(jb, i, ScrewMath.AdjointMultiply(ScrewMath.Adjoint(t), bodyScrews[i]));
        }
        return jb;
    }

    public static double[,] JacobianSpace(ProductOfExponentials poe, IReadOnlyList<double> theta) =>
        JacobianSpace(poe.SpaceScrews, theta);

    public static double[,] JacobianBody(ProductOfExponentials poe, IReadOnlyList<double> theta) =>
        JacobianBody(poe.BodyScrews, theta);

    /// <summary>Frobenius-ish scale proxy for singularity: max |Jij|.</summary>
    public static double MaxAbsEntry(double[,] j)
    {
        var max = 0.0;
        var rows = j.GetLength(0);
        var cols = j.GetLength(1);
        for (var r = 0; r < rows; r++)
        for (var c = 0; c < cols; c++)
            max = Math.Max(max, Math.Abs(j[r, c]));
        return max;
    }

    /// <summary>Rough condition estimate via √(λmax/λmin) of J Jᵀ (6×6) using power iteration lite.</summary>
    public static double EstimateConditionJjT(double[,] j)
    {
        var m = j.GetLength(0);
        var n = j.GetLength(1);
        var a = new double[m, m];
        for (var i = 0; i < m; i++)
        for (var k = 0; k < m; k++)
        {
            var sum = 0.0;
            for (var c = 0; c < n; c++)
                sum += j[i, c] * j[k, c];
            a[i, k] = sum;
        }

        // Gershgorin: λmax ≤ max row sum; λmin ≥ max(0, min diag − offdiag sum)
        var lambdaMax = 0.0;
        var lambdaMin = double.PositiveInfinity;
        for (var i = 0; i < m; i++)
        {
            var rowSum = 0.0;
            var off = 0.0;
            for (var k = 0; k < m; k++)
            {
                var v = Math.Abs(a[i, k]);
                rowSum += v;
                if (k != i) off += v;
            }
            lambdaMax = Math.Max(lambdaMax, rowSum);
            lambdaMin = Math.Min(lambdaMin, Math.Max(0, Math.Abs(a[i, i]) - off));
        }
        if (lambdaMin < 1e-16) return double.PositiveInfinity;
        return Math.Sqrt(lambdaMax / lambdaMin);
    }

    private static void WriteColumn(double[,] j, int col, double[] v6)
    {
        for (var r = 0; r < 6; r++)
            j[r, col] = v6[r];
    }
}
