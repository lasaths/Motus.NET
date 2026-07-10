using Motus.Core;

namespace Motus.Geometry;

/// <summary>Allocation-free DH FK for collision hot paths (link xyz only).</summary>
internal static class FastDhFk
{
    public static void ComputeLinkWorldPositions(
        KinematicsChain chain,
        ReadOnlySpan<double> joints,
        double[] baseM,
        Span<double> xyz,
        double[] matA,
        double[] matB,
        double[] matC)
    {
        var links = chain.Links;
        Array.Copy(baseM, matA, 16);
        var current = matA;
        var local = matB;
        var result = matC;
        for (var i = 0; i < links.Length; i++)
        {
            var link = links[i];
            Transforms.FromDhInto(local, joints[i] + link.ThetaOffset, link.D, link.A, link.Alpha);
            Transforms.MultiplyInto(result, current, local);
            (current, result) = (result, current);
            var o = i * 3;
            xyz[o] = current[3];
            xyz[o + 1] = current[7];
            xyz[o + 2] = current[11];
        }
    }
}
