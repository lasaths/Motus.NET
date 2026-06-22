namespace Motus.Geometry;

/// <summary>Standard DH link: theta offset applied to joint angle at runtime.</summary>
public readonly record struct DhLink(double A, double D, double Alpha, double ThetaOffset = 0);

public sealed class KinematicsChain
{
    public DhLink[] Links { get; }
    public double[] LinkRadiiMeters { get; }

    public KinematicsChain(DhLink[] links, double[]? linkRadiiMeters = null)
    {
        Links = links;
        LinkRadiiMeters = linkRadiiMeters ?? Enumerable.Repeat(0.08, links.Length).ToArray();
    }
}
