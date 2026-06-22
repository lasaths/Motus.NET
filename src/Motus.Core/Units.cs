namespace Motus.Core;

/// <summary>Motus uses radians for joints, seconds for time, meters for distance.</summary>
public static class Units
{
    public const double DegreesToRadians = Math.PI / 180.0;
    public const double RadiansToDegrees = 180.0 / Math.PI;

    public static double ToRadians(double degrees) => degrees * DegreesToRadians;
    public static double ToDegrees(double radians) => radians * RadiansToDegrees;
    public static double[] ToRadians(double[] degrees)
    {
        var r = new double[degrees.Length];
        for (var i = 0; i < degrees.Length; i++) r[i] = ToRadians(degrees[i]);
        return r;
    }
    public static double[] ToDegrees(double[] radians)
    {
        var d = new double[radians.Length];
        for (var i = 0; i < radians.Length; i++) d[i] = ToDegrees(radians[i]);
        return d;
    }
}
