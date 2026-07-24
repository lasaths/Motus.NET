namespace Motus.Core;

/// <summary>
/// Motus uses radians for revolute joints, meters for prismatic joints / distance, seconds for time.
/// Stewart leg lengths are meters (<see cref="JointCoordinateUnit.Meters"/>).
/// </summary>
public static class Units
{
    public const string StewartFamily = "stewart";

    public static bool IsStewart(RobotPreset preset) =>
        string.Equals(preset.Family, StewartFamily, StringComparison.OrdinalIgnoreCase);

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
