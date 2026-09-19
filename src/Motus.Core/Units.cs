namespace Motus.Core;

/// <summary>
/// Motus uses radians for revolute joints, meters for prismatic joints / distance, seconds for time.
/// Stewart leg lengths are meters (<see cref="JointCoordinateUnit.Meters"/>).
/// Legged Family joints are radians (coxa/femur/tibia); not Stewart meters.
/// Aerial Family uses HolonomicSE3 body poses (m + rad); Waypoints Q is not UR MoveJ.
/// </summary>
public static class Units
{
    public const string StewartFamily = "stewart";
    public const string LeggedFamily = "legged";
    /// <summary>Offline SE(3) free-flyer / multirotor hull. Not a flight stack.</summary>
    public const string AerialFamily = "aerial";

    public static bool IsStewart(RobotPreset preset) =>
        string.Equals(preset.Family, StewartFamily, StringComparison.OrdinalIgnoreCase);

    public static bool IsLegged(RobotPreset preset) =>
        string.Equals(preset.Family, LeggedFamily, StringComparison.OrdinalIgnoreCase);

    public static bool IsAerial(RobotPreset preset) =>
        string.Equals(preset.Family, AerialFamily, StringComparison.OrdinalIgnoreCase);

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
