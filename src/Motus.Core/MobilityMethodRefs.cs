namespace Motus.Core;

/// <summary>Traceability anchors for mobile-base planning models.</summary>
public static class MobilityMethodRefs
{
    /// <summary>
    /// LaValle, Planning Algorithms, Cambridge University Press, 2006.
    /// Open text: planning.cs.uiuc.edu.
    /// </summary>
    public const string LaVallePlanningAlgorithmsUrl = "http://planning.cs.uiuc.edu/";

    public static string DescribeHolonomicSe2() =>
        "Mobility=HolonomicSE2 sampled as x/y/yaw (SE(2)); bounds in meters/radians. " +
        "Reference: LaValle, Planning Algorithms (2006), " + LaVallePlanningAlgorithmsUrl + ".";

    public static string DescribeHolonomicSe3() =>
        "Mobility=HolonomicSE3 sampled as x/y/z/roll/pitch/yaw (SE(3) free-flyer); " +
        "RPY fixed-axis XYZ with singularity Status near pitch ±π/2; not a flight controller; " +
        "body poses ≠ UR MoveJ — trajectory Q is empty or unused for pure aerial. " +
        "Reference: LaValle, Planning Algorithms (2006), " + LaVallePlanningAlgorithmsUrl + ".";
}
