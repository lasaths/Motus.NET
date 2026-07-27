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
}
