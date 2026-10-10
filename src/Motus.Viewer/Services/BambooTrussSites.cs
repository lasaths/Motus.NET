namespace Motus.Viewer.Services;

/// <summary>
/// Sites (cane length + node gap + supports + stock + walk) for /bamboo-truss. <see cref="Chosen"/> is the shipped one.
/// </summary>
public static class SiteDefaults
{
    /// <summary>Anchor column under a support node: clampable from the ground up to just under the node connector.</summary>
    public static BambooTrussLayout.Stake Anchor(string name, double x, double nodeY, double clampY, double heading) =>
        new(name, x, BambooTrussLayout.PlaneZ, clampY, nodeY - BambooTrussLayout.HubHalf, heading);

    public static BambooTrussLayout.Site Make(double cane, double gap, double nodeY, double clampY, double heading,
        double feedDx, double feedZ, double feedYaw = 0, double pitDepth = 0.25, double feedTop = 0.12)
    {
        var p = cane + 2 * gap;
        return new BambooTrussLayout.Site(
            cane, gap, nodeY, pitDepth,
            [Anchor("anchor-A", -p, nodeY, clampY, heading), Anchor("anchor-D", p, nodeY, clampY, 180 - heading)],
            "anchor-A",
            -p + feedDx, feedZ, feedYaw, feedTop,
            "placed-AM", "anchor-A",
            new Dictionary<string, string>());
    }

    /// <summary>
    /// Shipped site: 0.24 m canes, 30 mm node gap (panel 0.30 m), nodes A / M / D 0.19 m above the ground, a 0.25 m pit.
    /// The robot starts with L on the pin column at A (clamp height 0.09 m, heading 60°); the stock lies flat in the
    /// truss plane 0.12 m outboard of A (on its pegs, outside the pit). Chosen by a reach + real-planner sweep
    /// (39 IK-feasible sites × build orders): triangle 1 builds in the order A–T1, T1–M, A–M (A–M first leaves no
    /// Motus-clear place for T1–M: the in-plane arm hits A–M).
    /// </summary>
    public static BambooTrussLayout.Site Chosen { get; } = Make(0.24, 0.03, 0.19, 0.09, 60, -0.12, 0.11);
}
