namespace Motus.Construction;

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
    /// The robot starts with L on the pin column at A (clamp height 0.09 m, heading 60°). The stock lies flat next to A,
    /// outside the pit: centre (−0.36, 0.32), along z, i.e. 0.06 m outboard of A and 0.21 m off the truss plane on the
    /// arm's side. Chosen by a stock sweep (24 positions): the in-plane stock of warren-pit-1 left 0 targeted landings on
    /// triangle 1 with both stock and place IK for M–T2 / T1–T2 / M–D; this one gives 10 / 117 / 112 and still builds
    /// triangle 1 from the A column in the order A–T1, T1–M, A–M.
    /// </summary>
    public static BambooTrussLayout.Site Chosen { get; } = Make(0.24, 0.03, 0.19, 0.09, 60, -0.06, 0.32);
}
