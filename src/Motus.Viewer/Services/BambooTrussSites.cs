namespace Motus.Viewer.Services;

/// <summary>
/// Sites (cane length + outboard supports + stock + walk) found by the offline length sweep for /bamboo-truss.
/// <see cref="Chosen"/> is the shipped one; see the README for the sweep table.
/// </summary>
public static class SiteDefaults
{
    static BambooTrussLayout.Stake St(string n, double x, double z, double clampY, double heading) =>
        new(n, x, z, clampY, clampY + BambooTrussLayout.StakeTopAbove, heading);

    /// <summary>
    /// 0.30 m canes — the longest round length at which the lower chord A–B–D can be built (at 0.35 m and longer no
    /// allowed support reaches baseR). Two outboard stakes 5 mm outboard of the A pad edge: the robot starts on S1,
    /// walks to S3 (R clamps S3 low at 13 cm as the base while L slides off S1 and onto S3) and builds baseL, baseR
    /// and diagL from S3. Stock lies flat on two pegs just outboard of A. sep and diagR are reported by the planner
    /// (no Motus-clear place from any allowed support).
    /// </summary>
    public static BambooTrussLayout.Site Chosen { get; } = new(
        0.30,
        [St("stake-S1", -0.295, 0.060, 0.18, 90), St("stake-S3", -0.295, -0.040, 0.21, 70)],
        "stake-S1",
        -0.3857, 0.0165, 0, 0.12,
        "stake-S3@0.13", "stake-S3",
        new Dictionary<string, string>());
}
