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
    /// 0.30 m canes (previous shipped site): builds the lower chord + diagL only — sep and diagR have no Motus-clear
    /// stock grasp / carry from any allowed support. Kept for sweeps and comparison.
    /// </summary>
    public static BambooTrussLayout.Site Site030 { get; } = new(
        0.30,
        [St("stake-S1", -0.295, 0.060, 0.18, 90), St("stake-S3", -0.295, -0.040, 0.21, 70)],
        "stake-S1",
        -0.3857, 0.0165, 0, 0.12,
        "stake-S3@0.13", "stake-S3",
        new Dictionary<string, string>());

    /// <summary>Shift of everything outboard of A when A moves in with a shorter cane: HalfSpan(0.30) − HalfSpan(0.25).</summary>
    static readonly double Dx025 = Math.Sqrt(0.75) * (0.30 - 0.25);

    /// <summary>
    /// 0.25 m canes — the whole bridge. Same stakes / stock as the 0.30 m site, moved in with anchor A by
    /// HalfSpan(0.30) − HalfSpan(0.25) = 43.3 mm (still outboard of the A pad). From S3 the arm builds all five members
    /// in the order baseL, baseR, diagL, diagR, sep (diagR before sep: with sep standing first, R ends wedged
    /// between the canes at C). Every placement must leave R a Motus-clear park.
    /// </summary>
    public static BambooTrussLayout.Site Chosen { get; } = new(
        0.25,
        [St("stake-S1", -0.295 + Dx025, 0.060, 0.18, 90), St("stake-S3", -0.295 + Dx025, -0.040, 0.21, 70)],
        "stake-S1",
        -0.3857 + Dx025, 0.0165, 0, 0.12,
        "stake-S3@0.13", "stake-S3",
        new Dictionary<string, string>());
}
