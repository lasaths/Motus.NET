namespace Motus.OMPL.NET;

/// <summary>Sampling/smoothing planner method references.</summary>
public static class PlanningMethodRefs
{
    public const string KaramanFrazzoli2011PrmStarDoi = "10.1177/0278364911406761";
    public const string Zucker2013ChompDoi = "10.1177/0278364913488805";
    public const string KuffnerLavalle2000RrtConnectLabel = "RRT-Connect, Kuffner & LaValle, ICRA 2000";

    public static string DescribePrmStar() =>
        $"PRM*: variable-radius probabilistic roadmap gamma(log n/n)^(1/d) (Karaman & Frazzoli 2011, DOI {KaramanFrazzoli2011PrmStarDoi}).";

    /// <remarks>Alternatives for future native/full-stack work: TrajOpt and GPMP2.</remarks>
    public static string DescribeChompLite() =>
        $"CHOMP-lite smoother: finite-difference smoothness with collision penalty checks (Zucker et al. 2013, DOI {Zucker2013ChompDoi}).";
}
