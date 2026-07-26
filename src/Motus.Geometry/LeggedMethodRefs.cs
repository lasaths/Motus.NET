namespace Motus.Geometry;

/// <summary>
/// Traceability anchors for legged IK / gait / stability (NASA-style method provenance).
/// Prefer peer-reviewed DOI sources; classify adaptations explicitly in call-site docs.
/// </summary>
/// <remarks>
/// Motus.NET owns the algorithms. Grasshopper must remain a thin Rhino I/O wrapper.
/// </remarks>
public static class LeggedMethodRefs
{
    /// <summary>Lynch &amp; Park, Modern Robotics (Cambridge, 2017). Analytic / numerical IK foundations.</summary>
    public const string LynchPark2017Doi = "10.1017/9781316095072";

    /// <summary>Aristidou &amp; Lasenby, FABRIK (Graphical Models, 2011). Iterative n-link position IK.</summary>
    public const string AristidouLasenby2011FabrikDoi = "10.1016/j.gmod.2011.05.003";

    /// <summary>Aristidou et al., Extending FABRIK with model constraints (CAVW, 2016).</summary>
    public const string AristidouEtAl2016ConstrainedFabrikDoi = "10.1002/cav.1630";

    /// <summary>Aristidou et al., IK techniques survey (CGF, 2018).</summary>
    public const string AristidouEtAl2018IkSurveyDoi = "10.1111/cgf.13310";

    /// <summary>McGhee &amp; Frank, quadruped creeping gait stability (Math. Biosciences, 1968).</summary>
    public const string McGheeFrank1968Doi = "10.1016/0025-5564(68)90041-2";

    /// <summary>Song &amp; Waldron, gait study / wave gaits (IJRR, 1987).</summary>
    public const string SongWaldron1987Doi = "10.1177/027836498700600205";

    /// <summary>Bretl &amp; Lall, testing static equilibrium (IEEE T-RO, 2008).</summary>
    public const string BretlLall2008Doi = "10.1109/TRO.2008.2001360";

    /// <summary>
    /// One-line provenance for Status / logs (algorithms + DOIs).
    /// </summary>
    public static string DescribeStack() =>
        "LegIk3R=analytic planar 2R after coxa (Lynch&Park doi:" + LynchPark2017Doi +
        "); not FABRIK (Aristidou&Lasenby doi:" + AristidouLasenby2011FabrikDoi +
        "). Gait=duty-factor swing groups (Song&Waldron doi:" + SongWaldron1987Doi +
        ") + creeping stance plants (McGhee&Frank doi:" + McGheeFrank1968Doi +
        "). SSM=support-polygon CoM test (McGhee&Frank); dynamic/wrench limits → Bretl&Lall doi:" +
        BretlLall2008Doi + ".";
}
