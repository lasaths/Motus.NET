namespace Motus.Geometry;

/// <summary>Traceability anchors for Stewart/Gough platform kinematics and singularity analysis.</summary>
public static class StewartMethodRefs
{
    /// <summary>Merlet, Parallel Robots, Springer, DOI 10.1007/1-4020-4133-0.</summary>
    public const string MerletParallelRobotsDoi = "10.1007/1-4020-4133-0";

    /// <summary>Dasgupta &amp; Mruthyunjaya, Stewart platform singularities, DOI 10.1016/S0094-114X(99)00006-3.</summary>
    public const string DasguptaMruthyunjaya1999Doi = "10.1016/S0094-114X(99)00006-3";

    public static string DescribeStack() =>
        "Stewart/Gough IK/FK and stroke-space planning; refs: Merlet doi:" + MerletParallelRobotsDoi +
        "; Dasgupta&Mruthyunjaya singularity analysis doi:" + DasguptaMruthyunjaya1999Doi + ".";
}
