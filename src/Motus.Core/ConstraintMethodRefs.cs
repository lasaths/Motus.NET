namespace Motus.Core;

/// <summary>References for planner constraint-validation seams.</summary>
public static class ConstraintMethodRefs
{
    public const string OmplSucan2012Doi = "10.1109/MRA.2012.2205651";

    public static string DescribeStack() =>
        $"TCP path constraints are Motus-managed validity checks with OMPL-style planner-state validity seam prior art (Sucan et al. 2012, DOI {OmplSucan2012Doi}).";
}
