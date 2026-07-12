namespace Motus.OMPL.NET;

/// <summary>Stable public planner IDs (string-parseable for Grasshopper).</summary>
public enum SamplingPlannerId
{
    RrtConnect = 0,
    RrtStar = 1,
    Aorrtc = 2,
    Lbkpiece = 3,
    AitStar = 4,
    EitStar = 5,
    BlitStar = 6,
    ParallelRace = 7,
}

/// <summary>Backward-compatible alias; prefer <see cref="SamplingPlannerId"/>.</summary>
public enum OmplPlannerId
{
    RrtConnect = 0,
    RrtStar = 1,
}
