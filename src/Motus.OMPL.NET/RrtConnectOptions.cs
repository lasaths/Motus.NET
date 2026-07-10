namespace Motus.OMPL.NET;

public enum OmplPlannerId
{
    RrtConnect = 0,
    RrtStar = 1
}

public class OmplPlannerOptions
{
    public int MaxIterations { get; init; } = 4000;
    /// <summary>When &gt; 0, native OMPL uses time budget instead of iteration count.</summary>
    public double MaxPlanTimeSeconds { get; init; } = 0;
    public double StepRadians { get; init; } = 0.12;
    public double GoalBias { get; init; } = 0.08;
    public double ConnectThresholdRadians { get; init; } = 0.2;
    public int RandomSeed { get; init; } = 42;
    public int MaxPathStates { get; init; } = 256;
    public OmplPlannerId PlannerId { get; init; } = OmplPlannerId.RrtConnect;
    /// <summary>When true, skip native OMPL even if motus_native is available.</summary>
    public bool PreferManaged { get; init; }
    public Func<bool>? ShouldCancel { get; init; }
}

/// <summary>Backward-compatible alias.</summary>
public sealed class RrtConnectOptions : OmplPlannerOptions;
