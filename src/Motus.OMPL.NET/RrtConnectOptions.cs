namespace Motus.OMPL.NET;

public class SamplingPlannerOptions
{
    public int MaxIterations { get; init; } = 4000;
    /// <summary>When &gt; 0, native OMPL uses time budget instead of iteration count.</summary>
    public double MaxPlanTimeSeconds { get; init; }
    public double StepRadians { get; init; } = 0.12;
    public double GoalBias { get; init; } = 0.08;
    public double ConnectThresholdRadians { get; init; } = 0.2;
    public int RandomSeed { get; init; } = 42;
    public int MaxPathStates { get; init; } = 256;
    public SamplingPlannerId PlannerId { get; init; } = SamplingPlannerId.RrtConnect;
    /// <summary>When true, skip native OMPL even if motus_native is available.</summary>
    public bool PreferManaged { get; init; }
    public Func<bool>? ShouldCancel { get; init; }
    /// <summary>Managed RRT loop only; called periodically with (iteration, maxIterations).</summary>
    public Action<int, int>? ReportIteration { get; init; }
}

/// <summary>Backward-compatible alias.</summary>
public sealed class RrtConnectOptions : SamplingPlannerOptions;

/// <summary>Backward-compatible alias.</summary>
public sealed class OmplPlannerOptions : SamplingPlannerOptions;
