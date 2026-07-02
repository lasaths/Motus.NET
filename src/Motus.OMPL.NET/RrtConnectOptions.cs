namespace Motus.OMPL.NET;

public sealed class RrtConnectOptions
{
    public int MaxIterations { get; init; } = 4000;
    public double StepRadians { get; init; } = 0.12;
    public double GoalBias { get; init; } = 0.08;
    public double ConnectThresholdRadians { get; init; } = 0.2;
    public int RandomSeed { get; init; } = 42;
    public int MaxPathStates { get; init; } = 256;
    public Func<bool>? ShouldCancel { get; init; }
}
