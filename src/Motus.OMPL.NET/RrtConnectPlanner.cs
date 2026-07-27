using Motus.Core;
using Motus.Geometry;

namespace Motus.OMPL.NET;

/// <summary>Backward-compatible wrapper; prefer <see cref="SamplingPlanner"/>.</summary>
[Obsolete("Use SamplingPlanner with SamplingPlannerOptions instead.")]
public sealed class RrtConnectPlanner : IPlanner
{
    private readonly SamplingPlanner _inner;

    public RrtConnectPlanner(RobotPreset preset, RrtConnectOptions? options = null)
        : this(preset, null, options) { }

    public RrtConnectPlanner(RobotPreset preset, SerialJointChain? serialChain, RrtConnectOptions? options = null)
    {
        _inner = new SamplingPlanner(preset, serialChain, CloneOptions(options, SamplingPlannerId.RrtConnect));
    }

    public RrtConnectPlanner(ICollisionChecker checker, RrtConnectOptions? options = null)
    {
        _inner = SamplingPlanner.Create(checker, CloneOptions(options, SamplingPlannerId.RrtConnect));
    }

    private static SamplingPlannerOptions CloneOptions(RrtConnectOptions? options, SamplingPlannerId plannerId)
    {
        if (options is null) return new SamplingPlannerOptions { PlannerId = plannerId };
        return new SamplingPlannerOptions
        {
            MaxIterations = options.MaxIterations,
            MaxPlanTimeSeconds = options.MaxPlanTimeSeconds,
            StepRadians = options.StepRadians,
            GoalBias = options.GoalBias,
            ConnectThresholdRadians = options.ConnectThresholdRadians,
            RandomSeed = options.RandomSeed,
            MaxPathStates = options.MaxPathStates,
            PrmStarGamma = options.PrmStarGamma,
            ChompIterations = options.ChompIterations,
            ChompLearningRate = options.ChompLearningRate,
            ChompFiniteDifferenceStep = options.ChompFiniteDifferenceStep,
            PlannerId = plannerId,
            PreferManaged = options.PreferManaged,
            ShouldCancel = options.ShouldCancel,
            ReportIteration = options.ReportIteration
        };
    }

    public PlanningResult Plan(PlanningRequest request) => _inner.Plan(request);
}
