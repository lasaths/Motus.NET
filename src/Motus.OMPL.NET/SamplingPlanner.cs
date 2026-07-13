using Motus.Core;
using Motus.Geometry;

namespace Motus.OMPL.NET;

/// <summary>Registry-driven sampling planner façade replacing direct RrtConnectPlanner use.</summary>
public sealed class SamplingPlanner : IPlanner
{
    private readonly ICollisionChecker? _defaultChecker;
    private readonly SamplingPlannerOptions _options;

    public SamplingPlanner(RobotPreset preset, SamplingPlannerOptions? options = null)
        : this(preset, null, options) { }

    public SamplingPlanner(RobotPreset preset, SerialJointChain? serialChain, SamplingPlannerOptions? options = null)
    {
        _defaultChecker = KinematicsResolver.SupportsModel(preset, serialChain)
            ? serialChain is null
                ? new SphereCollisionChecker(preset)
                : new SphereCollisionChecker(preset, serialChain)
            : null;
        _options = options ?? new SamplingPlannerOptions();
    }

    public SamplingPlanner(ICollisionChecker checker, SamplingPlannerOptions? options = null)
    {
        _defaultChecker = checker;
        _options = options ?? new SamplingPlannerOptions();
    }

    public static SamplingPlanner Create(ICollisionChecker checker, SamplingPlannerOptions options) =>
        new(checker, options);

    public static SamplingPlanner Create(RobotPreset preset, SamplingPlannerOptions? options = null) =>
        new(preset, options);

    public PlanningResult Plan(PlanningRequest request)
    {
        if (_options.StepRadians <= 0)
            return PlanningResult.Failed(new[]
            {
                new PlanningMessage(
                    PlanningMessageCodes.InvalidOptions,
                    "SamplingPlannerOptions.StepRadians must be positive.",
                    PlanningMessageSeverity.Error)
            });

        return SamplingPlannerRegistry.Dispatch(request, _options, _defaultChecker);
    }
}
