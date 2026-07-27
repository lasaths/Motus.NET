using Motus.Core;
using Motus.Geometry;

namespace Motus.OMPL.NET;

internal static class ManagedChompSmooth
{
    internal static PlanningResult Plan(
        PlanningRequest request,
        SamplingPlannerOptions options,
        ICollisionChecker? defaultChecker,
        SerialJointChain? serialChain)
    {
        var checker = PlanningPipeline.ResolveChecker(request, defaultChecker);
        var robot = request.Robot;
        var scene = request.Options.CollisionScene ?? new CollisionScene();
        var spaceFail = PlanningPipeline.TryBuildPlanSpace(request, out var space);
        if (spaceFail is not null) return spaceFail;
        var checkerAvailabilityFail = PlanningPipeline.ValidateCollisionCheckerAvailability(request.Options, scene, checker);
        if (checkerAvailabilityFail is not null) return checkerAvailabilityFail;
        var checkerFail = PlanningPipeline.ValidateMobileBaseChecker(space, checker);
        if (checkerFail is not null) return checkerFail;

        var startVal = request.Start.Validate(robot.Preset.JointLimits);
        var goalVal = request.Goal.Validate(robot.Preset.JointLimits);
        if (!startVal.IsValid) return PlanningResult.Failed(startVal.Errors.Select(e => $"Start: {e}"));
        if (!goalVal.IsValid) return PlanningResult.Failed(goalVal.Errors.Select(e => $"Goal: {e}"));

        var constraintFail = PlanningPipeline.TryBuildConstraintContext(request, serialChain, out var constraints);
        if (constraintFail is not null) return constraintFail;

        if (!ManagedRrtConnect.SegmentValid(space.Start, space.Goal, scene, checker, constraints, space, options.StepRadians))
            return PlanningResult.Failed(new[] { "CHOMP-lite direct seed violates collision or path constraints." });

        var smoothed = ChompLiteSmoother.SmoothInternal(
            new[] { space.Start, space.Goal },
            space.Limits,
            scene,
            checker,
            constraints,
            space,
            options);
        var full = smoothed.Select(q => new JointState(space.ToFull(q).Positions.ToArray())).ToList();
        if (space.HasMobility)
            return PlanningPipeline.BuildTrajectoryFromPlanSpace(
                robot, smoothed, space, request.Options, checker, usedNative: false, "CHOMP-lite");
        return PlanningPipeline.BuildTrajectory(robot, full, request.Options, checker, usedNative: false, "CHOMP-lite");
    }
}
