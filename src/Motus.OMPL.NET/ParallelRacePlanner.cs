using Motus.Core;
using Motus.Geometry;
using Motus.Native;

namespace Motus.OMPL.NET;

internal static class ParallelRacePlanner
{
    internal static PlanningResult Plan(
        PlanningRequest request,
        SamplingPlannerOptions options,
        ICollisionChecker? defaultChecker)
    {
        if (options.ShouldCancel?.Invoke() == true)
            return PlanningResult.Failed(new[] { "Planning cancelled." });

        var halfTime = options.MaxPlanTimeSeconds > 0 ? options.MaxPlanTimeSeconds * 0.5 : 0.0;
        var halfIter = Math.Max(1, options.MaxIterations / 2);

        var connectOpts = CloneOptions(options, SamplingPlannerId.RrtConnect, halfIter, halfTime);
        var aorrtcOpts = CloneOptions(options, SamplingPlannerId.Aorrtc, halfIter, halfTime);

        PlanningResult? connect = null;
        PlanningResult? aorrtc = null;

        if (SamplingPlannerRegistry.IsNativeAvailable(SamplingPlannerId.RrtConnect))
            connect = NativeOmplPlanner.TryPlan(request, connectOpts, defaultChecker, NativeBindings.PlannerRrtConnect, "RRT-Connect");
        connect ??= ManagedRrtConnect.Plan(request, connectOpts, defaultChecker);

        if (SamplingPlannerRegistry.IsNativeAvailable(SamplingPlannerId.Aorrtc))
            aorrtc = NativeOmplPlanner.TryPlan(request, aorrtcOpts, defaultChecker, NativeBindings.PlannerAorrtc, "AORRTC");

        if (connect?.Success == true && aorrtc?.Success == true)
        {
            var connectLen = PathLength(connect.Trajectory!);
            var aorrtcLen = PathLength(aorrtc.Trajectory!);
            return connectLen <= aorrtcLen ? connect : aorrtc;
        }

        if (connect?.Success == true) return connect;
        if (aorrtc?.Success == true) return aorrtc;

        var errors = new List<string>();
        if (connect is not null) errors.AddRange(connect.Errors);
        if (aorrtc is not null) errors.AddRange(aorrtc.Errors);
        if (errors.Count == 0)
            errors.Add("Parallel race failed: neither RRT-Connect nor AORRTC found a path.");
        return PlanningResult.Failed(errors.Distinct());
    }

    private static double PathLength(Trajectory trajectory)
    {
        var sum = 0.0;
        for (var i = 1; i < trajectory.Points.Count; i++)
        {
            var a = trajectory.Points[i - 1].JointState.Positions;
            var b = trajectory.Points[i].JointState.Positions;
            for (var j = 0; j < a.Length; j++)
            {
                var d = a[j] - b[j];
                sum += d * d;
            }
        }
        return sum;
    }

    private static SamplingPlannerOptions CloneOptions(
        SamplingPlannerOptions options,
        SamplingPlannerId plannerId,
        int maxIterations,
        double maxPlanTimeSeconds) => new()
    {
        MaxIterations = maxIterations,
        MaxPlanTimeSeconds = maxPlanTimeSeconds,
        StepRadians = options.StepRadians,
        GoalBias = options.GoalBias,
        ConnectThresholdRadians = options.ConnectThresholdRadians,
        RandomSeed = options.RandomSeed,
        MaxPathStates = options.MaxPathStates,
        PlannerId = plannerId,
        PreferManaged = options.PreferManaged,
        ShouldCancel = options.ShouldCancel,
        ReportIteration = options.ReportIteration
    };
}
