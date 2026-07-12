using System.Runtime.InteropServices;
using Motus.Core;
using Motus.Geometry;
using Motus.Native;
using Motus.OMPL.Native;

namespace Motus.OMPL.NET;

internal static class NativeOmplPlanner
{

    internal static PlanningResult? TryPlan(
        PlanningRequest request,
        SamplingPlannerOptions options,
        ICollisionChecker? defaultChecker,
        int nativePlannerId,
        string plannerLabel)
    {
        if (!NativeOmpl.IsAvailable || options.PreferManaged)
            return null;
        if (nativePlannerId >= 0 && NativeOmpl.motus_ompl_planner_available(nativePlannerId) == 0)
            return null;

        var checker = PlanningPipeline.ResolveChecker(request, defaultChecker);
        var robot = request.Robot;
        var scene = request.Options.CollisionScene ?? new CollisionScene();
        var space = PlanningPipeline.BuildPlanSpace(request);
        var endpointFail = PlanningCollision.ValidateEndpoints(
            space.ToFull(space.Start), space.ToFull(space.Goal), scene, checker);
        if (endpointFail is not null)
            return endpointFail;

        var n = space.Dims;
        var low = space.Limits.Select(l => l.MinRadians).ToArray();
        var high = space.Limits.Select(l => l.MaxRadians).ToArray();
        var maxStates = Math.Max(16, options.MaxPathStates);
        var buffer = new double[n * maxStates];

        var ctx = new NativePlanContext(checker, scene, options.StepRadians, space.ToFull);
        var handle = GCHandle.Alloc(ctx);
        try
        {
            NativeBindings.ValidityCallback stateCb = (statePtr, dims, user) =>
            {
                var context = (NativePlanContext)GCHandle.FromIntPtr(user).Target!;
                return context.ValidityCallback(statePtr, dims);
            };
            NativeBindings.MotionValidityCallback motionCb = (fromPtr, toPtr, dims, user) =>
            {
                var context = (NativePlanContext)GCHandle.FromIntPtr(user).Target!;
                return context.MotionValidityCallback(fromPtr, toPtr, dims);
            };

            lock (NativeSync.Gate)
            {
                var rc = NativeOmpl.motus_ompl_plan(
                    n, low, high, space.Start, space.Goal,
                    options.MaxIterations, options.MaxPlanTimeSeconds, options.StepRadians, options.GoalBias,
                    nativePlannerId,
                    stateCb, motionCb, GCHandle.ToIntPtr(handle),
                    buffer, maxStates, out var count);

                if (rc != NativeOmpl.Ok || count < 2) return null;

                var waypoints = new List<JointState>(count);
                for (var i = 0; i < count; i++)
                {
                    var q = new double[n];
                    Array.Copy(buffer, i * n, q, 0, n);
                    waypoints.Add(space.ToFull(q));
                }

                var simplified = PathSimplifier.Simplify(
                    waypoints, robot, checker, scene, options.StepRadians * 0.5);
                return PlanningPipeline.BuildTrajectory(robot, simplified, request.Options, checker, usedNative: true, plannerLabel);
            }
        }
        finally
        {
            handle.Free();
        }
    }
}
