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

            var simplified = SimplifyNativePath(
                waypoints, n, space, request, robot, checker, scene,
                stateCb, motionCb, handle, maxStates, options.StepRadians);
            return PlanningPipeline.BuildTrajectory(robot, simplified, request.Options, checker, usedNative: true, plannerLabel);
        }
        finally
        {
            handle.Free();
        }
    }

    private static IReadOnlyList<JointState> SimplifyNativePath(
        List<JointState> waypoints,
        int dims,
        PlanningPipeline.PlanSpace space,
        PlanningRequest request,
        RobotModel robot,
        ICollisionChecker? checker,
        CollisionScene scene,
        NativeBindings.ValidityCallback stateCb,
        NativeBindings.MotionValidityCallback motionCb,
        GCHandle handle,
        int maxStates,
        double stepRadians)
    {
        var pathCount = waypoints.Count;
        var flatPath = new double[pathCount * dims];
        for (var i = 0; i < pathCount; i++)
        {
            var groupQ = request.Options.GroupMap?.ExtractGroupPositions(waypoints[i])
                ?? waypoints[i].Positions.ToArray();
            Array.Copy(groupQ, 0, flatPath, i * dims, dims);
        }

        var simpBuf = new double[dims * maxStates];
        var simpRc = NativeOmpl.motus_ompl_simplify_path(
            dims, flatPath, pathCount, stepRadians * 0.5,
            stateCb, motionCb, GCHandle.ToIntPtr(handle),
            simpBuf, maxStates, out var simpCount);

        if (simpRc != NativeOmpl.Ok || simpCount < 2)
            return PathSimplifier.Simplify(waypoints, robot, checker, scene, stepRadians * 0.5);

        var simplified = new List<JointState>(simpCount);
        for (var i = 0; i < simpCount; i++)
        {
            var q = new double[dims];
            Array.Copy(simpBuf, i * dims, q, 0, dims);
            simplified.Add(space.ToFull(q));
        }
        return simplified;
    }
}
