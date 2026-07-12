using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.Native;

namespace Motus.OMPL.NET;

internal static class PlanningPipeline
{
    internal readonly record struct PlanSpace(
        JointState Seed,
        double[] Start,
        double[] Goal,
        IReadOnlyList<JointLimit> Limits,
        Func<double[], JointState> ToFull)
    {
        public int Dims => Limits.Count;
    }

    internal static ICollisionChecker? ResolveChecker(PlanningRequest request, ICollisionChecker? defaultChecker) =>
        request.Options.CollisionChecker
        ?? (request.Options.AttachedBodies is { Count: > 0 }
            ? CollisionCheckerFactory.Create(request.Robot, null, request.Options.AttachedBodies)
            : defaultChecker);

    internal static PlanSpace BuildPlanSpace(PlanningRequest request)
    {
        var limits = request.Robot.Preset.JointLimits;
        var map = request.Options.GroupMap;
        if (map is null)
        {
            return new PlanSpace(
                request.Start,
                request.Start.Positions.ToArray(),
                request.Goal.Positions.ToArray(),
                limits,
                q => new JointState(q));
        }

        var groupLimits = map.GroupToFull.Select(i => limits[i]).ToList();
        return new PlanSpace(
            request.Start,
            map.ExtractGroupPositions(request.Start),
            map.ExtractGroupPositions(request.Goal),
            groupLimits,
            q => map.EmbedGroupState(request.Start, q));
    }

    internal static PlanningResult BuildTrajectory(
        RobotModel robot,
        IReadOnlyList<JointState> waypoints,
        PlanningOptions opts,
        ICollisionChecker? checker,
        bool usedNative,
        string plannerLabel)
    {
        var warnings = new List<string> { $"{plannerLabel}: joint-space sampling path." };
        warnings.Add(MotusCapabilities.Describe());
        if (!usedNative)
            warnings.Add(NativeOmpl.StatusMessage);
        if (checker is null)
            warnings.Add("Planner ran without collision checker (no kinematics chain).");

        if (waypoints.Count < 2)
            return PlanningResult.Failed(new[] { "Sampling path has insufficient waypoints." });

        if (checker is not null && PlanningCollision.SceneHasObstacles(opts.CollisionScene))
            return PlanningResult.Succeeded(new Trajectory(robot, BuildWaypointTrajectory(waypoints, opts)), warnings);

        var segmentOpts = opts;
        var planner = new JointLinearPlanner();
        var points = new List<TrajectoryPoint> { new(0, waypoints[0]) };
        var t = 0.0;
        for (var i = 1; i < waypoints.Count; i++)
        {
            var seg = planner.Plan(new PlanningRequest(robot, waypoints[i - 1], waypoints[i], segmentOpts));
            if (!seg.Success) return PlanningResult.Failed(seg.Errors);
            var segPts = seg.Trajectory!.Points;
            for (var j = 1; j < segPts.Count; j++)
            {
                t += segPts[j].TimeSeconds - segPts[j - 1].TimeSeconds;
                points.Add(new TrajectoryPoint(t, segPts[j].JointState));
            }
        }

        return PlanningResult.Succeeded(new Trajectory(robot, points), warnings);
    }

    private static List<TrajectoryPoint> BuildWaypointTrajectory(IReadOnlyList<JointState> waypoints, PlanningOptions opts)
    {
        var points = new List<TrajectoryPoint>(waypoints.Count);
        var t = 0.0;
        var maxVel = opts.MaxJointVelocityRadiansPerSecond;
        var minDt = opts.TimeStepSeconds;
        points.Add(new TrajectoryPoint(t, waypoints[0]));
        for (var i = 1; i < waypoints.Count; i++)
        {
            var maxDelta = 0.0;
            var prev = waypoints[i - 1].Positions;
            var cur = waypoints[i].Positions;
            for (var j = 0; j < cur.Length; j++)
                maxDelta = Math.Max(maxDelta, Math.Abs(cur[j] - prev[j]));
            t += Math.Max(minDt, maxDelta / maxVel);
            points.Add(new TrajectoryPoint(t, waypoints[i]));
        }
        return points;
    }
}
