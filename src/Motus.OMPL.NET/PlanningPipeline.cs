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

        // RRT already validated segments — densify without re-running JointLinearPlanner collision checks.
        if (checker is not null && PlanningCollision.SceneHasObstacles(opts.CollisionScene))
            return PlanningResult.Succeeded(new Trajectory(robot, BuildWaypointTrajectory(waypoints, opts)), warnings);

        return PlanningResult.Succeeded(new Trajectory(robot, DensifyWaypoints(waypoints, opts)), warnings);
    }

    /// <summary>Interpolate joint-space path without limit/collision re-validation (post-RRT).</summary>
    private static List<TrajectoryPoint> DensifyWaypoints(IReadOnlyList<JointState> waypoints, PlanningOptions opts)
    {
        var step = opts.MaxJointStepRadians > 0 ? opts.MaxJointStepRadians : 0.12;
        var maxVel = opts.MaxJointVelocityRadiansPerSecond > 0 ? opts.MaxJointVelocityRadiansPerSecond : 1.0;
        var minDt = opts.TimeStepSeconds > 0 ? opts.TimeStepSeconds : 0.01;
        var points = new List<TrajectoryPoint> { new(0, waypoints[0]) };
        var t = 0.0;
        for (var i = 1; i < waypoints.Count; i++)
        {
            var from = waypoints[i - 1].Positions;
            var to = waypoints[i].Positions;
            var n = from.Length;
            var maxDelta = 0.0;
            for (var j = 0; j < n; j++)
                maxDelta = Math.Max(maxDelta, Math.Abs(to[j] - from[j]));
            var steps = Math.Max(1, (int)Math.Ceiling(maxDelta / step));
            for (var s = 1; s <= steps; s++)
            {
                var alpha = (double)s / steps;
                var pos = new double[n];
                for (var j = 0; j < n; j++)
                    pos[j] = from[j] + alpha * (to[j] - from[j]);
                var stepDelta = maxDelta / steps;
                t += Math.Max(minDt, stepDelta / maxVel);
                points.Add(new TrajectoryPoint(t, new JointState(pos)));
            }
        }
        return points;
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
