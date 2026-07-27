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
        Func<double[], JointState> ToFull,
        Func<double[], BaseFrame?> ToBaseFrame,
        bool HasMobility)
    {
        public int Dims => Limits.Count;
    }

    internal readonly record struct ConstraintContext(
        bool Enabled,
        IFkSolver? Fk,
        BaseFrame BaseFrame,
        ToolFrame ToolFrame,
        IConstraintChecker? PathConstraints,
        IConstraintChecker? ConstraintChecker);

    internal static ICollisionChecker? ResolveChecker(PlanningRequest request, ICollisionChecker? defaultChecker) =>
        request.Options.CollisionChecker
        ?? (request.Options.AttachedBodies is { Count: > 0 }
            ? CollisionCheckerFactory.Create(request.Robot, null, request.Options.AttachedBodies)
            : defaultChecker);

    internal static PlanningResult? TryBuildPlanSpace(PlanningRequest request, out PlanSpace space)
    {
        space = default;
        if (request.Options.Mobility is not null &&
            request.Options.Mobility is not MobilityModel.HolonomicSE2)
        {
            return InvalidOptions(
                $"Unsupported Mobility model '{request.Options.Mobility.GetType().Name}'. Managed planners support HolonomicSE2 only.");
        }

        var jointSpace = BuildJointPlanSpace(request);
        if (request.Options.Mobility is not MobilityModel.HolonomicSE2 goalBase)
        {
            space = jointSpace;
            return null;
        }

        var bounds = request.Options.MobilityBounds ?? MobilityBounds.Default;
        MobilityModel.HolonomicSE2 startBase;
        try
        {
            startBase = ToHolonomicSE2(request.Robot.Preset.BaseFrame.Frame);
        }
        catch (InvalidOperationException ex)
        {
            return InvalidOptions(ex.Message);
        }
        if (Math.Abs(goalBase.Z - startBase.Z) > 1e-9)
            return InvalidOptions("HolonomicSE2 sampling plans x/y/yaw only; goal Z must equal the robot start base Z.");
        if (bounds.Validate(startBase, "Start") is { } startErr)
            return InvalidOptions(startErr);
        if (bounds.Validate(goalBase, "Goal") is { } goalErr)
            return InvalidOptions(goalErr);

        var jointDims = jointSpace.Start.Length;
        var start = new double[jointDims + 3];
        var goal = new double[jointDims + 3];
        Array.Copy(jointSpace.Start, start, jointDims);
        Array.Copy(jointSpace.Goal, goal, jointDims);
        start[jointDims + 0] = startBase.X;
        start[jointDims + 1] = startBase.Y;
        start[jointDims + 2] = startBase.YawRadians;
        goal[jointDims + 0] = goalBase.X;
        goal[jointDims + 1] = goalBase.Y;
        goal[jointDims + 2] = goalBase.YawRadians;

        var limits = jointSpace.Limits.Concat(bounds.ToJointLimits()).ToList();
        space = new PlanSpace(
            request.Start,
            start,
            goal,
            limits,
            q =>
            {
                var joints = new double[jointDims];
                Array.Copy(q, joints, jointDims);
                return jointSpace.ToFull(joints);
            },
            q => new BaseFrame(new MobilityModel.HolonomicSE2(
                q[jointDims + 0],
                q[jointDims + 1],
                NormalizeYaw(q[jointDims + 2]),
                startBase.Z).BaseFrame),
            HasMobility: true);
        return null;
    }

    internal static PlanSpace BuildPlanSpace(PlanningRequest request)
    {
        var fail = TryBuildPlanSpace(request, out var space);
        if (fail is not null)
            throw new InvalidOperationException(string.Join("; ", fail.Errors));
        return space;
    }

    private static PlanSpace BuildJointPlanSpace(PlanningRequest request)
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
                q => new JointState(q),
                _ => null,
                HasMobility: false);
        }

        var groupLimits = map.GroupToFull.Select(i => limits[i]).ToList();
        return new PlanSpace(
            request.Start,
            map.ExtractGroupPositions(request.Start),
            map.ExtractGroupPositions(request.Goal),
            groupLimits,
            q => map.EmbedGroupState(request.Start, q),
            _ => null,
            HasMobility: false);
    }

    private static MobilityModel.HolonomicSE2 ToHolonomicSE2(Frame frame)
    {
        var q = Transforms.NormalizeQuat(frame.Qw, frame.Qx, frame.Qy, frame.Qz);
        if (Math.Abs(q.x) > 1e-9 || Math.Abs(q.y) > 1e-9)
            throw new InvalidOperationException("HolonomicSE2 start base must be yaw-only (no roll/pitch).");
        return new MobilityModel.HolonomicSE2(frame.X, frame.Y, NormalizeYaw(2.0 * Math.Atan2(q.z, q.w)), frame.Z);
    }

    private static double NormalizeYaw(double yaw)
    {
        while (yaw > Math.PI) yaw -= 2.0 * Math.PI;
        while (yaw < -Math.PI) yaw += 2.0 * Math.PI;
        return yaw;
    }

    internal static PlanningResult? TryBuildConstraintContext(
        PlanningRequest request,
        SerialJointChain? serialChain,
        out ConstraintContext context)
    {
        context = default;
        if (request.Options.PathConstraints is null && request.Options.ConstraintChecker is null)
            return null;

        try
        {
            var fk = KinematicsResolver.CreateFkSolver(request.Robot.Preset, serialChain);
            context = new ConstraintContext(
                true,
                fk,
                request.Robot.Preset.BaseFrame,
                request.Robot.Preset.ToolFrame,
                request.Options.PathConstraints,
                request.Options.ConstraintChecker);
            return null;
        }
        catch (Exception ex) when (ex is NotSupportedException or InvalidOperationException)
        {
            return PlanningResult.Failed(new[]
            {
                new PlanningMessage(
                    PlanningMessageCodes.ConstraintViolation,
                    $"ConstraintViolation: planner cannot evaluate TCP constraints for '{request.Robot.Preset.ModelName}': {ex.Message}",
                    PlanningMessageSeverity.Error)
            });
        }
    }

    internal static bool TryValidateConstraints(ConstraintContext context, JointState state, out string reason) =>
        TryValidateConstraints(context, state, null, out reason);

    internal static bool TryValidateConstraints(
        ConstraintContext context,
        JointState state,
        BaseFrame? baseFrameOverride,
        out string reason)
    {
        if (!context.Enabled)
        {
            reason = string.Empty;
            return true;
        }

        var tcp = Transforms.ToFrame(context.Fk!.ComputeTcpTransform(
            state.Positions,
            (baseFrameOverride ?? context.BaseFrame).Frame,
            context.ToolFrame.Frame));

        if (context.PathConstraints is not null && !context.PathConstraints.TryValidate(tcp, out reason))
            return false;
        if (context.ConstraintChecker is not null && !context.ConstraintChecker.TryValidate(tcp, out reason))
            return false;

        reason = string.Empty;
        return true;
    }

    internal static PlanningResult ConstraintFailure(string label, string reason) =>
        PlanningResult.Failed(new[]
        {
            new PlanningMessage(
                PlanningMessageCodes.ConstraintViolation,
                $"{label}: {reason}",
                PlanningMessageSeverity.Error)
        });

    internal static PlanningResult InvalidOptions(string reason) =>
        PlanningResult.Failed(new[]
        {
            new PlanningMessage(
                PlanningMessageCodes.InvalidOptions,
                reason,
                PlanningMessageSeverity.Error)
        });

    internal static PlanningResult? ValidateMobileBaseChecker(PlanSpace space, ICollisionChecker? checker)
    {
        if (!space.HasMobility || checker is null || checker is IBaseFrameCollisionChecker)
            return null;
        return InvalidOptions(
            $"HolonomicSE2 planning requires an {nameof(IBaseFrameCollisionChecker)} when collision checking is enabled; " +
            $"got {checker.GetType().Name}.");
    }

    internal static PlanningResult? ValidateCollisionCheckerAvailability(
        PlanningOptions options,
        CollisionScene scene,
        ICollisionChecker? checker)
    {
        if (checker is not null ||
            (!PlanningCollision.SceneHasObstacles(scene) && options.AttachedBodies is not { Count: > 0 }))
            return null;
        return InvalidOptions(
            "Collision scene or attached bodies provided but no collision checker is available. " +
            "Supply PlanningOptions.CollisionChecker for this RobotPreset.Family.");
    }

    internal static bool StateCollisionFree(
        ICollisionChecker? checker,
        JointState state,
        CollisionScene scene,
        BaseFrame? baseFrameOverride)
    {
        if (checker is null) return true;
        if (baseFrameOverride is null) return checker.IsCollisionFree(state, scene);
        return checker is IBaseFrameCollisionChecker mobile &&
               mobile.IsCollisionFree(state, scene, baseFrameOverride);
    }

    internal static PlanningResult? ValidateEndpoints(
        PlanSpace space,
        CollisionScene? scene,
        ICollisionChecker? checker,
        bool includeAttachedBodies = false)
    {
        if (checker is null || (!PlanningCollision.SceneHasObstacles(scene) && !includeAttachedBodies))
            return null;
        scene ??= new CollisionScene();

        var start = space.ToFull(space.Start);
        var goal = space.ToFull(space.Goal);
        var startBase = space.ToBaseFrame(space.Start);
        var goalBase = space.ToBaseFrame(space.Goal);

        if (!StateCollisionFree(checker, start, scene, startBase))
        {
            return PlanningResult.Failed(new[]
            {
                new PlanningMessage(
                    PlanningMessageCodes.EndpointCollision,
                    StateCollisionFree(checker, start, new CollisionScene(), startBase)
                        ? "Start configuration is in collision with an obstacle."
                        : "Start configuration has self-collision.",
                    PlanningMessageSeverity.Error)
            });
        }
        if (!StateCollisionFree(checker, goal, scene, goalBase))
        {
            return PlanningResult.Failed(new[]
            {
                new PlanningMessage(
                    PlanningMessageCodes.EndpointCollision,
                    StateCollisionFree(checker, goal, new CollisionScene(), goalBase)
                        ? "Goal configuration is in collision with an obstacle."
                        : "Goal configuration has self-collision.",
                    PlanningMessageSeverity.Error)
            });
        }
        return null;
    }

    internal static PlanningResult BuildTrajectoryFromPlanSpace(
        RobotModel robot,
        IReadOnlyList<double[]> waypoints,
        PlanSpace space,
        PlanningOptions opts,
        ICollisionChecker? checker,
        bool usedNative,
        string plannerLabel)
    {
        var fullWaypoints = waypoints
            .Select(q => new JointState(space.ToFull(q).Positions.ToArray()))
            .ToList();
        if (!space.HasMobility)
            return BuildTrajectory(robot, fullWaypoints, opts, checker, usedNative, plannerLabel);

        var warnings = BuildSamplingWarnings(checker, usedNative, plannerLabel);
        warnings.Add(MobilityMethodRefs.DescribeHolonomicSe2());
        if (waypoints.Count < 2)
            return PlanningResult.Failed(new[] { "Sampling path has insufficient waypoints." });

        var points = BuildWaypointTrajectory(waypoints, space, opts);
        return PlanningResult.Succeeded(new Trajectory(robot, points), warnings);
    }

    internal static PlanningResult BuildTrajectory(
        RobotModel robot,
        IReadOnlyList<JointState> waypoints,
        PlanningOptions opts,
        ICollisionChecker? checker,
        bool usedNative,
        string plannerLabel)
    {
        var warnings = BuildSamplingWarnings(checker, usedNative, plannerLabel);

        if (waypoints.Count < 2)
            return PlanningResult.Failed(new[] { "Sampling path has insufficient waypoints." });

        // RRT already validated segments — densify without re-running JointLinearPlanner collision checks.
        if (checker is not null && PlanningCollision.SceneHasObstacles(opts.CollisionScene))
            return PlanningResult.Succeeded(new Trajectory(robot, BuildWaypointTrajectory(waypoints, opts)), warnings);

        return PlanningResult.Succeeded(new Trajectory(robot, DensifyWaypoints(waypoints, opts)), warnings);
    }

    private static List<string> BuildSamplingWarnings(
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
        return warnings;
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

    private static List<TrajectoryPoint> BuildWaypointTrajectory(
        IReadOnlyList<double[]> waypoints,
        PlanSpace space,
        PlanningOptions opts)
    {
        var points = new List<TrajectoryPoint>(waypoints.Count);
        var t = 0.0;
        var maxVel = opts.MaxJointVelocityRadiansPerSecond > 0 ? opts.MaxJointVelocityRadiansPerSecond : 1.0;
        var minDt = opts.TimeStepSeconds > 0 ? opts.TimeStepSeconds : 0.01;
        points.Add(new TrajectoryPoint(
            t,
            new JointState(space.ToFull(waypoints[0]).Positions.ToArray()),
            baseFrameOverride: space.ToBaseFrame(waypoints[0])));
        for (var i = 1; i < waypoints.Count; i++)
        {
            var maxDelta = 0.0;
            var prev = waypoints[i - 1];
            var cur = waypoints[i];
            for (var j = 0; j < cur.Length; j++)
                maxDelta = Math.Max(maxDelta, Math.Abs(cur[j] - prev[j]));
            t += Math.Max(minDt, maxDelta / maxVel);
            points.Add(new TrajectoryPoint(
                t,
                new JointState(space.ToFull(cur).Positions.ToArray()),
                baseFrameOverride: space.ToBaseFrame(cur)));
        }
        return points;
    }
}
