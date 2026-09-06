using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Motion program planner for mixed PTP/LIN/CIRC/SET/WAIT/Attach/Detach segment lists.
/// Blend radii truncate TCP paths at segment corners when feasible; otherwise exact-stop fallback.
/// Attach/Detach mutate a <see cref="PlanningContext"/> mid-program and emit <see cref="AttachTimeSpan"/> windows.
/// </summary>
public sealed class IndustrialMotionPlanner
{
    private const double BlendPathEpsilon = 1e-9;

    private readonly JointLinearPlanner _joint = new();
    private readonly CartesianLinearPathPlanner _linPlanner;
    private readonly IFkSolver _fk;
    private readonly BaseFrame _base;
    private readonly ToolFrame _tool;

    public IndustrialMotionPlanner(RobotPreset preset, SerialJointChain? chain = null)
    {
        _linPlanner = new CartesianLinearPathPlanner(preset, chain);
        _fk = KinematicsResolver.CreateFkSolver(preset, chain);
        _base = preset.BaseFrame;
        _tool = preset.ToolFrame;
    }

    public PlanningResult Plan(MotionProgramRequest request)
    {
        if (request.Segments.Count == 0)
            return PlanningResult.Failed(new[] { "Motion program requires at least one segment." });

        var currentState = request.Start;
        var currentPose = TcpPose(currentState);
        var points = new List<TrajectoryPoint> { new(0, currentState) };
        var spans = new List<ToolStateTimeline.SegmentSpan>();
        var warnings = new List<string>();
        var t = 0.0;
        double pendingEntryBlend = 0;
        int pendingBlendFromSegment = -1;

        var ctx = BuildContext(request);
        var robotOnly = CollisionCheckerFactory.Create(ctx.Robot, attached: null);
        var liveOptions = CloneOptions(request.Options, ctx, robotOnly);
        var startCollision = PlanningCollision.ValidateEndpoints(
            request.Start,
            request.Start,
            liveOptions.CollisionScene,
            liveOptions.CollisionChecker,
            includeAttachedBodies: liveOptions.AttachedBodies is { Count: > 0 });
        if (startCollision is not null)
            return startCollision;

        var openAttachStart = new Dictionary<string, (double Start, AttachedBody Body)>(StringComparer.OrdinalIgnoreCase);
        var attachSpans = new List<AttachTimeSpan>();

        for (var i = 0; i < request.Segments.Count; i++)
        {
            var segment = request.Segments[i];

            if (segment is AttachSegment attachSeg)
            {
                var apply = ApplyAttach(ctx, attachSeg, request.Robot, ref liveOptions, robotOnly, out var err);
                if (err is not null)
                    return PlanningResult.Failed(new[] { $"Segment {i + 1} Attach: {err}" });
                ctx = apply;
                var body = ctx.Attached.First(a => string.Equals(a.Name, attachSeg.Name, StringComparison.OrdinalIgnoreCase));
                openAttachStart[attachSeg.Name] = (t, body);
                spans.Add(new ToolStateTimeline.SegmentSpan(i, segment, points.Count - 1, points.Count - 1));
                continue;
            }

            if (segment is DetachSegment detachSeg)
            {
                if (openAttachStart.TryGetValue(detachSeg.Name, out var open))
                {
                    attachSpans.Add(new AttachTimeSpan(open.Start, t, new[] { open.Body }, detachSeg.WorldPose));
                    openAttachStart.Remove(detachSeg.Name);
                }

                ctx = ctx.Detach(detachSeg.Name, detachSeg.WorldPose);
                liveOptions = CloneOptions(liveOptions, ctx, robotOnly);
                spans.Add(new ToolStateTimeline.SegmentSpan(i, segment, points.Count - 1, points.Count - 1));
                continue;
            }

            if (segment is SetToolStateSegment setSeg)
            {
                var spanStart = points.Count - 1;
                if (setSeg.DurationSeconds > 0)
                {
                    t += setSeg.DurationSeconds;
                    points.Add(new TrajectoryPoint(t, currentState, MotionPrimitiveType.Set, i));
                }
                spans.Add(new ToolStateTimeline.SegmentSpan(i, segment, spanStart, points.Count - 1));
                continue;
            }

            if (segment is WaitSegment waitSeg)
            {
                var spanStart = points.Count - 1;
                if (waitSeg.DurationSeconds > 0)
                {
                    t += waitSeg.DurationSeconds;
                    points.Add(new TrajectoryPoint(t, currentState, MotionPrimitiveType.Wait, i));
                }
                spans.Add(new ToolStateTimeline.SegmentSpan(i, segment, spanStart, points.Count - 1));
                continue;
            }

            var pointStartBefore = points.Count;
            var result = PlanSegment(request.Robot, liveOptions, currentState, currentPose, segment, warnings);
            if (!result.Success || result.Trajectory is null)
                return result;

            var rawPoints = result.Trajectory.Points;
            var startIdx = 0;
            var endIdx = rawPoints.Count - 1;
            var junction = SegmentGoalPose(segment);

            if (pendingEntryBlend > 0)
            {
                if (!TryTruncateStart(rawPoints, startIdx, endIdx, pendingEntryBlend, junction.Tcp, out startIdx))
                {
                    warnings.Add(
                        $"Blend radius {pendingEntryBlend:F3}m requested at segment {pendingBlendFromSegment}; fallback to exact-stop transition.");
                }
                pendingEntryBlend = 0;
                pendingBlendFromSegment = -1;
            }

            if (segment.BlendRadiusMeters > 0 && i < request.Segments.Count - 1)
            {
                var pathLen = TcpPathLength(rawPoints, startIdx, endIdx);
                if (pathLen < BlendPathEpsilon)
                {
                    // zero-length segment — no truncation needed
                }
                else if (TryTruncateEnd(rawPoints, startIdx, endIdx, segment.BlendRadiusMeters, junction.Tcp, out endIdx))
                {
                    pendingEntryBlend = segment.BlendRadiusMeters;
                    pendingBlendFromSegment = i;
                }
                else
                {
                    warnings.Add(
                        $"Blend radius {segment.BlendRadiusMeters:F3}m requested at segment {i}; fallback to exact-stop transition.");
                }
            }

            for (var p = Math.Max(startIdx, 1); p <= endIdx; p++)
            {
                t += rawPoints[p].TimeSeconds - rawPoints[p - 1].TimeSeconds;
                points.Add(new TrajectoryPoint(
                    t,
                    rawPoints[p].JointState,
                    segment.Type,
                    i,
                    segment.BlendRadiusMeters));
            }

            var spanFirst = Math.Max(pointStartBefore - 1, 0);
            var spanLast = points.Count - 1;
            if (spanLast >= spanFirst)
                spans.Add(new ToolStateTimeline.SegmentSpan(i, segment, spanFirst, spanLast));

            currentState = rawPoints[endIdx].JointState;
            currentPose = TcpPose(currentState);
        }

        foreach (var leftover in openAttachStart.Values)
            attachSpans.Add(new AttachTimeSpan(leftover.Start, t, new[] { leftover.Body }));

        var initialToolState = ResolveInitialToolState(request);
        var annotated = ToolStateTimeline.Apply(points, request.Segments, spans, initialToolState);
        var trajectory = new Trajectory(request.Robot, annotated);

        if (request.SessionTool is { Capabilities: not null } sessionTool)
        {
            warnings.AddRange(ToolStateCollision.ValidateTrajectory(
                trajectory,
                sessionTool,
                liveOptions.CollisionScene,
                liveOptions.CollisionChecker));
        }

        return PlanningResult.Succeeded(trajectory, warnings, attachSpans);
    }

    private static PlanningContext BuildContext(MotionProgramRequest request)
    {
        var scene = request.Options.CollisionScene ?? new CollisionScene();
        var attached = request.Options.AttachedBodies ?? Array.Empty<AttachedBody>();
        var ctx = PlanningContext.Create(request.Robot, scene);
        foreach (var body in attached)
            ctx = ctx.Attach(body);
        return ctx;
    }

    private static PlanningContext ApplyAttach(
        PlanningContext ctx,
        AttachSegment attach,
        RobotModel robot,
        ref PlanningOptions liveOptions,
        ICollisionChecker robotOnly,
        out string? error)
    {
        error = null;
        _ = robot;
        var next = ctx.Attach(attach.Name, WithIdentityPose(attach.Geometry), attach.TcpLocal);
        liveOptions = CloneOptions(liveOptions, next, robotOnly);
        return next;
    }

    private static CollisionObject WithIdentityPose(CollisionObject source) =>
        source.Shape switch
        {
            CollisionShape.Box => CollisionObject.Box(source.Name, Frame.Identity, source.ExtentX, source.ExtentY, source.ExtentZ),
            CollisionShape.Sphere => CollisionObject.Sphere(source.Name, Frame.Identity, source.ExtentX),
            CollisionShape.Capsule => CollisionObject.Capsule(source.Name, Frame.Identity, source.ExtentX, source.ExtentY),
            CollisionShape.Mesh when source.MeshVertices is not null && source.MeshIndices is not null =>
                CollisionObject.Mesh(source.Name, Frame.Identity, source.MeshVertices, source.MeshIndices),
            _ => source
        };

    private static PlanningOptions CloneOptions(
        PlanningOptions baseOpts, PlanningContext ctx, ICollisionChecker robotOnly)
    {
        // Robot-vs-scene without attached; attached volumes only vs scene (grasp occupies the part).
        ICollisionChecker checker = robotOnly;
        if (ctx.Attached is { Count: > 0 })
        {
            var fk = KinematicsResolver.CreateFkSolver(ctx.Robot.Preset);
            checker = new AttachAwareCollisionChecker(
                robotOnly, fk, ctx.Robot.Preset.BaseFrame, ctx.Robot.Preset.ToolFrame, ctx.Attached);
        }

        return new PlanningOptions
        {
            MaxJointStepRadians = baseOpts.MaxJointStepRadians,
            TimeStepSeconds = baseOpts.TimeStepSeconds,
            MaxJointVelocityRadiansPerSecond = baseOpts.MaxJointVelocityRadiansPerSecond,
            CollisionScene = ctx.Scene,
            CollisionChecker = checker,
            RetimeTrajectory = baseOpts.RetimeTrajectory,
            AttachedBodies = ctx.Attached,
            PathConstraints = baseOpts.PathConstraints,
            ConstraintChecker = baseOpts.ConstraintChecker,
            GroupMap = baseOpts.GroupMap,
            Mobility = baseOpts.Mobility,
            MobilityBounds = baseOpts.MobilityBounds
        };
    }

    private static EndEffectorState? ResolveInitialToolState(MotionProgramRequest request) =>
        request.InitialToolState
        ?? request.ToolCapabilities?.DefaultState()
        ?? ToolDefinition.FromPreset(request.Robot)?.Capabilities?.DefaultState();

    private CartesianPose SegmentGoalPose(MotionSegment segment) =>
        segment switch
        {
            PtpSegment ptp => TcpPose(ptp.Goal),
            LinSegment lin => lin.Goal,
            CircSegment circ => circ.Goal,
            SetToolStateSegment => throw new InvalidOperationException("SET segment has no Cartesian goal."),
            WaitSegment => throw new InvalidOperationException("WAIT segment has no Cartesian goal."),
            AttachSegment => throw new InvalidOperationException("Attach segment has no Cartesian goal."),
            DetachSegment => throw new InvalidOperationException("Detach segment has no Cartesian goal."),
            _ => throw new InvalidOperationException("Unsupported motion segment type.")
        };

    private CartesianPose TcpPose(JointState joints)
    {
        var frame = Transforms.ToFrame(_fk.ComputeTcpTransform(joints.Positions, _base.Frame, _tool.Frame));
        return new CartesianPose(frame);
    }

    private double TcpPathLength(IReadOnlyList<TrajectoryPoint> segPoints, int startIdx, int endIdx)
    {
        if (endIdx <= startIdx) return 0;
        var total = 0.0;
        for (var i = startIdx + 1; i <= endIdx; i++)
        {
            var a = TcpPose(segPoints[i - 1].JointState).Tcp;
            var b = TcpPose(segPoints[i].JointState).Tcp;
            total += TcpDist(a, b);
        }
        return total;
    }

    private bool TryTruncateEnd(
        IReadOnlyList<TrajectoryPoint> segPoints,
        int startIdx,
        int endIdx,
        double blendRadius,
        Frame cornerTcp,
        out int newEndIdx)
    {
        newEndIdx = endIdx;
        if (endIdx <= startIdx || blendRadius <= 0) return false;

        for (var i = endIdx; i >= startIdx; i--)
        {
            var tcp = TcpPose(segPoints[i].JointState).Tcp;
            if (TcpDist(tcp, cornerTcp) >= blendRadius - 1e-6)
            {
                newEndIdx = i;
                return i > startIdx;
            }
        }
        return false;
    }

    private bool TryTruncateStart(
        IReadOnlyList<TrajectoryPoint> segPoints,
        int startIdx,
        int endIdx,
        double blendRadius,
        Frame cornerTcp,
        out int newStartIdx)
    {
        newStartIdx = startIdx;
        if (endIdx <= startIdx || blendRadius <= 0) return false;

        for (var i = startIdx; i <= endIdx; i++)
        {
            var tcp = TcpPose(segPoints[i].JointState).Tcp;
            if (TcpDist(tcp, cornerTcp) >= blendRadius - 1e-6)
            {
                newStartIdx = i;
                return i < endIdx;
            }
        }
        return false;
    }

    private static double TcpDist(Frame a, Frame b)
    {
        var dx = a.X - b.X;
        var dy = a.Y - b.Y;
        var dz = a.Z - b.Z;
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    private PlanningResult PlanSegment(
        RobotModel robot,
        PlanningOptions options,
        JointState currentState,
        CartesianPose currentPose,
        MotionSegment segment,
        List<string> warnings)
    {
        return segment switch
        {
            PtpSegment ptp => _joint.Plan(new PlanningRequest(robot, currentState, ptp.Goal, options)),
            LinSegment lin => PlanLinearSegment(robot, options, currentState, currentPose, lin),
            CircSegment circ => PlanCircularSegment(robot, options, currentState, currentPose, circ, warnings),
            SetToolStateSegment => PlanningResult.Succeeded(new Trajectory(robot, new[] { new TrajectoryPoint(0, currentState) })),
            WaitSegment => PlanningResult.Succeeded(new Trajectory(robot, new[] { new TrajectoryPoint(0, currentState) })),
            AttachSegment => PlanningResult.Succeeded(new Trajectory(robot, new[] { new TrajectoryPoint(0, currentState) })),
            DetachSegment => PlanningResult.Succeeded(new Trajectory(robot, new[] { new TrajectoryPoint(0, currentState) })),
            _ => PlanningResult.Failed(new[] { "Unsupported motion segment type." })
        };
    }

    private PlanningResult PlanLinearSegment(
        RobotModel robot,
        PlanningOptions options,
        JointState currentState,
        CartesianPose currentPose,
        LinSegment segment)
    {
        if (segment.StepMeters <= 0)
            return PlanningResult.Failed(new[] { "LIN stepMeters must be positive." });

        var traj = _linPlanner.Plan(
            currentPose,
            segment.Goal,
            currentState,
            new CartesianLinOptions(StepMeters: segment.StepMeters, ContinueOnIkFailure: false));
        if (traj is null)
            return PlanningResult.Failed(new[] { "LIN planning failed (IK at intermediate poses)." });

        var fail = ValidateSegmentCollision(robot, options, traj);
        if (fail is not null) return fail;

        return PlanningResult.Succeeded(traj);
    }

    private PlanningResult PlanCircularSegment(
        RobotModel robot,
        PlanningOptions options,
        JointState currentState,
        CartesianPose currentPose,
        CircSegment segment,
        List<string> warnings)
    {
        if (segment.ArcSamples < 4)
            return PlanningResult.Failed(new[] { "CIRC arcSamples must be >= 4." });

        var arcWaypoints = BuildCircularWaypoints(currentPose, segment.Via, segment.Goal, segment.ArcSamples);
        if (arcWaypoints is null)
            return PlanningResult.Failed(new[] { "CIRC geometry invalid (collinear or degenerate start/via/goal)." });

        var traj = _linPlanner.PlanToolpath(arcWaypoints, currentState);
        if (traj is null)
            return PlanningResult.Failed(new[] { "CIRC planning failed (IK on circular waypoints)." });

        var fail = ValidateSegmentCollision(robot, options, traj);
        if (fail is not null) return fail;

        warnings.Add("CIRC orientation policy: SLERP from segment start orientation to final orientation.");
        return PlanningResult.Succeeded(traj);
    }

    private static PlanningResult? ValidateSegmentCollision(RobotModel robot, PlanningOptions options, Trajectory traj)
    {
        var checker = options.CollisionChecker;
        var scene = options.CollisionScene ?? new CollisionScene();
        if (!PlanningCollision.SceneHasObstacles(scene) &&
            options.AttachedBodies is not { Count: > 0 })
            return null;

        checker ??= CollisionCheckerFactory.Create(robot, attached: options.AttachedBodies);
        if (checker is null)
            return PlanningResult.Failed(new[] { "Collision scene provided but no collision checker available." });

        return PlanningCollision.ValidateTrajectory(traj, scene!, checker, options.MaxJointStepRadians);
    }

    private static IReadOnlyList<CartesianPose>? BuildCircularWaypoints(
        CartesianPose start,
        CartesianPose via,
        CartesianPose goal,
        int samples)
    {
        var a = (start.Tcp.X, start.Tcp.Y, start.Tcp.Z);
        var b = (via.Tcp.X, via.Tcp.Y, via.Tcp.Z);
        var c = (goal.Tcp.X, goal.Tcp.Y, goal.Tcp.Z);

        var ab = Sub(b, a);
        var ac = Sub(c, a);
        var normal = Cross(ab, ac);
        var nNorm = Norm(normal);
        if (nNorm < 1e-9) return null;
        normal = Scale(normal, 1.0 / nNorm);

        var u = Scale(ab, 1.0 / Math.Max(Norm(ab), 1e-12));
        var v = Cross(normal, u);

        var bx = Dot(ab, u);
        var cx = Dot(ac, u);
        var cy = Dot(ac, v);
        if (Math.Abs(cy) < 1e-9) return null;

        var ux = bx * 0.5;
        var uy = (cx * cx + cy * cy - bx * cx) / (2 * cy);
        var r = Math.Sqrt(ux * ux + uy * uy);
        if (r < 1e-9) return null;

        var aAngle = Math.Atan2(-uy, -ux);
        var bAngle = Math.Atan2(-uy, bx - ux);
        var cAngle = Math.Atan2(cy - uy, cx - ux);
        var endAngle = SelectArcEndAngle(aAngle, bAngle, cAngle);

        var outWaypoints = new List<CartesianPose>(samples + 1);
        for (var i = 0; i <= samples; i++)
        {
            var t = (double)i / samples;
            var ang = aAngle + (endAngle - aAngle) * t;
            var px = ux + r * Math.Cos(ang);
            var py = uy + r * Math.Sin(ang);
            var pos = Add(a, Add(Scale(u, px), Scale(v, py)));

            var q = Slerp(
                start.Tcp.Qw, start.Tcp.Qx, start.Tcp.Qy, start.Tcp.Qz,
                goal.Tcp.Qw, goal.Tcp.Qx, goal.Tcp.Qy, goal.Tcp.Qz,
                t);
            outWaypoints.Add(new CartesianPose(new Frame(pos.x, pos.y, pos.z, q.w, q.x, q.y, q.z)));
        }

        return outWaypoints;
    }

    private static double SelectArcEndAngle(double start, double via, double end)
    {
        var ccwEnd = NormalizeRelative(end, start, positiveDirection: true);
        var ccwVia = NormalizeRelative(via, start, positiveDirection: true);
        if (ccwVia >= 0 && ccwVia <= ccwEnd) return start + ccwEnd;

        var cwEnd = NormalizeRelative(end, start, positiveDirection: false);
        return start + cwEnd;
    }

    private static double NormalizeRelative(double target, double origin, bool positiveDirection)
    {
        var d = target - origin;
        var twoPi = 2 * Math.PI;
        if (positiveDirection)
        {
            while (d < 0) d += twoPi;
            while (d >= twoPi) d -= twoPi;
            return d;
        }

        while (d > 0) d -= twoPi;
        while (d <= -twoPi) d += twoPi;
        return d;
    }

    private static (double w, double x, double y, double z) Slerp(
        double aw, double ax, double ay, double az,
        double bw, double bx, double by, double bz, double t)
    {
        var dot = aw * bw + ax * bx + ay * by + az * bz;
        if (dot < 0)
        {
            bw = -bw; bx = -bx; by = -by; bz = -bz;
            dot = -dot;
        }

        if (dot > 0.9995)
            return (aw + t * (bw - aw), ax + t * (bx - ax), ay + t * (by - ay), az + t * (bz - az));

        var theta0 = Math.Acos(Math.Clamp(dot, -1, 1));
        var theta = theta0 * t;
        var s0 = Math.Cos(theta) - dot * Math.Sin(theta) / Math.Sin(theta0);
        var s1 = Math.Sin(theta) / Math.Sin(theta0);
        return (s0 * aw + s1 * bw, s0 * ax + s1 * bx, s0 * ay + s1 * by, s0 * az + s1 * bz);
    }

    private static (double x, double y, double z) Sub((double x, double y, double z) a, (double x, double y, double z) b) =>
        (a.x - b.x, a.y - b.y, a.z - b.z);

    private static (double x, double y, double z) Add((double x, double y, double z) a, (double x, double y, double z) b) =>
        (a.x + b.x, a.y + b.y, a.z + b.z);

    private static (double x, double y, double z) Scale((double x, double y, double z) a, double s) =>
        (a.x * s, a.y * s, a.z * s);

    private static double Dot((double x, double y, double z) a, (double x, double y, double z) b) =>
        a.x * b.x + a.y * b.y + a.z * b.z;

    private static (double x, double y, double z) Cross((double x, double y, double z) a, (double x, double y, double z) b) =>
        (a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);

    private static double Norm((double x, double y, double z) a) => Math.Sqrt(Dot(a, a));
}
