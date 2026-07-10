using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Motion program planner for mixed PTP/LIN/CIRC segment lists.
/// Blend radii are accepted and exported as metadata; unsupported blends fall back to exact-stop transitions.
/// </summary>
public sealed class IndustrialMotionPlanner
{
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
        var currentPose = new CartesianPose(Transforms.ToFrame(
            _fk.ComputeTcpTransform(currentState.Positions, _base.Frame, _tool.Frame)));
        var points = new List<TrajectoryPoint>
        {
            new(0, currentState)
        };
        var warnings = new List<string>();
        var t = 0.0;

        for (var i = 0; i < request.Segments.Count; i++)
        {
            var segment = request.Segments[i];
            var result = PlanSegment(request, currentState, currentPose, segment, warnings);
            if (!result.Success || result.Trajectory is null)
                return result;

            var segPoints = result.Trajectory.Points;
            for (var p = 1; p < segPoints.Count; p++)
            {
                t += segPoints[p].TimeSeconds - segPoints[p - 1].TimeSeconds;
                points.Add(new TrajectoryPoint(
                    t,
                    segPoints[p].JointState,
                    segment.Type,
                    i,
                    segment.BlendRadiusMeters));
            }

            currentState = segPoints[^1].JointState;
            currentPose = new CartesianPose(Transforms.ToFrame(
                _fk.ComputeTcpTransform(currentState.Positions, _base.Frame, _tool.Frame)));

            if (segment.BlendRadiusMeters > 0)
            {
                warnings.Add(
                    $"Blend radius {segment.BlendRadiusMeters:F3}m requested at segment {i}; fallback to exact-stop transition.");
            }
        }

        return PlanningResult.Succeeded(new Trajectory(request.Robot, points), warnings);
    }

    private PlanningResult PlanSegment(
        MotionProgramRequest request,
        JointState currentState,
        CartesianPose currentPose,
        MotionSegment segment,
        List<string> warnings)
    {
        return segment switch
        {
            PtpSegment ptp => _joint.Plan(new PlanningRequest(
                request.Robot,
                currentState,
                ptp.Goal,
                request.Options)),
            LinSegment lin => PlanLinearSegment(request, currentState, currentPose, lin),
            CircSegment circ => PlanCircularSegment(request, currentState, currentPose, circ, warnings),
            _ => PlanningResult.Failed(new[] { "Unsupported motion segment type." })
        };
    }

    private PlanningResult PlanLinearSegment(
        MotionProgramRequest request,
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

        var checker = request.Options.CollisionChecker;
        var scene = request.Options.CollisionScene;
        if (PlanningCollision.SceneHasObstacles(scene))
        {
            checker ??= CollisionCheckerFactory.Create(request.Robot, attached: request.Options.AttachedBodies);
            if (checker is null)
                return PlanningResult.Failed(new[] { "Collision scene provided but no collision checker available." });

            var fail = PlanningCollision.ValidateTrajectory(traj, scene!, checker, request.Options.MaxJointStepRadians);
            if (fail is not null) return fail;
        }

        return PlanningResult.Succeeded(traj);
    }

    private PlanningResult PlanCircularSegment(
        MotionProgramRequest request,
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

        var checker = request.Options.CollisionChecker;
        var scene = request.Options.CollisionScene;
        if (PlanningCollision.SceneHasObstacles(scene))
        {
            checker ??= CollisionCheckerFactory.Create(request.Robot, attached: request.Options.AttachedBodies);
            if (checker is null)
                return PlanningResult.Failed(new[] { "Collision scene provided but no collision checker available." });

            var fail = PlanningCollision.ValidateTrajectory(traj, scene!, checker, request.Options.MaxJointStepRadians);
            if (fail is not null) return fail;
        }

        warnings.Add("CIRC orientation policy: SLERP from segment start orientation to final orientation.");
        return PlanningResult.Succeeded(traj);
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

        // Start is (0,0), via is (bx,0), goal is (cx,cy).
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
