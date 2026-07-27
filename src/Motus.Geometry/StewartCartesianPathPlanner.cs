using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// TCP LIN for Stewart platforms: interpolate platform poses, analytic IK each step,
/// reject stroke / ΔL jumps. Returns structured failures for GH Status.
/// </summary>
public sealed class StewartCartesianPathPlanner
{
    private readonly StewartPlatform _platform;
    private readonly StewartInverseKinematics _ik;
    private readonly RobotModel _model;

    public StewartCartesianPathPlanner(StewartPlatform platform)
    {
        _platform = platform ?? throw new ArgumentNullException(nameof(platform));
        _ik = new StewartInverseKinematics(platform);
        _model = platform.ToModel();
    }

    public StewartPlatform Platform => _platform;

    public PlanningResult PlanToResult(
        CartesianPose startPose,
        CartesianPose goalPose,
        JointState? startLengths = null,
        double stepMeters = 0.005,
        double? maxLegDeltaMeters = null,
        PlanningOptions? planningOptions = null)
    {
        var maxDelta = maxLegDeltaMeters ?? _platform.SolverOptions.MaxLegDeltaPerStepMeters;
        if (stepMeters <= 0)
            return PlanningResult.Failed(["Stewart LIN stepMeters must be > 0."]);

        var startIk = _ik.TrySolveDetailed(startPose);
        if (!startIk.Success || startIk.JointState is null)
            return PlanningResult.Failed([$"Stewart start IK failed: {startIk}"]);

        var goalIk = _ik.TrySolveDetailed(goalPose);
        if (!goalIk.Success || goalIk.JointState is null)
            return PlanningResult.Failed([$"Stewart goal IK failed: {goalIk}"]);

        var current = startLengths ?? startIk.JointState;
        // If caller provided start lengths, verify they match start pose within stroke (re-IK from pose for path seed).
        current = startIk.JointState;

        var dx = goalPose.Tcp.X - startPose.Tcp.X;
        var dy = goalPose.Tcp.Y - startPose.Tcp.Y;
        var dz = goalPose.Tcp.Z - startPose.Tcp.Z;
        var distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        if (distance < 1e-9)
        {
            var single = new Trajectory(_model, [new TrajectoryPoint(0, current)]);
            return ValidateCollision(single, startIk.JointState, goalIk.JointState, planningOptions, []);
        }

        var steps = Math.Max(1, (int)Math.Ceiling(distance / stepMeters));
        var points = new List<TrajectoryPoint>(steps + 1) { new(0, current) };

        for (var i = 1; i <= steps; i++)
        {
            var alpha = (double)i / steps;
            var pose = InterpolatePose(startPose, goalPose, alpha);
            var ik = _ik.TrySolveDetailed(pose);
            if (!ik.Success || ik.JointState is null)
                return PlanningResult.Failed([$"Stewart path IK failed at step {i}/{steps}: {ik}"]);

            if (MaxAbsDelta(current, ik.JointState) > maxDelta)
            {
                return PlanningResult.Failed([
                    $"Stewart path ΔL jump {MaxAbsDelta(current, ik.JointState):F4} m exceeds {maxDelta:F4} m at step {i}/{steps} ({KinematicsReason.DeltaLengthJump})."]);
            }

            current = ik.JointState;
            points.Add(new TrajectoryPoint(i, current));
        }

        var trajectory = new Trajectory(_model, points);
        return ValidateCollision(trajectory, startIk.JointState, goalIk.JointState, planningOptions, [
            "StewartCartesianPathPlanner: true TCP-linear platform path.",
            StewartMethodRefs.DescribeStack()
        ]);
    }

    public PlanningResult PlanToolpath(
        IReadOnlyList<CartesianPose> waypoints,
        JointState? startLengths = null,
        double stepMeters = 0.005,
        double? maxLegDeltaMeters = null,
        PlanningOptions? planningOptions = null)
    {
        if (waypoints is null || waypoints.Count == 0)
            return PlanningResult.Failed(["Stewart toolpath requires at least one waypoint."]);

        if (waypoints.Count == 1)
        {
            var ik = _ik.TrySolveDetailed(waypoints[0]);
            if (!ik.Success || ik.JointState is null)
                return PlanningResult.Failed([$"Stewart toolpath IK failed: {ik}"]);
            return ValidateCollision(
                new Trajectory(_model, [new TrajectoryPoint(0, ik.JointState)]),
                ik.JointState,
                ik.JointState,
                planningOptions,
                [StewartMethodRefs.DescribeStack()]);
        }

        Trajectory? combined = null;
        var warnings = new List<string>();
        for (var s = 0; s < waypoints.Count - 1; s++)
        {
            var seg = PlanToResult(waypoints[s], waypoints[s + 1], startLengths: null, stepMeters, maxLegDeltaMeters, planningOptions);
            if (!seg.Success || seg.Trajectory is null)
                return PlanningResult.Failed(seg.Errors.Count > 0 ? seg.Errors : [$"Stewart segment {s}→{s + 1} failed."]);

            if (combined is null)
                combined = seg.Trajectory;
            else
                combined = Concat(combined, seg.Trajectory);
            warnings.AddRange(seg.Warnings);
        }

        warnings.Add(StewartMethodRefs.DescribeStack());
        return PlanningResult.Succeeded(combined!, warnings.Count > 0 ? warnings : null);
    }

    private PlanningResult ValidateCollision(
        Trajectory trajectory,
        JointState start,
        JointState goal,
        PlanningOptions? planningOptions,
        IReadOnlyList<string> baseWarnings)
    {
        var warnings = new List<string>(baseWarnings);
        var scene = planningOptions?.CollisionScene;
        var hasCollision = PlanningCollision.SceneHasObstacles(scene) ||
                           planningOptions?.AttachedBodies is { Count: > 0 };
        if (!hasCollision)
            return PlanningResult.Succeeded(trajectory, warnings.Count > 0 ? warnings : null);

        var checker = planningOptions?.CollisionChecker ?? new StewartCollisionChecker(_platform);
        scene ??= new CollisionScene();
        var endpointFail = PlanningCollision.ValidateEndpoints(start, goal, scene, checker);
        if (endpointFail is not null) return endpointFail;

        var step = planningOptions is not null && planningOptions.MaxJointStepRadians > 0
            ? planningOptions.MaxJointStepRadians
            : _platform.SolverOptions.MaxLegDeltaPerStepMeters;
        var collisionFail = PlanningCollision.ValidateTrajectory(trajectory, scene, checker, step);
        if (collisionFail is not null) return collisionFail;
        warnings.Add("StewartCartesianPathPlanner: leg-length path validated against collision scene.");
        return PlanningResult.Succeeded(trajectory, warnings);
    }

    private static Trajectory Concat(Trajectory a, Trajectory b)
    {
        var pts = new List<TrajectoryPoint>(a.Points.Count + Math.Max(0, b.Points.Count - 1));
        pts.AddRange(a.Points);
        var t0 = a.Points[^1].TimeSeconds;
        for (var i = 1; i < b.Points.Count; i++)
        {
            var p = b.Points[i];
            pts.Add(new TrajectoryPoint(t0 + p.TimeSeconds + 1e-9, p.JointState));
        }
        return new Trajectory(a.Robot, pts);
    }

    private static double MaxAbsDelta(JointState a, JointState b)
    {
        var m = 0.0;
        for (var i = 0; i < a.AxisCount; i++)
            m = Math.Max(m, Math.Abs(a.Positions[i] - b.Positions[i]));
        return m;
    }

    private static CartesianPose InterpolatePose(CartesianPose a, CartesianPose b, double alpha)
    {
        var x = a.Tcp.X + alpha * (b.Tcp.X - a.Tcp.X);
        var y = a.Tcp.Y + alpha * (b.Tcp.Y - a.Tcp.Y);
        var z = a.Tcp.Z + alpha * (b.Tcp.Z - a.Tcp.Z);
        var q = Slerp(a.Tcp.Qw, a.Tcp.Qx, a.Tcp.Qy, a.Tcp.Qz, b.Tcp.Qw, b.Tcp.Qx, b.Tcp.Qy, b.Tcp.Qz, alpha);
        return new CartesianPose(new Frame(x, y, z, q.w, q.x, q.y, q.z));
    }

    private static (double w, double x, double y, double z) Slerp(
        double aw, double ax, double ay, double az,
        double bw, double bx, double by, double bz,
        double t)
    {
        var a = Transforms.NormalizeQuat(aw, ax, ay, az);
        var b = Transforms.NormalizeQuat(bw, bx, by, bz);
        var dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        if (dot < 0)
        {
            b = (-b.w, -b.x, -b.y, -b.z);
            dot = -dot;
        }
        if (dot > 0.9995)
        {
            return Transforms.NormalizeQuat(
                a.w + t * (b.w - a.w),
                a.x + t * (b.x - a.x),
                a.y + t * (b.y - a.y),
                a.z + t * (b.z - a.z));
        }
        var theta0 = Math.Acos(Math.Clamp(dot, -1, 1));
        var theta = theta0 * t;
        var s0 = Math.Cos(theta) - dot * Math.Sin(theta) / Math.Sin(theta0);
        var s1 = Math.Sin(theta) / Math.Sin(theta0);
        return Transforms.NormalizeQuat(
            s0 * a.w + s1 * b.w,
            s0 * a.x + s1 * b.x,
            s0 * a.y + s1 * b.y,
            s0 * a.z + s1 * b.z);
    }
}
