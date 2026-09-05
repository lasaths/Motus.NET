using Motus.Core;

namespace Motus.Geometry;

/// <summary>True Cartesian linear (LIN) motion —— TCP follows straight line, joints via IK each step.</summary>
public sealed class CartesianLinearPathPlanner
{
    private const double IkPosTolMeters = 0.02;
    private const double IkOriTolRad = 0.15;
    /// <summary>
    /// Reject π-scale IK branch flips between waypoints. Allows a one-time ~1.8 rad
    /// reconfiguration when leaving wrist singularity (j5≈0 at default home).
    /// </summary>
    private const double MaxJointJumpRadians = 2.0;

    private readonly RobotPreset _preset;
    private readonly SerialJointChain? _chain;
    private readonly IInverseKinematics _ik;
    private readonly IFkSolver _fk;
    private readonly Random _rng;
    private readonly BaseFrame _base;
    private readonly ToolFrame _tool;
    private readonly double[] _baseM;
    private readonly double[] _toolM;

    public CartesianLinearPathPlanner(RobotPreset preset)
        : this(preset, null) { }

    public CartesianLinearPathPlanner(RobotPreset preset, SerialJointChain? chain)
    {
        _preset = preset;
        _chain = chain;
        _fk = KinematicsResolver.CreateFkSolver(preset, chain);
        _ik = KinematicsResolver.CreateInverseKinematics(preset, chain);
        _rng = new Random(42);
        _base = preset.BaseFrame;
        _tool = preset.ToolFrame;
        _baseM = Transforms.FromFrame(_base.Frame);
        _toolM = Transforms.FromFrame(_tool.Frame);
    }

    /// <summary>Plan a straight-line TCP path from startPose to goalPose.</summary>
    public Trajectory? Plan(
        CartesianPose startPose,
        CartesianPose goalPose,
        JointState startJoint,
        CartesianLinOptions options) =>
        Plan(startPose, goalPose, startJoint, options, options.ContinueOnIkFailure);

    /// <summary>Plan a straight-line TCP path from startPose to goalPose.</summary>
    /// <param name="startPose">Cartesian TCP start</param>
    /// <param name="goalPose">Cartesian TCP goal</param>
    /// <param name="startJoint">Initial joint configuration (seed for first IK)</param>
    /// <param name="stepMeters">Cartesian step size for interpolation (default: 5mm)</param>
    /// <param name="continueOnIKFailure">If true, return partial path on IK failures (default: true)</param>
    /// <returns>Trajectory or null if IK fails at intermediate poses</returns>
    public Trajectory? Plan(CartesianPose startPose, CartesianPose goalPose, JointState startJoint, double stepMeters = 0.005, bool continueOnIKFailure = true) =>
        Plan(startPose, goalPose, startJoint, new CartesianLinOptions(StepMeters: stepMeters, ContinueOnIkFailure: continueOnIKFailure), continueOnIKFailure);

    private Trajectory? Plan(
        CartesianPose startPose,
        CartesianPose goalPose,
        JointState startJoint,
        CartesianLinOptions options,
        bool continueOnIKFailure)
    {
        var elapsedFrames = 0;

        var startPos = new[] { startPose.Tcp.X, startPose.Tcp.Y, startPose.Tcp.Z };
        var goalPos = new[] { goalPose.Tcp.X, goalPose.Tcp.Y, goalPose.Tcp.Z };
        var dx = goalPos[0] - startPos[0];
        var dy = goalPos[1] - startPos[1];
        var dz = goalPos[2] - startPos[2];
        var distanceMeters = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        if (distanceMeters < 1e-9)
            return new Trajectory(new RobotModel(_preset), [new TrajectoryPoint(0, startJoint)]);

        var steps = options.StepCount(distanceMeters);

        var points = new List<TrajectoryPoint>(steps + 1);

        // PONYTAIL: First point is the start joint configuration
        var currentJoint = startJoint;
        points.Add(new TrajectoryPoint(elapsedFrames++, currentJoint));

        // Interpolate Cartesian poses, solve IK for each
        var consecutiveFailures = 0;
        const int maxConsecutiveFailures = 3;

        for (var i = 1; i <= steps; i++)
        {
            var alpha = (double)i / steps;
            var interpPose = InterpolatePose(startPose, goalPose, alpha);

            // Seed only from the previous waypoint (or local perturbations). Never accept a
            // discontinuous IK branch — that produces wrist/elbow flips that look like
            // "wild" robot paths even when TCP is linear.
            JointState? solvedJoint = null;
            if (_ik.TrySolveNear(interpPose, currentJoint, out var nearJoint))
            {
                nearJoint = UnwrapNear(currentJoint, nearJoint);
                if (PoseMatches(interpPose.Tcp, nearJoint) &&
                    MaxJointDelta(currentJoint, nearJoint) <= MaxJointJumpRadians)
                    solvedJoint = nearJoint;
            }

            var attemptSeed = currentJoint;
            for (var seedAttempts = 0; seedAttempts < options.MaxIkAttemptsPerStep && solvedJoint is null; seedAttempts++)
            {
                // TrySolveNear only — TrySolve runs a 10-seed workspace hunt and hangs LIN
                // on 180° orientation slers (pick-place home vs Z-down grasp).
                if (_ik.TrySolveNear(interpPose, attemptSeed, out var testJoint))
                {
                    testJoint = UnwrapNear(currentJoint, testJoint);
                    if (PoseMatches(interpPose.Tcp, testJoint) &&
                        MaxJointDelta(currentJoint, testJoint) <= MaxJointJumpRadians)
                        solvedJoint = testJoint;
                }

                if (solvedJoint is null)
                    attemptSeed = GenerateLocalSeed(currentJoint, seedAttempts);
            }

            if (solvedJoint is null)
            {
                if (!continueOnIKFailure)
                    return null;

                consecutiveFailures++;
                if (consecutiveFailures >= maxConsecutiveFailures)
                {
                    if (points.Count >= steps / 2)
                        break;
                    return null;
                }
                continue;
            }

            consecutiveFailures = 0;
            currentJoint = solvedJoint;
            points.Add(new TrajectoryPoint(elapsedFrames++, currentJoint));
        }

        // Snap only when every waypoint succeeded — avoid a goal jump on partial paths.
        if (points.Count == steps + 1 && _ik.TrySolveNear(goalPose, points[^2].JointState, out var finalJoint))
        {
            finalJoint = UnwrapNear(points[^2].JointState, finalJoint);
            if (PoseMatches(goalPose.Tcp, finalJoint) &&
                MaxJointDelta(points[^2].JointState, finalJoint) <= MaxJointJumpRadians)
            {
                var last = points[^1];
                points[^1] = new TrajectoryPoint(last.TimeSeconds, finalJoint);
            }
        }

        // PONYTAIL: Check we have enough points for a valid trajectory
        if (points.Count < 2)
            return null;

        // Assign physical timing from joint deltas (frame indices are not seconds).
        var robot = new RobotModel(_preset);
        var geometric = new Trajectory(robot, points);
        return TrajectoryRetimer.Retime(geometric);
    }

    /// <summary>Plan a toolpath through multiple Cartesian waypoints.</summary>
    /// <param name="waypoints">Sequence of Cartesian waypoints</param>
    /// <param name="startJoint">Initial joint configuration</param>
    /// <param name="stepMeters">Cartesian step size (default: 5mm)</param>
    /// <returns>Trajectory or null if planning fails</returns>
    public Trajectory? PlanToolpath(IEnumerable<CartesianPose> waypoints, JointState startJoint, double stepMeters = 0.005)
    {
        var wpArray = waypoints.ToArray();
        if (wpArray.Length < 1)
            return null;

        var allPoints = new List<TrajectoryPoint>();
        var currentJoint = startJoint;
        var time = 0.0;
        const double timeStep = 0.01;  // PONYTAIL: 10ms per step

        // PONYTAIL: First point
        allPoints.Add(new TrajectoryPoint(time, currentJoint));

        var segmentFailures = 0;
        const int maxSegmentFailures = 3;  // PONYTAIL: More forgiving for toolpaths

        for (var i = 0; i < wpArray.Length - 1; i++)
        {
            var from = wpArray[i];
            var to = wpArray[i + 1];

            var segment = Plan(from, to, currentJoint, stepMeters);
            if (segment is null || segment.Points.Count < 2)
            {
                // PONYTAIL: Segment failed, count and continue with partial toolpath if possible
                segmentFailures++;
                if (segmentFailures >= maxSegmentFailures)
                {
                    // Return what we have if it's still a substantial path
                    if (allPoints.Count >= 5)
                        break;  // Return partial toolpath
                    return null;  // Too many failures with insufficient points
                }
                continue;  // Skip this segment and try the next
            }

            segmentFailures = 0;  // Reset on success

            // PONYTAIL: Skip first point of each segment (duplicate unless it was first)
            var startIdx = (i == 0) ? 1 : 1;
            for (var j = startIdx; j < segment.Points.Count; j++)
            {
                time += timeStep;
                allPoints.Add(new TrajectoryPoint(time, segment.Points[j].JointState));
            }

            currentJoint = segment.Points[^1].JointState;
        }

        if (allPoints.Count < 2)
            return null;

        var robot = new RobotModel(_preset);
        return TrajectoryRetimer.Retime(new Trajectory(robot, allPoints));
    }

    private static CartesianPose InterpolatePose(CartesianPose a, CartesianPose b, double alpha)
    {
        // PONYTAIL: Linear position interpolation
        var x = a.Tcp.X + alpha * (b.Tcp.X - a.Tcp.X);
        var y = a.Tcp.Y + alpha * (b.Tcp.Y - a.Tcp.Y);
        var z = a.Tcp.Z + alpha * (b.Tcp.Z - a.Tcp.Z);

        // PONYTAIL: Spherical linear interpolation (SLERP) for quaternions
        var q = Slerp(a.Tcp.Qw, a.Tcp.Qx, a.Tcp.Qy, a.Tcp.Qz,
                     b.Tcp.Qw, b.Tcp.Qx, b.Tcp.Qy, b.Tcp.Qz, alpha);

        return new CartesianPose(new Frame(x, y, z, q.w, q.x, q.y, q.z));
    }

    private static (double w, double x, double y, double z) Slerp(
        double aw, double ax, double ay, double az,
        double bw, double bx, double by, double bz, double t)
    {
        // Quaternion SLERP
        var dot = aw * bw + ax * bx + ay * by + az * bz;

        if (dot < 0)
        {
            bw = -bw; bx = -bx; by = -by; bz = -bz;
            dot = -dot;
        }

        if (dot > 0.9995)
        {
            // PONYTAIL: Quaternions nearly parallel, linear interp
            return NormalizeQuat((
                aw + t * (bw - aw),
                ax + t * (bx - ax),
                ay + t * (by - ay),
                az + t * (bz - az)));
        }

        var theta_0 = Math.Acos(Math.Clamp(dot, -1, 1));
        var theta = theta_0 * t;
        var sinTheta = Math.Sin(theta);
        var sinTheta_0 = Math.Sin(theta_0);

        var s0 = Math.Cos(theta) - dot * sinTheta / sinTheta_0;
        var s1 = sinTheta / sinTheta_0;

        return NormalizeQuat((
            s0 * aw + s1 * bw,
            s0 * ax + s1 * bx,
            s0 * ay + s1 * by,
            s0 * az + s1 * bz));
    }

    private static (double w, double x, double y, double z) NormalizeQuat((double w, double x, double y, double z) q)
    {
        var n = Math.Sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        if (n < 1e-12) return (1, 0, 0, 0);
        return (q.w / n, q.x / n, q.y / n, q.z / n);
    }

    private bool PoseMatches(Frame targetTcp, JointState joints) =>
        Transforms.TcpMatches(
            Transforms.TcpFromJoints(_fk, joints.Positions, _baseM, _toolM),
            targetTcp,
            IkPosTolMeters,
            IkOriTolRad);

    private JointState UnwrapNear(JointState reference, JointState raw)
    {
        var q = new double[raw.AxisCount];
        for (var i = 0; i < q.Length; i++)
        {
            var v = raw.Positions[i];
            while (v - reference.Positions[i] > Math.PI) v -= 2 * Math.PI;
            while (v - reference.Positions[i] < -Math.PI) v += 2 * Math.PI;
            if (!_preset.JointLimits[i].Contains(v))
                v = raw.Positions[i];
            q[i] = v;
        }
        return new JointState(q);
    }

    private static double MaxJointDelta(JointState a, JointState b)
    {
        var max = 0.0;
        for (var i = 0; i < a.AxisCount; i++)
        {
            var d = b.Positions[i] - a.Positions[i];
            while (d > Math.PI) d -= 2 * Math.PI;
            while (d < -Math.PI) d += 2 * Math.PI;
            max = Math.Max(max, Math.Abs(d));
        }
        return max;
    }

    /// <summary>Local perturbation around the current config. Full-space random seeds cause IK branch flips.</summary>
    private JointState GenerateLocalSeed(JointState currentJoint, int attemptNumber)
    {
        var q = new double[_preset.AxisCount];
        var perturbationRadius = 0.05 + attemptNumber * 0.08;
        for (var j = 0; j < q.Length; j++)
        {
            var lim = _preset.JointLimits[j];
            var perturbation = (_rng.NextDouble() - 0.5) * 2.0 * perturbationRadius;
            q[j] = Math.Clamp(currentJoint.Positions[j] + perturbation, lim.MinRadians, lim.MaxRadians);
        }

        return new JointState(q);
    }

    /// <summary>Plan LIN from a Cartesian planning request (FK start pose → goal).</summary>
    public PlanningResult PlanToResult(CartesianPlanningRequest request, double stepMeters = 0.005) =>
        PlanToResult(request, new CartesianLinOptions(StepMeters: stepMeters));

    public PlanningResult PlanToResult(CartesianPlanningRequest request, CartesianLinOptions options)
    {
        var robot = request.Robot;
        var scene = request.CollisionScene ?? request.Options.CollisionScene ?? new CollisionScene();
        var warnings = new List<string>();

        if (!KinematicsResolver.SupportsModel(robot.Preset, _chain))
            return PlanningResult.Failed(new[] { $"No kinematics profile for '{robot.Preset.ModelName}'." });

        var startPose = new CartesianPose(Transforms.ToFrame(
            _fk.ComputeTcpTransform(request.Start.Positions, _base.Frame, _tool.Frame)));

        var startConstraintFail = ValidateConstraints(request.Options, startPose.Tcp, "start");
        if (startConstraintFail is not null) return startConstraintFail;
        var goalConstraintFail = ValidateConstraints(request.Options, request.Goal.Tcp, "goal");
        if (goalConstraintFail is not null) return goalConstraintFail;

        var workspace = CartesianWorkspace.CheckReach(robot.Preset, request.Goal, startPose);
        if (!workspace.IsWithinReach)
            return PlanningResult.Failed(new[] { workspace.Reason ?? "Goal TCP is outside robot reach." });

        var linOptions = options with { ContinueOnIkFailure = false };
        var traj = Plan(startPose, request.Goal, request.Start, linOptions);
        if (traj is null)
        {
            return PlanningResult.Failed(new[]
            {
                "TCP-LIN failed at intermediate poses. Use Joint State goal or wire Start near target."
            });
        }

        var hasCollision = PlanningCollision.SceneHasObstacles(scene) ||
                           request.Options.AttachedBodies is { Count: > 0 };
        var checker = request.Options.CollisionChecker;
        if (hasCollision && checker is null)
            checker = CollisionCheckerFactory.Create(robot, attached: request.Options.AttachedBodies);

        if (hasCollision)
        {
            if (checker is null)
                return PlanningResult.Failed(new[] { "Collision scene provided but no collision checker available." });
            if (_ik.TrySolve(request.Goal, request.Start, out var goalJoints))
            {
                var endpointFail = PlanningCollision.ValidateEndpoints(
                    request.Start, goalJoints, scene, checker,
                    request.Options.AttachedBodies is { Count: > 0 });
                if (endpointFail is not null)
                    return endpointFail;
            }

            var collisionFail = PlanningCollision.ValidateTrajectory(
                traj, scene!, checker, request.Options.MaxJointStepRadians);
            if (collisionFail is not null) return collisionFail;
            warnings.Add("CartesianLinearPathPlanner: LIN path validated against collision scene.");
        }

        var pathConstraintFail = ValidateTrajectoryConstraints(traj, request.Options);
        if (pathConstraintFail is not null) return pathConstraintFail;

        if (request.Options.RetimeTrajectory)
            traj = TrajectoryRetimer.Retime(traj);

        warnings.Add("CartesianLinearPathPlanner: true TCP-linear (LIN) motion.");
        return PlanningResult.Succeeded(traj, warnings);
    }

    private PlanningResult? ValidateTrajectoryConstraints(Trajectory trajectory, PlanningOptions options)
    {
        if (options.PathConstraints is null && options.ConstraintChecker is null)
            return null;

        foreach (var point in trajectory.Points)
        {
            var tcp = Transforms.ToFrame(_fk.ComputeTcpTransform(point.JointState.Positions, _base.Frame, _tool.Frame));
            var fail = ValidateConstraints(options, tcp, $"t={point.TimeSeconds:F4}s");
            if (fail is not null) return fail;
        }

        return null;
    }

    private static PlanningResult? ValidateConstraints(PlanningOptions options, Frame tcp, string label)
    {
        if (options.PathConstraints is not null && !options.PathConstraints.TryValidate(tcp, out var pathReason))
            return ConstraintFailure(label, pathReason);
        if (options.ConstraintChecker is not null && !options.ConstraintChecker.TryValidate(tcp, out var checkerReason))
            return ConstraintFailure(label, checkerReason);
        return null;
    }

    private static PlanningResult ConstraintFailure(string label, string reason) =>
        PlanningResult.Failed(new[]
        {
            new PlanningMessage(
                PlanningMessageCodes.ConstraintViolation,
                $"{label}: {reason}",
                PlanningMessageSeverity.Error)
        });
}
