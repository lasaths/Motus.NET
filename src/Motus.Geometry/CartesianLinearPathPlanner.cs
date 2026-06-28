using Motus.Core;

namespace Motus.Geometry;

/// <summary>True Cartesian linear (LIN) motion —— TCP follows straight line, joints via IK each step.</summary>
public sealed class CartesianLinearPathPlanner
{
    private readonly RobotPreset _preset;
    private readonly IInverseKinematics _ik;
    private readonly DhForwardKinematics _fk;
    private readonly Random _rng;

    public CartesianLinearPathPlanner(RobotPreset preset)
    {
        _preset = preset;
        _ik = KinematicsResolver.CreateInverseKinematics(preset);
        _fk = new DhForwardKinematics(preset);
        _rng = new Random(42);  // PONYTAIL: Deterministic for testing
    }

    /// <summary>Plan a straight-line TCP path from startPose to goalPose.</summary>
    /// <param name="startPose">Cartesian TCP start</param>
    /// <param name="goalPose">Cartesian TCP goal</param>
    /// <param name="startJoint">Initial joint configuration (seed for first IK)</param>
    /// <param name="stepMeters">Cartesian step size for interpolation (default: 5mm)</param>
    /// <param name="continueOnIKFailure">If true, return partial path on IK failures (default: true)</param>
    /// <returns>Trajectory or null if IK fails at intermediate poses</returns>
    public Trajectory? Plan(CartesianPose startPose, CartesianPose goalPose, JointState startJoint, double stepMeters = 0.005, bool continueOnIKFailure = true)
    {
        var elapsedFrames = 0;

        // PONYTAIL: Compute number of steps from linear distance
        var startPos = new[] { startPose.Tcp.X, startPose.Tcp.Y, startPose.Tcp.Z };
        var goalPos = new[] { goalPose.Tcp.X, goalPose.Tcp.Y, goalPose.Tcp.Z };
        var dx = goalPos[0] - startPos[0];
        var dy = goalPos[1] - startPos[1];
        var dz = goalPos[2] - startPos[2];
        var distanceMeters = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        var steps = Math.Max(2, (int)Math.Ceiling(distanceMeters / stepMeters));

        var points = new List<TrajectoryPoint>(steps + 1);

        // PONYTAIL: First point is the start joint configuration
        var currentJoint = startJoint;
        points.Add(new TrajectoryPoint(elapsedFrames++, currentJoint));

        // Interpolate Cartesian poses, solve IK for each
        var seed = startJoint;
        var consecutiveFailures = 0;
        const int maxConsecutiveFailures = 3;  // PONYTAIL: Allow some failures but stop path if too many

        for (var i = 1; i <= steps; i++)
        {
            var alpha = (double)i / steps;
            var interpPose = InterpolatePose(startPose, goalPose, alpha);

            // PONYTAIL: Try IK with geometric seed strategy
            JointState? solvedJoint = null;
            for (var seedAttempts = 0; seedAttempts < 20 && solvedJoint is null; seedAttempts++)
            {
                if (_ik.TrySolve(interpPose, seed, out var testJoint))
                {
                    var maxDelta = MaxJointDelta(currentJoint, testJoint);
                    if (maxDelta < 3.0)  // PONYTAIL: Allow up to ~170° jumps to handle singularities
                    {
                        solvedJoint = testJoint;
                    }
                }

                if (solvedJoint is null)
                {
                    seed = GenerateGeometricSeed(currentJoint, seedAttempts);
                }
            }

            if (solvedJoint is null)
            {
                // PONYTAIL: IK failed at this waypoint
                if (!continueOnIKFailure)
                    return null;

                consecutiveFailures++;
                if (consecutiveFailures >= maxConsecutiveFailures)
                {
                    // PONYTAIL: Allow partial path if we have enough points, otherwise fail
                    if (points.Count >= steps / 2)
                        break;  // Return partial path
                    return null;
                }
                continue;
            }

            consecutiveFailures = 0;  // Reset on success
            // PONYTAIL: Prefer solutions close to previous waypoint (continuity)
            if (MaxJointDelta(currentJoint, solvedJoint) < 1.0)  // ~57° tolerance for continuity preference
            {
                seed = solvedJoint;
            }
            else
            {
                seed = GenerateGeometricSeed(solvedJoint, 5);  // Use perturbed search next iteration
            }
            currentJoint = solvedJoint;
            points.Add(new TrajectoryPoint(elapsedFrames++, currentJoint));
        }

        // PONYTAIL: Check we have enough points for a valid trajectory
        if (points.Count < 2)
            return null;

        // PONYTAIL: Assign simple timing (1 frame per step, constant velocity)
        var robot = new RobotModel(_preset);
        return new Trajectory(robot, points);
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
        return new Trajectory(robot, allPoints);
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
        // PONYTAIL: Quaternion SLERP
var dot = aw * bw + ax * bx + ay * by + az * bz;

        if (dot < 0)
        {
            bw = -bw; bx = -bx; by = -by; bz = -bz;
            dot = -dot;
        }

        if (dot > 0.9995)
        {
            // PONYTAIL: Quaternions nearly parallel, linear interp
            return (aw + t * (bw - aw),
                    ax + t * (bx - ax),
                    ay + t * (by - ay),
                    az + t * (bz - az));
        }

        var theta_0 = Math.Acos(Math.Clamp(dot, -1, 1));
        var theta = theta_0 * t;
        var sinTheta = Math.Sin(theta);
        var sinTheta_0 = Math.Sin(theta_0);

        var s0 = Math.Cos(theta) - dot * sinTheta / sinTheta_0;
        var s1 = sinTheta / sinTheta_0;

        return (s0 * aw + s1 * bw,
                s0 * ax + s1 * bx,
                s0 * ay + s1 * by,
                s0 * az + s1 * bz);
    }

    private static double MaxJointDelta(JointState a, JointState b)
    {
        var max = 0.0;
        for (var i = 0; i < a.AxisCount; i++)
            max = Math.Max(max, Math.Abs(b.Positions[i] - a.Positions[i]));
        return max;
    }

    /// <summary>Generate geometric seed for IK solver with progressive search strategy.</summary>
    /// <param name="currentJoint">Current joint configuration (for local search)</param>
    /// <param name="attemptNumber">Retry attempt number (0-4: local, 5-19: global)</param>
    /// <returns>Generated joint seed for next IK attempt</returns>
    private JointState GenerateGeometricSeed(JointState currentJoint, int attemptNumber)
    {
        var q = new double[_preset.AxisCount];
        
        // PONYTAIL: Attempts 0-4: Progressive perturbation around current joint (local search)
        if (attemptNumber < 5)
        {
            // PONYTAIL: Grow perturbation radius with attempt number: 0.05 -> 0.25 rad
            var perturbationRadius = 0.05 + attemptNumber * 0.05;
            
            for (var j = 0; j < q.Length; j++)
            {
                var lim = _preset.JointLimits[j];
                // Perturb current joint with gaussian-like distribution
                var perturbation = (_rng.NextDouble() - 0.5) * 2.0 * perturbationRadius;
                q[j] = Math.Clamp(currentJoint.Positions[j] + perturbation, lim.MinRadians, lim.MaxRadians);
            }
        }
        // PONYTAIL: Attempts 5-19: Uniform workspace random (global search)
        else
        {
            for (var j = 0; j < q.Length; j++)
            {
                var lim = _preset.JointLimits[j];
                // Uniform random across joint limits
                q[j] = lim.MinRadians + _rng.NextDouble() * (lim.MaxRadians - lim.MinRadians);
            }
        }
        
        return new JointState(q);
    }
}
