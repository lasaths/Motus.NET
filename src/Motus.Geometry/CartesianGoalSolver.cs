using Motus.Core;

namespace Motus.Geometry;

/// <summary>Multi-seed IK solver for Cartesian goal poses.</summary>
public sealed class CartesianGoalSolver
{
    public CartesianReachResult TryReach(
        RobotModel robot,
        CartesianPose goal,
        IEnumerable<JointState> seeds,
        SerialJointChain? chain = null)
    {
        if (!KinematicsResolver.SupportsModel(robot.Preset, chain))
        {
            return CartesianReachResult.Failed(
                $"No kinematics profile for '{robot.Preset.ModelName}'.");
        }

        var ik = KinematicsResolver.CreateInverseKinematics(robot.Preset, chain);
        var numerical = ik as NumericalInverseKinematics;
        JointState? best = null;
        var bestDelta = double.MaxValue;
        JointState? reference = null;
        string? lastFailure = null;
        foreach (var seed in seeds)
        {
            reference ??= seed;
            JointState solution;
            if (numerical is not null)
            {
                var detailed = numerical.TrySolveDetailed(goal, seed);
                if (!detailed.Success)
                {
                    lastFailure = PreferFailureReason(lastFailure, detailed.FailureReason);
                    continue;
                }

                solution = detailed.Solution;
            }
            else if (!ik.TrySolve(goal, seed, out solution))
            {
                continue;
            }

            var delta = MaxAbsJointDelta(reference, solution);
            if (delta >= bestDelta)
                continue;

            bestDelta = delta;
            best = solution;
            // Close enough to the start seed — no need to hunt random configs.
            if (delta < 0.35)
                break;
        }

        if (best is not null)
            return CartesianReachResult.Succeeded(best);

        var reason = lastFailure ?? "failed";
        return CartesianReachResult.Failed(
            $"Goal TCP is not reachable (IK {reason}). Wire Motus TCP Pose for valid orientation.");
    }

    private static string PreferFailureReason(string? current, string? next)
    {
        if (string.IsNullOrEmpty(next)) return current ?? NumericalIkFailureReasons.NoConvergence;
        // Prefer named structural failures over generic non-convergence when any seed hit them.
        if (next == NumericalIkFailureReasons.InvalidInput ||
            next == NumericalIkFailureReasons.SingularJacobian)
            return next;
        return current ?? next;
    }

    private static double MaxAbsJointDelta(JointState a, JointState b)
    {
        var max = 0.0;
        var n = Math.Min(a.AxisCount, b.AxisCount);
        for (var i = 0; i < n; i++)
        {
            var d = b.Positions[i] - a.Positions[i];
            while (d > Math.PI) d -= 2 * Math.PI;
            while (d < -Math.PI) d += 2 * Math.PI;
            max = Math.Max(max, Math.Abs(d));
        }
        return max;
    }

    /// <summary>Multi-seed IK with joint-space homotopy fallback for URDF / numerical chains.</summary>
    public static CartesianReachResult TryReachFromStart(
        RobotModel robot,
        CartesianPose goal,
        JointState start,
        SerialJointChain? chain = null,
        int homotopySteps = 4)
    {
        var seeds = EnumerateDefaultSeeds(start, robot);
        var solver = new CartesianGoalSolver();
        var reach = solver.TryReach(robot, goal, seeds, chain);
        if (reach.Success || chain is null || homotopySteps <= 0) return reach;

        for (var step = 1; step <= homotopySteps; step++)
        {
            var t = step / (double)homotopySteps;
            foreach (var targetSeed in EnumerateDefaultSeeds(start, robot))
            {
                if (targetSeed.AxisCount != start.AxisCount) continue;

                var q = new double[start.AxisCount];
                for (var j = 0; j < q.Length; j++)
                    q[j] = start.Positions[j] + t * (targetSeed.Positions[j] - start.Positions[j]);

                var attempt = solver.TryReach(robot, goal, new[] { new JointState(q) }, chain);
                if (attempt.Success) return attempt;
            }
        }

        return reach;
    }

    public static IEnumerable<JointState> EnumerateDefaultSeeds(JointState start, RobotModel robot)
    {
        yield return start;
        yield return new JointState(new double[robot.Preset.AxisCount]);
        var rng = new Random(42);
        for (var i = 0; i < 12; i++)
        {
            var q = new double[robot.Preset.AxisCount];
            for (var j = 0; j < q.Length; j++)
            {
                var lim = robot.Preset.JointLimits[j];
                q[j] = lim.MinRadians + rng.NextDouble() * (lim.MaxRadians - lim.MinRadians);
            }
            yield return new JointState(q);
        }
    }
}
