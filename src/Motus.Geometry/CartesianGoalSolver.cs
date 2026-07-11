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
        foreach (var seed in seeds)
        {
            if (ik.TrySolve(goal, seed, out var solution))
                return CartesianReachResult.Succeeded(solution);
        }

        return CartesianReachResult.Failed(
            "Goal TCP is not reachable (IK failed). Wire Motus TCP Pose for valid orientation.");
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
