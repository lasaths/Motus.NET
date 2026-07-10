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
