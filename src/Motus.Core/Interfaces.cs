namespace Motus.Core;

public interface IPlanner
{
    PlanningResult Plan(PlanningRequest request);
}

public interface ITrajectoryValidator
{
    ValidationResult Validate(Trajectory trajectory);
}

public interface IForwardKinematics
{
    CartesianPose ComputeTcp(JointState state, BaseFrame baseFrame, ToolFrame toolFrame);
}

public interface IInverseKinematics
{
    bool TrySolve(CartesianPose target, JointState seed, out JointState solution);
}

public interface ICollisionChecker
{
    bool IsCollisionFree(JointState state, CollisionScene scene);

    /// <summary>Discrete segment check; default samples intermediate configurations.</summary>
    bool SegmentCollisionFree(JointState from, JointState to, CollisionScene scene, double stepRadians)
    {
        var n = from.AxisCount;
        var maxDelta = 0.0;
        for (var i = 0; i < n; i++)
            maxDelta = Math.Max(maxDelta, Math.Abs(to.Positions[i] - from.Positions[i]));
        var steps = Math.Max(1, (int)Math.Ceiling(maxDelta / stepRadians));
        for (var s = 0; s <= steps; s++)
        {
            var alpha = (double)s / steps;
            var q = new double[n];
            for (var i = 0; i < n; i++)
                q[i] = from.Positions[i] + alpha * (to.Positions[i] - from.Positions[i]);
            if (!IsCollisionFree(new JointState(q), scene))
                return false;
        }
        return true;
    }
}
