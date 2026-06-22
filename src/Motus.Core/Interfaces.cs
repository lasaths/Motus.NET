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
}
