namespace Motus.Core;

public sealed class CartesianPlanningRequest
{
    public RobotModel Robot { get; }
    public JointState Start { get; }
    public CartesianPose Goal { get; }
    public PlanningOptions Options { get; }
    public CollisionScene? CollisionScene { get; }

    public CartesianPlanningRequest(
        RobotModel robot,
        JointState start,
        CartesianPose goal,
        PlanningOptions? options = null,
        CollisionScene? collisionScene = null)
    {
        Robot = robot;
        Start = start;
        Goal = goal;
        Options = options ?? new PlanningOptions();
        CollisionScene = collisionScene;
    }
}
