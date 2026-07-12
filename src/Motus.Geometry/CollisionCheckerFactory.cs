using Motus.Core;

namespace Motus.Geometry;

public static class CollisionCheckerFactory
{
    public static ICollisionChecker Create(
        RobotModel robot,
        SerialJointChain? chain = null,
        IReadOnlyList<AttachedBody>? attached = null) =>
        CreateCore(robot, chain, attached);

    public static ICollisionChecker GetOrCreate(
        RobotModel robot,
        SerialJointChain? chain,
        IReadOnlyList<AttachedBody>? attached,
        CollisionScene? scene) =>
        CollisionCheckerSessionCache.GetOrCreate(robot, chain, attached, scene);

    private static ICollisionChecker CreateCore(
        RobotModel robot,
        SerialJointChain? chain,
        IReadOnlyList<AttachedBody>? attached)
    {
        if (FclCollisionChecker.SupportsFcl(robot, attached))
            return new FclCollisionChecker(robot, chain, attached);

        var fk = KinematicsResolver.CreateFkSolver(robot.Preset, chain);
        ICollisionChecker inner;
        if (robot.CollisionModel is not null && (robot.CollisionModel.Links.Count > 0 || robot.CollisionModel.ToolGeometry is not null))
            inner = new RobotMeshCollisionChecker(robot, chain, attached);
        else
            inner = chain is null
                ? new SphereCollisionChecker(robot.Preset)
                : new SphereCollisionChecker(robot.Preset, chain);

        if (attached is { Count: > 0 } && inner is not RobotMeshCollisionChecker)
            return new AttachAwareCollisionChecker(inner, fk, robot.Preset.BaseFrame, robot.Preset.ToolFrame, attached);
        return inner;
    }
}
