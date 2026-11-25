using Motus.Core;

namespace Motus.Geometry;

public static class CollisionCheckerFactory
{
    public static ICollisionChecker Create(
        RobotModel robot,
        SerialJointChain? chain = null,
        IReadOnlyList<AttachedBody>? attached = null) =>
        CreateCore(robot, chain, attached, tree: null, planJointNames: null, treeDriverHome: null);

    /// <summary>
    /// TreeFK collision when Plan DOF includes side-branch drivers (e.g. DKP beside arm). Tree link-to-link collision is
    /// on (<see cref="DefaultTreeSelfCollision"/>); scene allowed pairs (SRDF) also skip link pairs. Opt-out: build
    /// <c>new TreeFkCollisionChecker(..., selfCollision: null)</c> and pass it as <c>PlanningOptions.CollisionChecker</c>.
    /// </summary>
    public static ICollisionChecker Create(
        RobotModel robot,
        KinematicTree tree,
        SerialJointChain? tipChain,
        IReadOnlyList<string>? planJointNames,
        IReadOnlyList<double>? treeDriverHome = null,
        IReadOnlyList<AttachedBody>? attached = null) =>
        CreateCore(robot, tipChain, attached, tree, planJointNames, treeDriverHome);

    public static ICollisionChecker GetOrCreate(
        RobotModel robot,
        SerialJointChain? chain,
        IReadOnlyList<AttachedBody>? attached,
        CollisionScene? scene) =>
        CollisionCheckerSessionCache.GetOrCreate(robot, chain, attached, scene);

    public static ICollisionChecker GetOrCreate(
        RobotModel robot,
        KinematicTree tree,
        SerialJointChain? tipChain,
        IReadOnlyList<string>? planJointNames,
        IReadOnlyList<double>? treeDriverHome,
        IReadOnlyList<AttachedBody>? attached,
        CollisionScene? scene) =>
        CollisionCheckerSessionCache.GetOrCreate(robot, tipChain, attached, scene, tree, planJointNames, treeDriverHome);

    private static ICollisionChecker CreateCore(
        RobotModel robot,
        SerialJointChain? chain,
        IReadOnlyList<AttachedBody>? attached,
        KinematicTree? tree,
        IReadOnlyList<string>? planJointNames,
        IReadOnlyList<double>? treeDriverHome)
    {
        var tipN = chain?.Joints.Length ?? 0;
        var useTree = tree is not null
            && robot.CollisionModel is { Links.Count: > 0 }
            && (robot.Preset.AxisCount > tipN || tipN == 0);

        if (useTree)
            return new TreeFkCollisionChecker(robot, tree!, chain, planJointNames, treeDriverHome, attached,
                DefaultTreeSelfCollision(tree!, robot.CollisionModel!));

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

    /// <summary>
    /// Self-collision used by the factory for trees: no clearance (as the serial self-check; planners grow obstacles
    /// themselves), and the links that touch by design are allowed: adjacent links, fixed siblings, links joined only by
    /// fixed joints, and links of one mimic group (gripper fingers). SRDF pairs come in through the scene.
    /// </summary>
    public static TreeSelfCollision DefaultTreeSelfCollision(KinematicTree tree, RobotCollisionModel collision) =>
        new(0, TreeAdjacency.DefaultPairs(tree, collision.Links.Select(l => l.LinkName).ToHashSet(StringComparer.Ordinal)));
}
