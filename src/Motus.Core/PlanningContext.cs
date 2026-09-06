namespace Motus.Core;

/// <summary>Mutable planning session: robot, obstacles, attached bodies.</summary>
public sealed class PlanningContext
{
    public RobotModel Robot { get; }
    public CollisionScene Scene { get; }
    public IReadOnlyList<AttachedBody> Attached { get; }
    public PlanningGroup? ActiveGroup { get; }

    private PlanningContext(RobotModel robot, CollisionScene scene, IReadOnlyList<AttachedBody> attached, PlanningGroup? group)
    {
        Robot = robot;
        Scene = scene;
        Attached = attached;
        ActiveGroup = group;
    }

    public static PlanningContext Create(RobotModel robot, CollisionScene? scene = null) =>
        new(robot, scene ?? new CollisionScene(), Array.Empty<AttachedBody>(), null);

    public PlanningContext ForGroup(PlanningGroup group) =>
        new(Robot, Scene, Attached, group);

    /// <summary>Attach geometry at TCP; hides matching scene obstacle by name.</summary>
    public PlanningContext Attach(string sceneObjectName, CollisionObject geometry, Frame tcpLocalPose)
    {
        var attached = Attached.ToList();
        attached.Add(new AttachedBody(sceneObjectName, tcpLocalPose, geometry, sceneObjectName));
        var filtered = Scene.Objects.Where(o => !string.Equals(o.Name, sceneObjectName, StringComparison.OrdinalIgnoreCase)).ToList();
        return new PlanningContext(Robot, new CollisionScene(filtered, Scene.AllowedPairs), attached, ActiveGroup);
    }

    public PlanningContext Attach(AttachedBody body)
    {
        var attached = Attached.ToList();
        attached.Add(body);
        var filtered = body.SourceSceneObjectName is { } src
            ? Scene.Objects.Where(o => !string.Equals(o.Name, src, StringComparison.OrdinalIgnoreCase)).ToList()
            : Scene.Objects.ToList();
        return new PlanningContext(Robot, new CollisionScene(filtered, Scene.AllowedPairs), attached, ActiveGroup);
    }

    public PlanningContext Detach(string name, Frame worldPose)
    {
        var body = Attached.FirstOrDefault(a => string.Equals(a.Name, name, StringComparison.OrdinalIgnoreCase));
        if (body is null) return this;

        var attached = Attached.Where(a => !string.Equals(a.Name, name, StringComparison.OrdinalIgnoreCase)).ToList();
        var objects = Scene.Objects.ToList();
        if (body.SourceSceneObjectName is not null)
            objects.Add(TransformGeometryToWorld(body.Geometry, worldPose));
        return new PlanningContext(Robot, new CollisionScene(objects, Scene.AllowedPairs), attached, ActiveGroup);
    }

    private static CollisionObject TransformGeometryToWorld(CollisionObject local, Frame worldPose)
    {
        return local.Shape switch
        {
            CollisionShape.Sphere => CollisionObject.Sphere(local.Name, worldPose, local.ExtentX),
            CollisionShape.Box => CollisionObject.Box(local.Name, worldPose, local.ExtentX, local.ExtentY, local.ExtentZ),
            CollisionShape.Capsule => CollisionObject.Capsule(local.Name, worldPose, local.ExtentX, local.ExtentY),
            CollisionShape.Mesh => CollisionObject.Mesh(local.Name, worldPose, local.MeshVertices!, local.MeshIndices!),
            _ => local
        };
    }

    public PlanningOptions ToPlanningOptions(PlanningOptions? baseOptions = null)
    {
        var opts = baseOptions ?? new PlanningOptions();
        return new PlanningOptions
        {
            MaxJointStepRadians = opts.MaxJointStepRadians,
            TimeStepSeconds = opts.TimeStepSeconds,
            MaxJointVelocityRadiansPerSecond = opts.MaxJointVelocityRadiansPerSecond,
            CollisionScene = Scene,
            CollisionChecker = opts.CollisionChecker,
            RetimeTrajectory = opts.RetimeTrajectory,
            AttachedBodies = Attached,
            PathConstraints = opts.PathConstraints,
            ConstraintChecker = opts.ConstraintChecker,
            GroupMap = ActiveGroup is not null ? JointIndexMap.Resolve(Robot, ActiveGroup) : opts.GroupMap,
            Mobility = opts.Mobility,
            MobilityBounds = opts.MobilityBounds
        };
    }
}
