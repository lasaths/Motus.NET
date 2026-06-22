namespace Motus.Core;

public enum CollisionShape
{
    Sphere,
    Box
}

public sealed class CollisionObject
{
    public string Name { get; }
    public Frame Pose { get; }
    public CollisionShape Shape { get; }
  /// <summary>Sphere radius or box half-extent X (meters).</summary>
    public double ExtentX { get; }
    public double ExtentY { get; }
    public double ExtentZ { get; }

    public CollisionObject(string name, Frame pose, CollisionShape shape, double extentX, double extentY = 0, double extentZ = 0)
    {
        Name = name;
        Pose = pose;
        Shape = shape;
        ExtentX = extentX;
        ExtentY = shape == CollisionShape.Box ? extentY : extentX;
        ExtentZ = shape == CollisionShape.Box ? extentZ : extentX;
    }

    public static CollisionObject Sphere(string name, Frame pose, double radiusMeters) =>
        new(name, pose, CollisionShape.Sphere, radiusMeters);

    public static CollisionObject Box(string name, Frame pose, double halfX, double halfY, double halfZ) =>
        new(name, pose, CollisionShape.Box, halfX, halfY, halfZ);
}

public sealed class CollisionScene
{
    public IReadOnlyList<CollisionObject> Objects { get; }

    public CollisionScene(IReadOnlyList<CollisionObject>? objects = null)
    {
        Objects = objects ?? Array.Empty<CollisionObject>();
    }
}
