namespace Motus.Core;

public sealed class CollisionObject
{
    public string Name { get; }
    public Frame Pose { get; }

    public CollisionObject(string name, Frame pose)
    {
        Name = name;
        Pose = pose;
    }
}

public sealed class CollisionScene
{
    public IReadOnlyList<CollisionObject> Objects { get; }

    public CollisionScene(IReadOnlyList<CollisionObject>? objects = null)
    {
        Objects = objects ?? Array.Empty<CollisionObject>();
    }
}
