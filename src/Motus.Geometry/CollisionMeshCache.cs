using Motus.Core;

namespace Motus.Geometry;

/// <summary>Shared mesh BVH cache keyed by obstacle geometry identity (survives checker instances).</summary>
public static class CollisionMeshCache
{
    private static readonly Dictionary<int, BvhNode> Cache = new();
    private static readonly object Gate = new();

    public static BvhNode GetOrBuild(CollisionObject meshObj)
    {
        if (meshObj.Shape != CollisionShape.Mesh ||
            meshObj.MeshVertices is null || meshObj.MeshIndices is null)
            throw new ArgumentException("CollisionMeshCache requires a mesh CollisionObject.", nameof(meshObj));

        var key = GeometryFingerprint(meshObj);
        lock (Gate)
        {
            if (Cache.TryGetValue(key, out var node))
                return node;
            node = BvhBuilder.Build(meshObj);
            Cache[key] = node;
            return node;
        }
    }

    public static bool TryGet(CollisionObject meshObj, out BvhNode? node)
    {
        node = null;
        if (meshObj.Shape != CollisionShape.Mesh ||
            meshObj.MeshVertices is null || meshObj.MeshIndices is null)
            return false;

        var key = GeometryFingerprint(meshObj);
        lock (Gate)
        {
            if (!Cache.TryGetValue(key, out var cached))
                return false;
            node = cached;
            return true;
        }
    }

    public static void Clear() { lock (Gate) Cache.Clear(); }

    /// <summary>Uses <see cref="CollisionObject.ContentHash"/> computed once at construction.</summary>
    internal static int GeometryFingerprint(CollisionObject obj) => obj.ContentHash;
}
