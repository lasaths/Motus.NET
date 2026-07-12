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

    internal static int GeometryFingerprint(CollisionObject obj)
    {
        var hash = new HashCode();
        hash.Add(obj.Name, StringComparer.Ordinal);
        hash.Add(obj.ExtentX);
        hash.Add(obj.ExtentY);
        hash.Add(obj.ExtentZ);
        if (obj.MeshVertices is { } verts)
        {
            hash.Add(verts.Count);
            foreach (var v in verts)
            {
                hash.Add(v[0]);
                hash.Add(v[1]);
                hash.Add(v[2]);
            }
        }
        if (obj.MeshIndices is { } indices)
        {
            hash.Add(indices.Count);
            foreach (var i in indices)
                hash.Add(i);
        }
        return hash.ToHashCode();
    }
}
