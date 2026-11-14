using Motus.Core;

namespace Motus.Geometry;

/// <summary>Shared mesh BVH cache keyed by obstacle geometry identity (survives checker instances).</summary>
public static class CollisionMeshCache
{
    // ponytail: procedurally generated scene meshes (e.g. a per-brick layout script rebuilding
    // box geometry every Auto Plan / attach cycle) mint a fresh ContentHash whenever the mesh's
    // own vertex data changes — unlike CollisionCheckerSessionCache, this cache previously had NO
    // cap, no eviction, and stored strong references, so it grew forever for the life of the
    // process. BvhNode is plain managed data (no native handle), so LRU eviction here is fully
    // safe: any RobotMeshCollisionChecker that already copied a BvhNode reference into its own
    // private cache keeps working after eviction; the shared cache just rebuilds on next miss.
    private const int MaxEntries = 256;
    private static readonly Dictionary<int, (CollisionObject Source, BvhNode Node)> Cache = new();
    private static readonly LinkedList<int> Order = new();
    private static readonly Dictionary<int, LinkedListNode<int>> OrderNodes = new();
    private static readonly object Gate = new();

    public static BvhNode GetOrBuild(CollisionObject meshObj)
    {
        if (meshObj.Shape != CollisionShape.Mesh ||
            meshObj.MeshVertices is null || meshObj.MeshIndices is null)
            throw new ArgumentException("CollisionMeshCache requires a mesh CollisionObject.", nameof(meshObj));

        var key = GeometryFingerprint(meshObj);
        lock (Gate)
        {
            if (Cache.TryGetValue(key, out var hit) && SameMesh(hit.Source, meshObj))
            {
                Touch(key);
                return hit.Node;
            }
            var node = BvhBuilder.Build(meshObj);
            Insert(key, meshObj, node);
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
            if (!Cache.TryGetValue(key, out var cached) || !SameMesh(cached.Source, meshObj))
                return false;
            Touch(key);
            node = cached.Node;
            return true;
        }
    }

    public static void Clear() { lock (Gate) { Cache.Clear(); Order.Clear(); OrderNodes.Clear(); } }

    private static void Touch(int key)
    {
        if (!OrderNodes.TryGetValue(key, out var node)) return;
        Order.Remove(node);
        Order.AddLast(node);
    }

    // A colliding key replaces the older mesh's entry (one slot per key).
    private static void Insert(int key, CollisionObject source, BvhNode node)
    {
        if (OrderNodes.Remove(key, out var old)) Order.Remove(old);
        Cache[key] = (source, node);
        OrderNodes[key] = Order.AddLast(key);
        while (Order.Count > MaxEntries)
        {
            var oldest = Order.First!;
            Order.RemoveFirst();
            OrderNodes.Remove(oldest.Value);
            Cache.Remove(oldest.Value);
        }
    }

    // The 32-bit ContentHash is only the slot: a hit must have the same vertices and indices (the BVH depends on nothing else).
    private static bool SameMesh(CollisionObject a, CollisionObject b)
    {
        if (ReferenceEquals(a, b)) return true;
        if (a.MeshIndices!.Count != b.MeshIndices!.Count || a.MeshVertices!.Count != b.MeshVertices!.Count) return false;
        for (var i = 0; i < a.MeshIndices.Count; i++)
            if (a.MeshIndices[i] != b.MeshIndices[i]) return false;
        for (var i = 0; i < a.MeshVertices.Count; i++)
            if (!a.MeshVertices[i].AsSpan().SequenceEqual(b.MeshVertices[i])) return false;
        return true;
    }

    /// <summary>Uses <see cref="CollisionObject.ContentHash"/> computed once at construction.</summary>
    internal static int GeometryFingerprint(CollisionObject obj) => obj.ContentHash;
}
