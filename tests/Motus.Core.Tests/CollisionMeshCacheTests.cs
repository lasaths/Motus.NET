using Motus.Core;
using Motus.Geometry;

namespace Motus.Core.Tests;

/// <summary>
/// Procedurally generated scene meshes (e.g. a per-brick layout script rebuilding box geometry
/// every Auto Plan / attach cycle in a pick-place scene) each mint a distinct ContentHash.
/// CollisionMeshCache is a static, process-lifetime cache with strong references (no
/// WeakReference), so unlike CollisionCheckerSessionCache it must bound itself directly rather
/// than relying on GC visibility. Eviction must be transparent: any consumer that already holds
/// a BvhNode reference keeps working, and a subsequent GetOrBuild for an evicted shape simply
/// rebuilds rather than throwing.
/// </summary>
public class CollisionMeshCacheTests
{
    private static CollisionObject Triangle(int i)
    {
        var verts = new List<double[]>
        {
            new[] { 0.0 + i * 0.001, 0.0, 0.0 },
            new[] { 1.0 + i * 0.001, 0.0, 0.0 },
            new[] { 0.0 + i * 0.001, 1.0, 0.0 },
        };
        var indices = new List<int> { 0, 1, 2 };
        return CollisionObject.Mesh($"tri{i:D4}", new Frame(0, 0, 0), verts, indices);
    }

    [Fact]
    public void ManyDistinctMeshShapes_DoNotGrowCacheUnbounded()
    {
        CollisionMeshCache.Clear();

        // Far more than the cache's internal cap — simulates a long session where a layout
        // script regenerates slightly different box/mesh geometry on every replan.
        for (var i = 0; i < 500; i++)
            CollisionMeshCache.GetOrBuild(Triangle(i));

        // An early (now evicted) shape must still be buildable, not throw.
        var rebuilt = CollisionMeshCache.GetOrBuild(Triangle(0));
        Assert.NotNull(rebuilt);

        // A very recently used shape should still be a cache hit (present via TryGet).
        Assert.True(CollisionMeshCache.TryGet(Triangle(499), out var recent));
        Assert.NotNull(recent);
    }

    [Fact]
    public void PreviouslyFetchedNode_RemainsUsableAfterEviction()
    {
        CollisionMeshCache.Clear();

        var first = CollisionMeshCache.GetOrBuild(Triangle(0));

        for (var i = 1; i < 500; i++)
            CollisionMeshCache.GetOrBuild(Triangle(i));

        // Eviction from the shared cache must never invalidate a BvhNode a caller already holds.
        Assert.NotNull(first);
        Assert.False(CollisionMeshCache.TryGet(Triangle(0), out _));
    }

    // Same name, different place: only the vertices differ, so ContentHash (32 bits) collides by chance.
    private static CollisionObject Shifted(int i) => CollisionObject.Mesh("col", Frame.Identity,
        new List<double[]> { new[] { i * 1.0, 0, 0 }, new[] { i * 1.0 + 1, 0, 0 }, new[] { i * 1.0, 1, 0 } }, new List<int> { 0, 1, 2 });

    [Fact]
    public void HashCollision_DoesNotReturnTheOtherMeshBvh()
    {
        // Birthday search for two different meshes with the same 32-bit ContentHash (HashCode is seeded per process).
        var seen = new Dictionary<int, int>();
        int a = -1, b = -1;
        for (var i = 0; i < 2_000_000 && a < 0; i++)
        {
            var h = Shifted(i).ContentHash;
            if (seen.TryGetValue(h, out var j)) { a = j; b = i; }
            else seen[h] = i;
        }
        Assert.True(a >= 0, "no 32-bit collision found");
        CollisionMeshCache.Clear();
        CollisionMeshCache.GetOrBuild(Shifted(a));
        var got = CollisionMeshCache.GetOrBuild(Shifted(b));
        Assert.Equal(b * 1.0, got.Min[0]); // a stale hit has the BVH box of mesh a
        Assert.True(CollisionMeshCache.TryGet(Shifted(b), out var again));
        Assert.Equal(b * 1.0, again!.Min[0]);
        Assert.True(CollisionMeshCache.TryGet(Shifted(a), out var other) == false || other!.Min[0] == a * 1.0);
    }
}
