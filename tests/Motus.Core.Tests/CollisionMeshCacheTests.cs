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
}
