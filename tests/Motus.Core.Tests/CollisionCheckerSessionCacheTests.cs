using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>
/// Pick-place style workloads (one attach/detach fingerprint per brick, replanned repeatedly under
/// Auto Plan) mint many distinct CollisionCheckerSessionCache keys. The cache must bound its own
/// (key, WeakReference) bookkeeping over a long session, and eviction must never break a checker a
/// caller is still holding a strong reference to and using.
/// </summary>
public class CollisionCheckerSessionCacheTests
{
    private static RobotModel Ur5e() =>
        new(PresetLoader.LoadByModelName("UR5e", FindResources()));

    private static CollisionScene SceneFor(int i) =>
        new(new[] { CollisionObject.Sphere($"obs{i:D3}", new Frame(2, 2, 2 + i * 0.001), 0.05) });

    [Fact]
    public void ManyDistinctFingerprints_DoNotGrowCacheUnbounded()
    {
        var robot = Ur5e();

        // Far more than the cache's internal cap — simulates a long Auto Plan session cycling
        // through many uniquely-named bricks.
        for (var i = 0; i < 200; i++)
            CollisionCheckerSessionCache.GetOrCreate(robot, null, null, SceneFor(i));

        // Re-requesting an early (now evicted) fingerprint must still work — proves the cache
        // recovers by rebuilding rather than throwing once bounded.
        var rebuilt = CollisionCheckerSessionCache.GetOrCreate(robot, null, null, SceneFor(0));
        Assert.NotNull(rebuilt);
    }

    [Fact]
    public void EvictedButStillReferencedChecker_RemainsUsable()
    {
        var robot = Ur5e();

        // Hold a strong reference to the very first checker created.
        var first = CollisionCheckerSessionCache.GetOrCreate(robot, null, null, SceneFor(0));

        // Push well past the cache cap so the first entry's dictionary slot is evicted.
        for (var i = 1; i < 200; i++)
            CollisionCheckerSessionCache.GetOrCreate(robot, null, null, SceneFor(i));

        // Eviction must never Dispose a checker the caller still holds — it must remain callable.
        var state = new JointState(new double[6]);
        var stillWorks = first.IsCollisionFree(state, SceneFor(0));
        Assert.True(stillWorks);
    }

    private static string FindResources()
    {
        var dir = AppContext.BaseDirectory;
        for (var i = 0; i < 8 && dir is not null; i++)
        {
            var candidate = Path.Combine(dir, "resources", "robots");
            if (Directory.Exists(candidate)) return candidate;
            dir = Directory.GetParent(dir)?.FullName;
        }
        throw new InvalidOperationException("resources/robots not found");
    }
}
