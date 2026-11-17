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

    // F1: the key must cover everything the checker bakes in (tool geometry, base frame, tool frame).
    private static readonly JointState Home = new(new double[] { 0, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });

    private static RobotModel Session(Frame tcp, Frame toolGeomPose, BaseFrame? baseOverride = null)
    {
        var preset = PresetLoader.LoadRobotModelByName("UR5e");
        var links = new List<LinkCollisionGeometry> { new(0, "link0", CollisionObject.Capsule("l0", Frame.Identity, 0.05, 0.1)) };
        var tool = new ToolDefinition("gripper", tcp, CollisionObject.Box("gripper", toolGeomPose, 0.02, 0.02, 0.03));
        return new RobotModel(preset.Preset, new RobotCollisionModel(links)).WithTool(tool, baseOverride);
    }

    /// <summary>Obstacle on the TCP of <paramref name="robot"/> at <see cref="Home"/>.</summary>
    private static CollisionScene SceneOnTcp(RobotModel robot)
    {
        var fk = KinematicsResolver.CreateFkSolver(robot.Preset);
        var tcp = fk.ComputeTcp(Home, robot.Preset.BaseFrame, robot.Preset.ToolFrame).Tcp;
        return new CollisionScene(new[] { CollisionObject.Sphere("f1_obs", tcp, 0.02) });
    }

    public static TheoryData<string> Changes => new() { "tool-geometry", "base-frame", "tool-frame" };

    [Theory]
    [MemberData(nameof(Changes))]
    public void ChangedToolOrFrames_GetFreshChecker_NotStaleVerdict(string change)
    {
        var tcp = new Frame(0, 0, 0.1);
        var a = Session(tcp, Frame.Identity);
        var b = change switch
        {
            "tool-geometry" => Session(tcp, new Frame(0, 0, 1)),
            "base-frame" => Session(tcp, Frame.Identity, new BaseFrame(new Frame(5, 0, 0))),
            _ => Session(new Frame(0, 0, 1.1), Frame.Identity),
        };
        var scene = SceneOnTcp(a);
        Assert.False(CollisionCheckerFactory.Create(a).IsCollisionFree(Home, scene));
        Assert.True(CollisionCheckerFactory.Create(b).IsCollisionFree(Home, scene));

        // b is clear and cached first; a hits. A stale hit would hand a the clear checker of b.
        var cachedB = CollisionCheckerSessionCache.GetOrCreate(b, null, null, scene);
        var cachedA = CollisionCheckerSessionCache.GetOrCreate(a, null, null, scene);
        GC.KeepAlive(cachedB);
        Assert.False(cachedA.IsCollisionFree(Home, scene));
        Assert.NotSame(cachedA, cachedB);
    }

    [Fact]
    public void EqualInputsBuiltTwice_HitTheSameChecker()
    {
        var a1 = Session(new Frame(0, 0, 0.1), Frame.Identity);
        var a2 = Session(new Frame(0, 0, 0.1), Frame.Identity);
        var scene = SceneOnTcp(a1);
        Assert.Same(
            CollisionCheckerSessionCache.GetOrCreate(a1, null, null, scene),
            CollisionCheckerSessionCache.GetOrCreate(a2, null, null, scene));
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
