using Motus.Construction;

namespace Motus.Core.Tests;

public partial class BambooTrussTests
{
    /// <summary>
    /// Transit cache: replanning T1M from the same state reuses every RRT path (exact re-check, no RRT) and gives the
    /// same motion; a cached path that is not clear in the current scene is refused and RRT runs again.
    /// </summary>
    [Fact]
    public void Planner_TransitCache_HitSkipsRrt_BlockedHitFallsBackToRrt()
    {
        BambooTrussMotion.UseCell(Cell);
        var p = new BambooTrussPlanner(Cell) { TransitCache = new() };
        Assert.True(p.Start(), p.Error);
        Assert.True(p.BuildMember(BambooTrussLayout.Def("AT1")!), p.Error);
        string Keys() => System.Text.Json.JsonSerializer.Serialize(p.Keys);

        var cp = p.SaveDeep();
        var rrt0 = p.RrtRuns;
        Assert.True(p.BuildMember(BambooTrussLayout.Def("T1M")!), p.Error);
        Assert.True(p.RrtRuns > rrt0, "T1M needs RRT");
        Assert.NotEmpty(p.TransitCache!);
        var first = Keys();

        p.RollbackDeep(cp);
        var (rrt1, hits1, checks1) = (p.RrtRuns, p.TransitCacheHits, p.Checks);
        Assert.True(p.BuildMember(BambooTrussLayout.Def("T1M")!), p.Error);
        _out.WriteLine($"first: {rrt1 - rrt0} RRT runs, {p.TransitCache!.Count} cached; second: {p.RrtRuns - rrt1} RRT runs, {p.TransitCacheHits - hits1} hits; same keys {first == Keys()}");
        Assert.Equal(rrt1, p.RrtRuns);
        Assert.True(p.TransitCacheHits > hits1);
        Assert.True(p.Checks > checks1);
        Assert.Equal(first, Keys());

        // Blocked: the direct segment already failed here, so a cached straight move is not clear in this scene.
        p.RollbackDeep(cp);
        foreach (var k in p.TransitCache!.Keys.ToList()) p.TransitCache[k] = (new List<double[]>(), p.TransitCache[k].Rng);
        var (rrt2, hits2) = (p.RrtRuns, p.TransitCacheHits);
        p.BuildMember(BambooTrussLayout.Def("T1M")!);
        Assert.Equal(hits2, p.TransitCacheHits);
        Assert.True(p.RrtRuns > rrt2);
    }
}

public partial class BambooTrussTests
{
    /// <summary>
    /// A path cached in a rolled-back branch is not reused in another state: with a cache filled by building AM first
    /// (then rolled back), building T1M then AM gives exactly the motion of a fresh planner without a cache, so whatever
    /// the build search confirms with the cache, a replay without it rebuilds.
    /// </summary>
    [Fact]
    public void Planner_TransitCache_RolledBackBranch_ReplaysLikeAFreshPlanner()
    {
        BambooTrussMotion.UseCell(Cell);
        string Run(BambooTrussPlanner p, bool pollute)
        {
            Assert.True(p.Start(), p.Error);
            Assert.True(p.BuildMember(BambooTrussLayout.Def("AT1")!), p.Error);
            if (pollute)
            {
                var cp = p.SaveDeep();
                Assert.True(p.BuildMember(BambooTrussLayout.Def("AM")!), p.Error);
                p.RollbackDeep(cp);
            }
            var okT1M = p.BuildMember(BambooTrussLayout.Def("T1M")!);
            var okAm = okT1M && p.BuildMember(BambooTrussLayout.Def("AM")!);
            return $"{okT1M},{okAm},{p.Error}|" + System.Text.Json.JsonSerializer.Serialize(p.Keys);
        }
        var cached = new BambooTrussPlanner(Cell) { TransitCache = new() };
        var withCache = Run(cached, pollute: true);
        var fresh = Run(new BambooTrussPlanner(Cell), pollute: false);
        _out.WriteLine($"cache: {cached.TransitCache!.Count} entries, {cached.TransitCacheHits} hits; {withCache[..Math.Min(80, withCache.Length)]}");
        Assert.NotEmpty(cached.TransitCache!);
        Assert.Equal(0, cached.TransitCacheHits); // the AM paths were planned in another scene and random-stream position
        Assert.Equal(fresh, withCache);
    }
}
