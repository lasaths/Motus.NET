using Motus.Core;
using Motus.Viewer.Services;

namespace Motus.Core.Tests;

/// <summary>
/// Store→truss member Motus paths for /bamboo-truss. Exercises the same viewer services the page runs.
/// </summary>
public class BambooTrussTests
{
    private static readonly Lazy<BambooIcdCell> LazyCell = new(() => BambooIcdCell.LoadFile(UrdfPath));

    private static string UrdfPath => Path.GetFullPath(Path.Combine(
        AppContext.BaseDirectory, "..", "..", "..", "..", "..", "tests", "fixtures", "bamboo_icd", "bamboo_icd.urdf"));

    private static BambooIcdCell Cell => LazyCell.Value;

    private static BambooTrussMotion.TrussBuild Build(List<BambooIcdMotion.TaskEdit>? tasks = null)
    {
        var built = BambooTrussMotion.Build(tasks);
        Assert.True(string.IsNullOrEmpty(built.Error), $"truss motion should build: {built.Error}");
        Assert.NotEmpty(built.Keys);
        Assert.NotEmpty(built.Cycles);
        return built;
    }

    [Fact]
    public void Layout_IsTwoTrianglesWithSharedMiddleEdge()
    {
        Assert.Equal(5, BambooTrussLayout.Members.Count);
        Assert.Equal(4, BambooTrussLayout.FootNodes.Length);
        var ids = BambooTrussLayout.Members.Select(m => m.Id).ToArray();
        Assert.Equal(new[] { "baseL", "baseR", "sep", "diagL", "diagR" }, ids);
        // Shared upright is B–C; bases share B; diags meet at C.
        var sep = BambooTrussLayout.Members.First(m => m.Id == "sep");
        Assert.Equal(0, sep.EdgeA.X, 9);
        Assert.Equal(BambooTrussLayout.FootZ0, sep.EdgeA.Z, 9);
        Assert.Equal(0, sep.EdgeB.X, 9);
        Assert.Equal(BambooTrussLayout.FootZ0 + BambooTrussLayout.FootH, sep.EdgeB.Z, 9);
    }

    [Fact]
    public void DefaultTasks_AreStoreToMemberPickPlacePairs()
    {
        var tasks = BambooTrussMotion.DefaultTasks();
        Assert.Equal(10, tasks.Count);
        var pairs = BambooTrussMotion.Pairs(tasks);
        Assert.Equal(5, pairs.Count);
        Assert.All(pairs, p =>
        {
            Assert.Equal("pick", p.Pick.Identity);
            Assert.Equal("place", p.Place.Identity);
            Assert.Equal(p.MemberId, p.Pick.Object);
            Assert.Equal(p.MemberId, p.Place.Object);
            Assert.Contains(p.Pick.Planes, x => x.Name == "grasp");
            Assert.Contains(p.Place.Planes, x => x.Name == "release");
        });
    }

    [Fact]
    public void StoreToFirstMember_PathIsCollisionFree_UnderMotus()
    {
        // Only the first store→baseL cycle — a clear path must validate.
        var tasks = BambooTrussMotion.DefaultTasks().Take(2).ToList();
        var built = Build(tasks);
        Assert.Single(built.Cycles);
        Assert.Equal("baseL", built.Cycles[0].MemberId);

        var scan = new BambooTrussScan(Cell, built.Keys, built.Cycles, BambooTrussLayout.Members, maxStepDeg: 0.5).RunToEnd();
        Assert.Null(scan.TrackMiss);
        Assert.True(scan.Complete);
        Assert.True(scan.FirstHit is null,
            $"store→baseL should be clear, Motus hit: {scan.FirstHit?.Describe()} at {scan.FirstHit?.T:F3} ({scan.FirstHit?.Label})");
        Assert.Equal(1.0, scan.LastClearT, 5);
    }

    [Fact]
    public void StoreToFirstMember_CollidingGraspNudge_FailsWithNamedContact()
    {
        var tasks = BambooTrussMotion.DefaultTasks().Take(2).ToList();
        // Aim the grasp into the strut / neighbours — Motus must stop and name the hit.
        tasks[0].Planes.First(p => p.Name == "grasp").YMm = -1.5;
        var built = BambooTrussMotion.Build(tasks);
        // Motion may still build (floor heuristics are preferences); collision is Motus-only.
        Assert.True(string.IsNullOrEmpty(built.Error), built.Error);

        var scan = new BambooTrussScan(Cell, built.Keys, built.Cycles, BambooTrussLayout.Members, maxStepDeg: 0.5).RunToEnd();
        var hit = Assert.IsType<BambooTrussScan.Hit>(scan.FirstHit);
        Assert.True(hit.Contacts.Count > 0, "expected Motus contacts");
        Assert.True(hit.LastClearT < hit.T);
        Assert.Contains(hit.Contacts, c =>
            c.BodyA.Contains("strut", StringComparison.OrdinalIgnoreCase) ||
            c.BodyB.Contains("strut", StringComparison.OrdinalIgnoreCase) ||
            c.BodyA.Contains("ground", StringComparison.OrdinalIgnoreCase) ||
            c.BodyB.Contains("ground", StringComparison.OrdinalIgnoreCase) ||
            c.BodyA.Contains("store-", StringComparison.OrdinalIgnoreCase) ||
            c.BodyB.Contains("store-", StringComparison.OrdinalIgnoreCase) ||
            c.BodyA.Contains("neck", StringComparison.OrdinalIgnoreCase) ||
            c.BodyB.Contains("neck", StringComparison.OrdinalIgnoreCase));
    }

    [Fact]
    public void FullTrussMotion_BuildsFiveCycles()
    {
        var built = Build();
        Assert.Equal(5, built.Cycles.Count);
        Assert.Equal(BambooTrussLayout.Members.Select(m => m.Id), built.Cycles.Select(c => c.MemberId));
        Assert.Equal(0, built.Cycles[0].T0, 9);
        Assert.Equal(1, built.Cycles[^1].T1, 9);
    }

    [Fact]
    public void IkSolveSpatial_MatchesPlanarWhenXIsZero()
    {
        var planar = BambooIcdMotion.IkSolve(0.00778, 0.07, 0);
        var spatial = BambooIcdMotion.IkSolveSpatial(0, 0.00778, 0.07, 0);
        Assert.NotEmpty(planar);
        Assert.NotEmpty(spatial);
        Assert.Equal(planar[0][0], spatial[0][0], 6);
        Assert.Equal(planar[0][1], spatial[0][1], 6);
    }
}
