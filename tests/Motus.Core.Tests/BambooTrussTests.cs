using Motus.Core;
using Motus.Viewer.Services;

namespace Motus.Core.Tests;

/// <summary>
/// Store→truss member Motus paths for /bamboo-truss. Exercises the same viewer services the page runs.
/// </summary>
public class BambooTrussTests
{
    // The truss page loads the cell with the truss cane length — collision uses exactly that mesh.
    private static readonly Lazy<BambooIcdCell> LazyCell = new(() => BambooIcdCell.LoadFile(UrdfPath, BambooTrussLayout.StrutLength));

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
    public void Layout_IsTwoTrianglesWithSharedMiddleEdge_AndHalfMetreCanes()
    {
        Assert.Equal(0.5, BambooTrussLayout.StrutLength, 9);
        Assert.Equal(5, BambooTrussLayout.Members.Count);
        Assert.Equal(4, BambooTrussLayout.FootNodes.Length);
        var ids = BambooTrussLayout.Members.Select(m => m.Id).ToArray();
        Assert.Equal(new[] { "baseL", "baseR", "sep", "diagL", "diagR" }, ids);
        Assert.All(BambooTrussLayout.Members, m => Assert.Equal(-90, m.Place.PitchDeg, 9));
        Assert.True(BambooTrussLayout.HangForMemberIndex(0).IsPole);
        Assert.True(BambooTrussLayout.HangForMemberIndex(1).IsPole);
        var walk = BambooTrussLayout.WalkHang;
        Assert.False(walk.IsPole);
        Assert.Equal(BambooTrussLayout.Members[0].Place.X + BambooTrussLayout.WalkHangOffsetX, walk.X, 9);
        Assert.Equal(BambooTrussLayout.Members[0].Place.Z + BambooTrussLayout.WalkHangOffsetZ, walk.Z, 9);
        Assert.Equal(BambooTrussLayout.WalkHangY, walk.Y, 9);
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
        var tasks = BambooTrussMotion.DefaultTasks().Take(2).ToList();
        var built = Build(tasks);
        Assert.Single(built.Cycles);
        Assert.Equal("baseL", built.Cycles[0].MemberId);
        Assert.True(built.Cycles[0].Hang.IsPole);

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
        tasks[0].Planes.First(p => p.Name == "grasp").YMm = -1.5;
        var built = BambooTrussMotion.Build(tasks);
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
    public void FullTruss_PathIsCollisionFree_UnderMotus()
    {
        var built = Build();
        // Five members + brachiate walk on both-ends-fixed baseL.
        Assert.Equal(6, built.Cycles.Count);
        Assert.Equal("brachiate", built.Cycles[1].MemberId);
        Assert.False(built.Cycles[1].PlaceHang.IsPole);
        Assert.Equal(BambooTrussLayout.WalkHang.X, built.Cycles[1].PlaceHang.X, 9);
        Assert.Equal(BambooTrussLayout.WalkHang.Z, built.Cycles[1].PlaceHang.Z, 9);
        Assert.Contains(built.Keys, k => k.Label.StartsWith("brachiate:", StringComparison.Ordinal));

        // Fine Motus scan (≤0.5°/joint) is the authority — planner geometric clearance is not Motus.
        var scan = new BambooTrussScan(Cell, built.Keys, built.Cycles, BambooTrussLayout.Members, maxStepDeg: 0.5).RunToEnd();
        Assert.Null(scan.TrackMiss);
        Assert.True(scan.Complete);
        Assert.True(scan.SampleCount > 2000,
            $"fine Motus scan expected thousands of samples, got {scan.SampleCount} (coarse planner alone must not define clear)");
        Assert.True(scan.FirstHit is null,
            $"full store→truss should be clear, Motus hit: {scan.FirstHit?.Describe()} at {scan.FirstHit?.T:F3} ({scan.FirstHit?.Label})");
        Assert.Equal(1.0, scan.LastClearT, 5);
    }

    [Fact]
    public void FullTruss_EveryCaneIsReleasedExactlyOnItsMember()
    {
        var built = Build();
        var scan = new BambooTrussScan(Cell, built.Keys, built.Cycles, BambooTrussLayout.Members, maxStepDeg: 0.5);
        Assert.Null(scan.TrackMiss);
        foreach (var c in built.Cycles)
        {
            if (c.MemberId == "brachiate") continue;
            var member = BambooTrussLayout.Members.First(m => m.Id == c.MemberId);
            var t = c.T1 - 1e-7;
            var s = BambooIcdMotion.Sample(built.Keys, t);
            Assert.Equal(2, s.Hold);
            var active = scan.StrutPoses(t, s.Hold, s.Q).Single(p => p.Name == $"placed-{member.Id}");
            var want = BambooIcdCell.ToViewerPose(BambooIcdCell.PlacementWorld(member.Place));
            for (var k = 0; k < 3; k++)
                Assert.True(Math.Abs(active.Pose[k] - want[k]) < 1e-3, $"{member.Id}: released position off by {Math.Abs(active.Pose[k] - want[k]) * 1000:F2} mm (axis {k})");
            var a = AxisOf(active.Pose); var b = AxisOf(want);
            Assert.True(Math.Abs(a.X * b.X + a.Y * b.Y + a.Z * b.Z) > Math.Cos(2 * Math.PI / 180), $"{member.Id}: released cane axis differs");
            // Vertical upright.
            Assert.True(Math.Abs(a.Y) > 0.98, $"{member.Id}: cane should stand vertical");
        }
    }

    [Fact]
    public void FullTruss_PlacedMembersAreLiveMotusObstacles()
    {
        var members = BambooTrussLayout.Members.ToList();
        var last = members[^1];
        members[^1] = last with { Place = members[1].Place };
        var built = Build(BambooTrussMotion.DefaultTasks(members));
        var scan = new BambooTrussScan(Cell, built.Keys, built.Cycles, members, maxStepDeg: 0.5).RunToEnd();
        var hit = Assert.IsType<BambooTrussScan.Hit>(scan.FirstHit);
        Assert.Equal(last.Id, BambooTrussMotion.MemberIdAt(built.Cycles, hit.T));
        Assert.True(hit.T > built.Cycles[^1].T0);
        Assert.Contains(hit.Contacts, c =>
            c.BodyA.StartsWith("placed-", StringComparison.Ordinal) || c.BodyB.StartsWith("placed-", StringComparison.Ordinal));
    }

    [Fact]
    public void Layout_UprightsAreReachable_AndNeverTouch()
    {
        var all = BambooTrussLayout.Members.Select(m => (Id: m.Id, P: m.Place)).Append((Id: "store", P: BambooTrussLayout.Store)).ToList();
        for (var i = 0; i < BambooTrussLayout.Members.Count; i++)
        {
            var m = BambooTrussLayout.Members[i];
            Assert.NotEmpty(BambooIcdMotion.IkSolveAxis(m.Place.X, m.Place.Y, m.Place.Z, m.Place.PitchDeg, m.Place.Axis));
            Assert.True(m.Place.Y - BambooTrussLayout.StrutLength / 2 > -0.002, $"{m.Id} bottom below ground");
        }
        Assert.NotEmpty(BambooIcdMotion.IkSolveAxis(
            BambooTrussLayout.Store.X, BambooTrussLayout.Store.Y, BambooTrussLayout.Store.Z, 0, BambooTrussLayout.Store.Axis));

        for (var i = 0; i < all.Count; i++)
        for (var j = i + 1; j < all.Count; j++)
        {
            var d = SegmentDistance(all[i].P, all[j].P, BambooTrussLayout.StrutLength);
            Assert.True(d > 2 * BambooIcdCell.StrutRadius + 0.004, $"{all[i].Id}–{all[j].Id} canes too close: {d * 1000:F1} mm");
        }
    }

    [Fact]
    public void FullTrussMotion_BuildsFiveCycles()
    {
        var built = Build();
        Assert.Equal(6, built.Cycles.Count);
        Assert.Equal(
            new[] { "baseL", "brachiate", "baseR", "sep", "diagL", "diagR" },
            built.Cycles.Select(c => c.MemberId));
        Assert.Equal(0, built.Cycles[0].T0, 9);
        Assert.Equal(1, built.Cycles[^1].T1, 9);
        Assert.False(built.Cycles[1].PlaceHang.IsPole);
    }

    [Fact]
    public void IkSolveAxis_MatchesSpatialForThePlanarStrut()
    {
        var spatial = BambooIcdMotion.IkSolveSpatial(0, 0.00778, 0.07, 0);
        var axis = BambooIcdMotion.IkSolveAxis(0, 0.00778, 0.07, 0, BambooIcdMotion.StrutAxis(0));
        Assert.NotEmpty(axis);
        Assert.Contains(axis, q => Enumerable.Range(0, 5).All(k => Math.Abs(q[k] - spatial[0][k]) < 1e-6));
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

    private static (double X, double Y, double Z) AxisOf(double[] pose7)
    {
        double x = pose7[3], y = pose7[4], z = pose7[5], w = pose7[6];
        return (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y));
    }

    private static double SegmentDistance(BambooIcdCell.StrutPlacement a, BambooIcdCell.StrutPlacement b, double len)
    {
        var ua = a.Axis; var ub = b.Axis;
        var best = double.MaxValue;
        for (var i = 0; i <= 200; i++)
        for (var j = 0; j <= 200; j++)
        {
            var sa = (i / 200.0 - 0.5) * len; var sb = (j / 200.0 - 0.5) * len;
            var dx = a.X + ua.X * sa - (b.X + ub.X * sb);
            var dy = a.Y + ua.Y * sa - (b.Y + ub.Y * sb);
            var dz = a.Z + ua.Z * sa - (b.Z + ub.Z * sb);
            best = Math.Min(best, Math.Sqrt(dx * dx + dy * dy + dz * dz));
        }
        return best;
    }





}
