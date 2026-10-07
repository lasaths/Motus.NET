using Motus.Core;
using Motus.Viewer.Services;

namespace Motus.Core.Tests;

/// <summary>
/// Store→truss member Motus paths for /bamboo-truss. Exercises the same viewer services the page runs.
/// </summary>
public class BambooTrussTests
{
    // The truss page loads the cell with the truss cane length — collision uses exactly that mesh.
    private static readonly Lazy<BambooIcdCell> LazyCell = new(() =>
    {
        var cell = BambooIcdCell.LoadFile(UrdfPath, BambooTrussLayout.StrutLength);
        cell.SetAnchorPads(BambooTrussLayout.AnchorPads
            .Select(a => (a.X, a.Z, BambooTrussLayout.AnchorPadHalf)).ToList());
        return cell;
    });

    private static string UrdfPath => Path.GetFullPath(Path.Combine(
        AppContext.BaseDirectory, "..", "..", "..", "..", "..", "tests", "fixtures", "bamboo_icd", "bamboo_icd.urdf"));

    private static BambooIcdCell Cell => LazyCell.Value;

    private static BambooTrussMotion.TrussBuild Build(List<BambooIcdMotion.TaskEdit>? tasks = null)
    {
        // Same binding the Blazor page uses so planner Motus == scan Motus.
        BambooTrussMotion.UseCell(Cell);
        var built = BambooTrussMotion.Build(tasks);
        Assert.True(string.IsNullOrEmpty(built.Error), $"truss motion should build: {built.Error}");
        Assert.NotEmpty(built.Keys);
        Assert.NotEmpty(built.Cycles);
        return built;
    }

    [Fact]
    public void Layout_IsBridgeSpanTwoTriangles_InVerticalPlane()
    {
        Assert.Equal(5, BambooTrussLayout.Members.Count);
        Assert.Equal(4, BambooTrussLayout.FootNodes.Length);
        var ids = BambooTrussLayout.Members.Select(m => m.Id).ToArray();
        Assert.Equal(new[] { "baseL", "baseR", "sep", "diagL", "diagR" }, ids);
        Assert.Equal(new[] { "/", "_", "\\", "/", "\\" }, BambooTrussLayout.Members.Select(m => m.Glyph).ToArray());
        Assert.Equal(2, BambooTrussLayout.AnchorPads.Length);
        // Shared upright B–C is vertical in the XY plane at PlaneZ.
        var sep = BambooTrussLayout.Members.First(m => m.Id == "sep");
        Assert.Equal(0, sep.EdgeA.X, 9);
        Assert.Equal(BambooTrussLayout.FootY0, sep.EdgeA.Y, 9);
        Assert.Equal(BambooTrussLayout.PlaneZ, sep.EdgeA.Z, 9);
        Assert.Equal(0, sep.EdgeB.X, 9);
        Assert.Equal(BambooTrussLayout.FootY0 + BambooTrussLayout.Height, sep.EdgeB.Y, 9);
        Assert.Equal(BambooTrussLayout.PlaneZ, sep.EdgeB.Z, 9);
        // All nodes share the vertical plane.
        Assert.All(BambooTrussLayout.FootNodes, n => Assert.Equal(BambooTrussLayout.PlaneZ, n.Z, 9));
        // Sep is upright (pitch −90).
        Assert.Equal(-90, sep.Place.PitchDeg, 5);
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
        BambooTrussMotion.UseCell(Cell);
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
        Assert.Equal(5, built.Cycles.Count);

        var scan = new BambooTrussScan(Cell, built.Keys, built.Cycles, BambooTrussLayout.Members, maxStepDeg: 0.5).RunToEnd();
        Assert.Null(scan.TrackMiss);
        Assert.True(scan.Complete);
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
        for (var i = 0; i < built.Cycles.Count; i++)
        {
            var c = built.Cycles[i];
            var member = BambooTrussLayout.Members[i];
            var t = c.T1 - 1e-7;
            var s = BambooIcdMotion.Sample(built.Keys, t);
            Assert.Equal(2, s.Hold);
            var active = scan.StrutPoses(t, s.Hold, s.Q).Single(p => p.Name == $"placed-{member.Id}");
            var want = BambooIcdCell.ToViewerPose(BambooIcdCell.PlacementWorld(member.Place));
            for (var k = 0; k < 3; k++)
                Assert.True(Math.Abs(active.Pose[k] - want[k]) < 1e-3, $"{member.Id}: released position off by {Math.Abs(active.Pose[k] - want[k]) * 1000:F2} mm (axis {k})");
            var a = AxisOf(active.Pose); var b = AxisOf(want);
            Assert.True(Math.Abs(a.X * b.X + a.Y * b.Y + a.Z * b.Z) > Math.Cos(2 * Math.PI / 180), $"{member.Id}: released cane axis differs");
            // Vertical-plane truss: canes are not ground-horizontal; they follow triangle edges.
            Assert.True(active.Pose[1] > BambooIcdCell.StrutRadius - 1e-3, $"{member.Id}: cane below ground");
        }
    }

    [Fact]
    public void FullTruss_PlacedMembersAreLiveMotusObstacles()
    {
        var members = BambooTrussLayout.Members.ToList();
        var last = members[^1];
        members[^1] = last with { Place = members[1].Place };
        var built = Build(BambooTrussMotion.DefaultTasks(members)); // UseCell via Build()
        var scan = new BambooTrussScan(Cell, built.Keys, built.Cycles, members, maxStepDeg: 0.5).RunToEnd();
        var hit = Assert.IsType<BambooTrussScan.Hit>(scan.FirstHit);
        Assert.Equal(last.Id, BambooTrussMotion.MemberIdAt(built.Cycles, hit.T));
        Assert.True(hit.T > built.Cycles[^1].T0);
        Assert.Contains(hit.Contacts, c =>
            c.BodyA.StartsWith("placed-", StringComparison.Ordinal) || c.BodyB.StartsWith("placed-", StringComparison.Ordinal));
    }

    [Fact]
    public void Layout_EdgeCanesAreMotusReachable_AndNeverTouch()
    {
        var all = BambooTrussLayout.Members.Select(m => (Id: m.Id, P: m.Place)).Append((Id: "store", P: BambooTrussLayout.Store)).ToList();
        foreach (var (id, p) in all)
        {
            // Store is horizontal pitch 0; truss edges use multi-tilt IK.
            if (id == "store")
            {
                Assert.Equal(0, p.PitchDeg, 9);
                Assert.NotEmpty(BambooIcdMotion.IkSolveAxis(p.X, p.Y, p.Z, 0, p.Axis));
            }
            else
            {
                var sols = new List<double[]>();
                foreach (var tilt in new[] { p.PitchDeg, -Math.Abs(p.PitchDeg), Math.Abs(p.PitchDeg), 0.0, 20.0, -20.0, -40.0, -60.0, -80.0, -90.0 })
                    sols.AddRange(BambooIcdMotion.IkSolveAxis(p.X, p.Y, p.Z, tilt, p.Axis, BambooIcdMotion.HangPose.Pole, BambooTrussMotion.PlaceAxisErrDeg));
                Assert.True(sols.Count > 0, $"{id} place unreachable under Motus jaw-axis IK");
            }
            Assert.True(p.Y - BambooIcdCell.StrutRadius > -0.002, $"{id} must not sink through the ground slab");
        }
        for (var i = 0; i < all.Count; i++)
        for (var j = i + 1; j < all.Count; j++)
        {
            var d = SegmentDistance(all[i].P, all[j].P, BambooTrussLayout.StrutLength);
            Assert.True(d > 2 * BambooIcdCell.StrutRadius + 0.002, $"{all[i].Id}–{all[j].Id} canes too close: {d * 1000:F1} mm");
        }
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

}

