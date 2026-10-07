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
        cell.IncludePole = false;
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
    public void Layout_IsBridgeLowerChord_TwoMembers_AtHalfMetre_NoHangRod()
    {
        Assert.Equal(0.5, BambooTrussLayout.StrutLength, 9);
        Assert.Equal(2, BambooTrussLayout.Members.Count);
        Assert.Equal(4, BambooTrussLayout.FootNodes.Length); // A,B,D + notional apex C
        Assert.Equal(new[] { "baseL", "baseR" }, BambooTrussLayout.Members.Select(m => m.Id).ToArray());
        Assert.Equal(new[] { "/", "_" }, BambooTrussLayout.Members.Select(m => m.Glyph).ToArray());
        Assert.Equal(2, BambooTrussLayout.AnchorPads.Length);
        Assert.All(BambooTrussLayout.FootNodes, n => Assert.Equal(BambooTrussLayout.PlaneZ, n.Z, 9));
        foreach (var m in BambooTrussLayout.Members)
        {
            var dx = m.EdgeB.X - m.EdgeA.X; var dy = m.EdgeB.Y - m.EdgeA.Y; var dz = m.EdgeB.Z - m.EdgeA.Z;
            Assert.Equal(BambooTrussLayout.StrutLength, Math.Sqrt(dx * dx + dy * dy + dz * dz), 6);
        }
        Assert.Empty(BambooTrussLayout.WalkableMemberIdsAfter(1));
        Assert.Equal(new[] { "baseL", "baseR" }, BambooTrussLayout.WalkableMemberIdsAfter(2));
    }

    [Fact]
    public void DefaultTasks_AreStoreToMemberPickPlacePairs()
    {
        var tasks = BambooTrussMotion.DefaultTasks();
        Assert.Equal(4, tasks.Count);
        var pairs = BambooTrussMotion.Pairs(tasks);
        Assert.Equal(2, pairs.Count);
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
        Assert.Equal(3, built.Cycles.Count); // baseL + baseR + brachiate

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
        var memberCycles = built.Cycles.Where(c => c.MemberId != "brachiate").ToList();
        Assert.Equal(BambooTrussLayout.Members.Count, memberCycles.Count);
        for (var i = 0; i < memberCycles.Count; i++)
        {
            var c = memberCycles[i];
            var member = BambooTrussLayout.Members[i];
            // Sample just before the cycle ends while Hold=2 (released on member).
            var t = c.T0 + (c.T1 - c.T0) * 0.99;
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
    public void TrussCell_OmitsHangPole_AndUsesAnchorPads()
    {
        Assert.False(Cell.IncludePole);
        Assert.Equal(2, Cell.AnchorPads.Count);
    }

    [Fact]
    public void FullTruss_PlacedMembersAreLiveMotusObstacles()
    {
        // Force baseR release onto baseL's place so the second cycle must hit placed-baseL.
        var members = BambooTrussLayout.Members.ToList();
        var last = members[^1];
        members[^1] = last with { Place = members[0].Place };
        BambooTrussMotion.UseCell(Cell);
        var built = BambooTrussMotion.Build(BambooTrussMotion.DefaultTasks(members));
        // Overlapping release may fail IK/link; if Build succeeds, scan must report placed-* contact.
        if (!string.IsNullOrEmpty(built.Error))
        {
            Assert.Contains("baseR", built.Error, StringComparison.OrdinalIgnoreCase);
            return;
        }
        var scan = new BambooTrussScan(Cell, built.Keys, built.Cycles, members, maxStepDeg: 0.5).RunToEnd();
        var hit = Assert.IsType<BambooTrussScan.Hit>(scan.FirstHit);
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
        // Adjacent chord canes share node B (d≈0 at the joint); only non-adjacent / store pairs must clear.
        var memberById = BambooTrussLayout.Members.ToDictionary(m => m.Id);
        for (var i = 0; i < all.Count; i++)
        for (var j = i + 1; j < all.Count; j++)
        {
            if (all[i].Id != "store" && all[j].Id != "store"
                && memberById.TryGetValue(all[i].Id, out var mi)
                && memberById.TryGetValue(all[j].Id, out var mj)
                && SharesNode(mi, mj))
                continue;
            var d = SegmentDistance(all[i].P, all[j].P, BambooTrussLayout.StrutLength);
            Assert.True(d > 2 * BambooIcdCell.StrutRadius + 0.002, $"{all[i].Id}–{all[j].Id} canes too close: {d * 1000:F1} mm");
        }
    }

    private static bool SharesNode(BambooTrussLayout.MemberDef a, BambooTrussLayout.MemberDef b)
    {
        bool Near((double X, double Y, double Z) p, (double X, double Y, double Z) q) =>
            (p.X - q.X) * (p.X - q.X) + (p.Y - q.Y) * (p.Y - q.Y) + (p.Z - q.Z) * (p.Z - q.Z) < 1e-8;
        return Near(a.EdgeA, b.EdgeA) || Near(a.EdgeA, b.EdgeB) || Near(a.EdgeB, b.EdgeA) || Near(a.EdgeB, b.EdgeB);
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
    public void FullTrussMotion_BuildsMemberCycles_ThenBrachiate()
    {
        var built = Build();
        Assert.Equal(3, built.Cycles.Count);
        Assert.Equal(new[] { "baseL", "baseR", "brachiate" }, built.Cycles.Select(c => c.MemberId).ToArray());
        Assert.Equal(0, built.Cycles[0].T0, 9);
        Assert.Equal(1, built.Cycles[^1].T1, 9);
        Assert.Equal(BambooTrussLayout.StartDock, built.Cycles[0].PickHang);
        Assert.Equal(BambooTrussLayout.WalkHang, built.Cycles[^1].PlaceHang);
    }


    [Fact]
    public void WalkHang_IsOutboardLGripAtA_OnDoubleFixedChord()
    {
        var a = BambooTrussLayout.FootNodes[0];
        var w = BambooTrussLayout.WalkHang;
        Assert.Equal(a.X - BambooTrussLayout.WalkHangOffset, w.X, 9);
        Assert.Equal(a.Y, w.Y, 9);
        Assert.Equal(BambooTrussLayout.PlaneZ, w.Z, 9);
        Assert.Equal(new[] { "baseL", "baseR" }, BambooTrussLayout.WalkableMemberIdsAfter(2));
        Assert.True(BambooTrussLayout.BrachiateHangPath.Count >= 4);
        Assert.Equal(BambooTrussLayout.StartDock, BambooTrussLayout.BrachiateHangPath[0]);
        Assert.Equal(BambooTrussLayout.WalkHang, BambooTrussLayout.BrachiateHangPath[3]);
    }

    [Fact]
    public void SepAndDiags_AreJawAxisUnreachable_FromWalkHang()
    {
        // Motus-honest blocker: on-chord WalkHang cannot IK-reach apex members at 0.5 m.
        static bool Ik(BambooIcdCell.StrutPlacement p, BambooIcdMotion.HangPose hang)
        {
            foreach (var tilt in new[] { p.PitchDeg, -Math.Abs(p.PitchDeg), 0.0, -40.0, -60.0, -80.0, -90.0, 20.0, 40.0 })
                if (BambooIcdMotion.IkSolveAxis(p.X, p.Y, p.Z, tilt, p.Axis, hang, BambooTrussMotion.PlaceAxisErrDeg).Count > 0)
                    return true;
            return false;
        }
        var hang = BambooTrussLayout.WalkHang;
        Assert.False(Ik(BambooTrussLayout.NotionalSep, hang));
        Assert.False(Ik(BambooTrussLayout.NotionalDiagL, hang));
        Assert.False(Ik(BambooTrussLayout.NotionalDiagR, hang));
        // Along-chord outboard hangs likewise cannot place sep.
        for (var t = 0.0; t <= 1.0; t += 0.25)
            Assert.False(Ik(BambooTrussLayout.NotionalSep, BambooTrussLayout.WalkHangAt(t)), $"sep reachable at WalkHangAt({t:F2})");
    }

    [Fact]
    public void WalkHang_HomeIsMotusClear_WithChordPlanted()
    {
        var home = BambooIcdMotion.HomeDeg.ToArray();
        var keys = new List<BambooIcdMotion.MotionKey>
        {
            new() { T = 0, Label = "Home", Q = home, GR = BambooTrussLayout.JawOpenMm, Hold = 0 },
            new() { T = 1, Label = "Home", Q = home, GR = BambooTrussLayout.JawOpenMm, Hold = 0 }
        };
        var extras = BambooTrussLayout.Members
            .Select(m => ($"placed-{m.Id}", m.Place)).ToList();
        try
        {
            Cell.SetHang(BambooTrussLayout.WalkHang);
            var track = Cell.TrackStrut(keys, BambooTrussLayout.Store);
            Assert.True(Cell.IsClear(home, BambooTrussLayout.JawOpenMm, 24, 0, "Home", track, extras),
                "WalkHang Home must be Motus-clear with baseL+baseR planted");
        }
        finally
        {
            Cell.SetHang(BambooTrussLayout.StartDock);
        }
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

