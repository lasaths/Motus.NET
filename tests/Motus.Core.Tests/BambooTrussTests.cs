using Motus.Core;
using Motus.Geometry;
using Motus.Viewer.Services;
using K = Motus.Viewer.Services.BambooGripKinematics;

namespace Motus.Core.Tests;

/// <summary>
/// Grounded, base-swapping /bamboo-truss motion (true IK). Exercises the same viewer services the page runs and
/// re-derives every claim from the keys alone: one gripper always clamped on a real fixed object as the IK base,
/// no rigid moves of the whole robot, FK-exact grips, and a Motus-clear full path.
/// </summary>
public class BambooTrussTests
{
    private static readonly Lazy<BambooIcdCell> LazyCell = new(() =>
    {
        var cell = BambooIcdCell.LoadFile(UrdfPath, BambooTrussLayout.StrutLength);
        BambooTrussLayout.Configure(cell);
        return cell;
    });

    private static string UrdfPath => Path.GetFullPath(Path.Combine(
        AppContext.BaseDirectory, "..", "..", "..", "..", "..", "tests", "fixtures", "bamboo_icd", "bamboo_icd.urdf"));

    private static BambooIcdCell Cell => LazyCell.Value;

    // The plan is deterministic; build it once for the whole class (≈7 s desktop).
    private static readonly Lazy<BambooTrussMotion.TrussBuild> LazyBuild = new(() =>
    {
        BambooTrussMotion.UseCell(Cell);
        return BambooTrussMotion.Build();
    });

    private static BambooTrussMotion.TrussBuild Build()
    {
        var built = LazyBuild.Value;
        Assert.True(string.IsNullOrEmpty(built.Error), $"truss motion should build: {built.Error}");
        Assert.NotEmpty(built.Keys);
        Assert.NotEmpty(built.Cycles);
        return built;
    }

    private static BambooTrussScan Scan(BambooTrussMotion.TrussBuild built) =>
        new(Cell, built.Keys, built.Cycles, BambooTrussLayout.Members, BambooTrussLayout.JawClosedMm, 0.5);

    private const double Closed = BambooTrussLayout.JawClosedMm + 1e-9;

    // ---------------- layout ----------------

    [Fact]
    public void Layout_IsLowerChord_HalfMetreCanes_NoHangRod_GroundedFixtures()
    {
        Assert.Equal(0.5, BambooTrussLayout.StrutLength, 9);
        Assert.Equal(new[] { "baseL", "baseR" }, BambooTrussLayout.Members.Select(m => m.Id).ToArray());
        foreach (var m in BambooTrussLayout.Members)
        {
            var dx = m.EdgeB.X - m.EdgeA.X; var dy = m.EdgeB.Y - m.EdgeA.Y; var dz = m.EdgeB.Z - m.EdgeA.Z;
            Assert.Equal(BambooTrussLayout.StrutLength, Math.Sqrt(dx * dx + dy * dy + dz * dz), 6);
        }
        Assert.False(Cell.IncludePole);
        var pads = Cell.Pads.Select(p => p.Name).ToList();
        Assert.Contains("ground-S1", pads);
        Assert.Contains("ground-S2", pads);
        // Stakes stand on the ground (posts from y = 0) and the start base sits on S1's axis, slot along it.
        var posts = BambooTrussLayout.Posts.ToDictionary(p => p.Name);
        foreach (var st in BambooTrussLayout.Stakes)
        {
            Assert.True(posts.ContainsKey(st.Name));
            Assert.True(st.ClampY + 0.015 < st.Top, $"{st.Name}: L jaws must fit below the free top");
        }
        var b = BambooTrussLayout.StartBase;
        var (pos, axis) = K.Residual(b, BambooTrussLayout.S1.Grip);
        Assert.True(pos < 1e-12 && axis < 1e-9);
        Assert.Equal(new[] { "baseL", "baseR" }, BambooTrussLayout.WalkableMemberIdsAfter(2));
        Assert.Empty(BambooTrussLayout.WalkableMemberIdsAfter(1));
    }

    [Fact]
    public void Layout_CanesNeverIntersect_LashedAtB()
    {
        var all = BambooTrussLayout.Members.Select(m => (Id: m.Id, P: m.Place)).Append((Id: "store", P: BambooTrussLayout.Store)).ToList();
        for (var i = 0; i < all.Count; i++)
        for (var j = i + 1; j < all.Count; j++)
        {
            var d = SegmentDistance(all[i].P, all[j].P, BambooTrussLayout.StrutLength);
            Assert.True(d > 2 * BambooIcdCell.StrutRadius + 0.001, $"{all[i].Id}–{all[j].Id} canes too close: {d * 1000:F1} mm");
        }
        // The feed cane rests on its rail (seat gap only).
        Assert.Equal(BambooTrussLayout.RackTop + BambooIcdCell.StrutRadius + BambooTrussLayout.RackSeatGap, BambooTrussLayout.Store.Y, 9);
    }

    [Fact]
    public void GripKinematics_MatchMotusFk()
    {
        var rng = new Random(5);
        var worst = 0.0;
        for (var n = 0; n < 50; n++)
        {
            var q = Enumerable.Range(0, 5).Select(k => BambooIcdMotion.LimitsDeg[k].Min + (BambooIcdMotion.LimitsDeg[k].Max - BambooIcdMotion.LimitsDeg[k].Min) * rng.NextDouble()).ToArray();
            var lw = K.StakeClampFrame(rng.NextDouble() - 0.5, 0.1 + 0.3 * rng.NextDouble(), rng.NextDouble() - 0.5, 360 * rng.NextDouble());
            Cell.SetMountViewer(lw);
            var motus = Cell.JawWorld(q);
            var mine = BambooTrussPlanner.ToMotus(K.Mul(lw, K.Tlr(q)));
            for (var i = 0; i < 12; i++) worst = Math.Max(worst, Math.Abs(motus[i] - mine[i]));
            // R-base inverse chain gives back the same L frame.
            var back = BambooTrussPlanner.LWorld("R", K.Mul(lw, K.Tlr(q)), q);
            for (var i = 0; i < 12; i++) Assert.True(Math.Abs(back[i] - lw[i]) < 1e-9);
        }
        Cell.SetMountViewer(BambooTrussLayout.StartBase);
        Assert.True(worst < 1e-9, $"grip FK differs from Motus FK by {worst:E2}");
    }

    // ---------------- plan structure ----------------

    [Fact]
    public void Plan_BuildsBothMembers_ThenWalksWithTwoBaseSwaps()
    {
        var built = Build();
        Assert.Equal(new[] { "baseL", "baseR" }, built.Built.ToArray());
        Assert.Equal(new[] { "start", "baseL", "baseR", "walk" }, built.Cycles.Select(c => c.MemberId).ToArray());
        Assert.Equal(0, built.Cycles[0].T0, 9);
        Assert.Equal(1, built.Cycles[^1].T1, 9);
        var first = built.Keys[0];
        Assert.Equal("L", first.Base);
        Assert.Equal(BambooTrussLayout.S1.Name, first.BaseOn);
        Assert.Equal(BambooTrussLayout.StartBase, first.BaseWorld);
        var swaps = new List<(string From, string To, string On)>();
        for (var i = 1; i < built.Keys.Count; i++)
            if (built.Keys[i].Base != built.Keys[i - 1].Base)
                swaps.Add((built.Keys[i - 1].Base!, built.Keys[i].Base!, built.Keys[i].BaseOn!));
        Assert.Equal(2, swaps.Count);
        Assert.Equal(("L", "R", "placed-baseL"), swaps[0]);
        Assert.Equal(("R", "L", BambooTrussLayout.S2.Name), swaps[1]);
        for (var i = 1; i < built.Keys.Count; i++)
            Assert.True(built.Keys[i].T > built.Keys[i - 1].T, $"key {i} ({built.Keys[i].Label}) not after {built.Keys[i - 1].Label}");
    }

    // ---------------- grounded base ----------------

    [Fact]
    public void EverySegment_HasAClampedBase_OnARealFixedObject_ConstantWhileClamped()
    {
        var built = Build();
        var scan = Scan(built);
        var keys = built.Keys;
        for (var k = 1; k < keys.Count; k++)
        {
            var a = keys[k - 1]; var b = keys[k];
            Assert.NotNull(b.Base); Assert.NotNull(b.BaseWorld); Assert.NotNull(b.BaseOn);
            var g = b.Base!;
            // The base gripper is closed on its object at both ends of the segment (so throughout it).
            foreach (var key in new[] { a, b })
            {
                var jaw = g == "L" ? key.GL : key.GR;
                var on = (g == "L" ? key.LOn : key.ROn) ?? "";
                Assert.True(jaw <= Closed, $"{b.Label}: base {g} not closed ({jaw} mm) at {key.Label}");
                Assert.Contains(b.BaseOn!, on.Split('|'));
            }
            // Constant while the same gripper stays base.
            if (a.Base == b.Base) Assert.Equal(a.BaseWorld, b.BaseWorld);
            // The base frame coincides with the fixed object's grip frame (FK-exact clamp).
            var (pos, axis, inRange) = OnObject(scan, b.T, b.BaseOn!, b.BaseWorld!);
            Assert.True(pos < K.GripPosTolM && axis < K.GripAxisTolDeg && inRange,
                $"{b.Label}: base {g} {pos * 1000:F4} mm / {axis:F4}° off {b.BaseOn} (in range {inRange})");
            // Canes used as a base must be fixed at both ends (both chord members already placed).
            if (b.BaseOn!.StartsWith("placed-", StringComparison.Ordinal))
            {
                var placed = scan.World[BambooTrussMotion.MemberIndexAt(built.Cycles, b.T)].Placed.Select(p => p.Name).ToList();
                Assert.Equal(2, placed.Count);
                Assert.Contains(b.BaseOn["placed-".Length..], BambooTrussLayout.WalkableMemberIdsAfter(placed.Count));
            }
        }
    }

    [Fact]
    public void NoSample_HasBothGrippersFree()
    {
        var built = Build();
        var scan = Scan(built);
        foreach (var t in scan.SampleTimes)
        {
            var s = BambooTrussMotion.SampleState(built.Keys, t);
            var key = built.Keys[s.SegEnd];
            var lClamped = s.GL <= Closed && IsFixed(key.LOn);
            var rClamped = s.GR <= Closed && IsFixed(key.ROn);
            Assert.True(lClamped || rClamped, $"t={t:F4} ({s.Label}): both grippers free (gL {s.GL:F1}, gR {s.GR:F1})");
        }
    }

    [Fact]
    public void NoStep_MovesTheWholeRobotRigidly_AndEverySwingMovesJoints()
    {
        var built = Build();
        var keys = built.Keys;
        for (var k = 1; k < keys.Count; k++)
        {
            var a = keys[k - 1]; var b = keys[k];
            // Pose at key k-1 is the same whichever base is used (no teleport at a base swap).
            var (la, ra) = (BambooTrussPlanner.LWorld(a.Base, a.BaseWorld!, a.Q), BambooTrussPlanner.RWorld(a.Base, a.BaseWorld!, a.Q));
            var (lb, rb) = (BambooTrussPlanner.LWorld(b.Base, b.BaseWorld!, a.Q), BambooTrussPlanner.RWorld(b.Base, b.BaseWorld!, a.Q));
            Assert.True(MaxDiff(la, lb) < 1e-9 && MaxDiff(ra, rb) < 1e-9, $"{b.Label}: grippers jump at the segment start");
            // Frozen joints ⇒ nothing moves.
            if (K.JointDist(a.Q, b.Q) < 1e-9)
            {
                var (l2, r2) = (BambooTrussPlanner.LWorld(b.Base, b.BaseWorld!, b.Q), BambooTrussPlanner.RWorld(b.Base, b.BaseWorld!, b.Q));
                Assert.True(MaxDiff(la, l2) < 1e-9 && MaxDiff(ra, r2) < 1e-9, $"{b.Label}: robot moved with frozen joints");
            }
            if (a.Base != b.Base) Assert.True(K.JointDist(a.Q, b.Q) < 1e-9, $"{b.Label}: base swap must happen at rest");
        }
        // Each swing (stretch between base swaps) is joint motion.
        var start = 0;
        for (var k = 1; k <= keys.Count; k++)
        {
            if (k < keys.Count && keys[k].Base == keys[k - 1].Base) continue;
            var travel = 0.0;
            for (var i = start + 1; i < k; i++) travel = Math.Max(travel, K.JointDist(keys[start].Q, keys[i].Q));
            Assert.True(travel > 10, $"swing from {keys[start].Label} moves joints only {travel:F1}°");
            start = k;
        }
    }

    // ---------------- true grips ----------------

    [Fact]
    public void AllGrips_AreFkExact_WithinTightTolerance()
    {
        var built = Build();
        Assert.NotEmpty(built.Grips);
        foreach (var g in built.Grips)
            Assert.True(g.PosM < K.GripPosTolM && g.AxisDeg < K.GripAxisTolDeg,
                $"{g.Label} {g.Gripper} on {g.On}: {g.PosM * 1000:F4} mm / {g.AxisDeg:F4}°");
        Assert.True(K.GripPosTolM <= 0.0005 && K.GripAxisTolDeg <= 0.5);
    }

    [Fact]
    public void EveryClampedSample_HasTheJawOnItsObject_ByFk()
    {
        var built = Build();
        var scan = Scan(built);
        Assert.Null(scan.TrackMiss);
        var worstPos = 0.0; var worstAxis = 0.0; var checkedCount = 0;
        foreach (var t in scan.SampleTimes)
        {
            var s = BambooTrussMotion.SampleState(built.Keys, t);
            var key = built.Keys[s.SegEnd];
            var prev = built.Keys[Math.Max(0, s.SegEnd - 1)];
            var (l, r) = scan.GrippersAt(t);
            foreach (var (frame, jaw, on, onPrev, jawPrev) in new[] { (l, s.GL, key.LOn, prev.LOn, prev.GL), (r, s.GR, key.ROn, prev.ROn, prev.GR) })
            {
                // Clamped through the whole segment on a fixed object.
                if (jaw > Closed || jawPrev > Closed || !IsFixed(on) || !IsFixed(onPrev)) continue;
                var obj = on!.Split('|').First(o => !o.StartsWith(BambooIcdCell.Strut, StringComparison.Ordinal) || o.StartsWith("placed-", StringComparison.Ordinal));
                if (!(onPrev ?? "").Split('|').Contains(obj)) continue;
                var (pos, axis, inRange) = OnObject(scan, t, obj, frame);
                Assert.True(inRange, $"t={t:F4}: grip off the end of {obj}");
                worstPos = Math.Max(worstPos, pos); worstAxis = Math.Max(worstAxis, axis);
                checkedCount++;
            }
        }
        Assert.True(checkedCount > 1000, $"only {checkedCount} clamped samples checked");
        Assert.True(worstPos < K.GripPosTolM && worstAxis < K.GripAxisTolDeg, $"worst clamped residual {worstPos * 1000:F4} mm / {worstAxis:F4}°");
    }

    [Fact]
    public void EveryCane_IsReleasedExactlyOnItsMember()
    {
        var built = Build();
        var scan = Scan(built);
        foreach (var m in BambooTrussLayout.Members)
        {
            var w = scan.World.Single(x => x.ActiveId == m.Id);
            Assert.NotNull(w.Released);
            var want = BambooIcdCell.PlacementWorld(m.Place);
            var d = Math.Sqrt(Sq(w.Released![3] - want[3]) + Sq(w.Released[7] - want[7]) + Sq(w.Released[11] - want[11]));
            var dot = Math.Abs(w.Released[2] * want[2] + w.Released[6] * want[6] + w.Released[10] * want[10]);
            Assert.True(d < K.GripPosTolM, $"{m.Id}: released {d * 1000:F4} mm off design");
            Assert.True(Math.Acos(Math.Min(1, dot)) * 180 / Math.PI < K.GripAxisTolDeg, $"{m.Id}: released axis off design");
        }
    }

    [Fact]
    public void TamperedGrasp_IsCaughtAsAMiss()
    {
        var built = Build();
        var keys = built.Keys.Select(Clone).ToList();
        var close = keys.First(k => k.Label == "baseL:Close");
        close.Q = close.Q.ToArray();
        close.Q[2] += 2; // a jaw 2° off the planned grasp no longer encloses the cane on its slot axis
        var scan = new BambooTrussScan(Cell, keys, built.Cycles, BambooTrussLayout.Members, BambooTrussLayout.JawClosedMm, 0.5);
        Assert.NotNull(scan.TrackMiss);
        Assert.Contains("baseL", scan.TrackMiss);
    }

    // ---------------- Motus ----------------

    [Fact]
    public void FullPath_IsMotusClear()
    {
        var built = Build();
        var scan = Scan(built).RunToEnd();
        Assert.Null(scan.TrackMiss);
        Assert.True(scan.Complete);
        Assert.True(scan.FirstHit is null,
            $"Motus hit: {scan.FirstHit?.Describe()} at {scan.FirstHit?.T:F4} ({scan.FirstHit?.Label})");
        Assert.Equal(1.0, scan.LastClearT, 9);
        Assert.Equal(1.0, scan.Frontier, 9);
    }

    [Fact]
    public void PlacedMembers_AreLiveMotusObstacles()
    {
        var built = Build();
        var scan = Scan(built);
        var walk = built.Cycles.Single(c => c.MemberId == "walk");
        var env = scan.EnvironmentAt(walk.T0 + 1e-6, 0).Select(o => o.Name).ToList();
        Assert.Contains("placed-baseL", env);
        Assert.Contains("placed-baseR", env);
        Assert.Contains(BambooTrussLayout.S1.Name, env);
        Assert.Contains("store-rack", env);
        // The walk's chord grasp, evaluated at a time when the R-grip is not allowed to touch the chord, must hit it.
        var close = built.Keys.First(k => k.Label == "walk:Close");
        var t = built.Keys[walk.FirstKey].T;
        var contacts = scan.ContactsAt(t, close.Q, BambooTrussLayout.JawClosedMm, close.GL, 0);
        Assert.Contains(contacts, c => c.BodyA == "placed-baseL" || c.BodyB == "placed-baseL");
    }

    // ---------------- shipped plan (WASM) ----------------

    private static string PlanCachePath => Path.GetFullPath(Path.Combine(
        AppContext.BaseDirectory, "..", "..", "..", "..", "..", "src", "Motus.Viewer", "Services", "BambooTrussDefaultPlan.json"));

    /// <summary>
    /// The page loads the default plan from an embedded build-time copy (live planning takes minutes in WASM).
    /// It must be exactly what the planner produces now. Regenerate with BAMBOO_TRUSS_WRITE_PLAN=1.
    /// </summary>
    [Fact]
    public void ShippedDefaultPlan_IsIdenticalToLivePlanning()
    {
        var built = Build();
        var json = BambooTrussMotion.SerializePlan(built);
        if (Environment.GetEnvironmentVariable("BAMBOO_TRUSS_WRITE_PLAN") == "1")
            File.WriteAllText(PlanCachePath, json);
        Assert.True(File.Exists(PlanCachePath), "shipped plan missing — run with BAMBOO_TRUSS_WRITE_PLAN=1");
        Assert.Equal(json, File.ReadAllText(PlanCachePath));
        var cached = BambooTrussMotion.LoadCachedDefaultPlan();
        Assert.NotNull(cached);
        Assert.Equal(built.Keys.Count, cached!.Keys.Count);
        for (var i = 0; i < built.Keys.Count; i++)
        {
            Assert.Equal(built.Keys[i].Label, cached.Keys[i].Label);
            Assert.Equal(built.Keys[i].Q, cached.Keys[i].Q);
            Assert.Equal(built.Keys[i].BaseWorld, cached.Keys[i].BaseWorld);
            Assert.Equal(built.Keys[i].T, cached.Keys[i].T);
        }
        // The page path (useCache) returns the shipped plan for the default tasks.
        BambooTrussMotion.UseCell(Cell);
        var page = BambooTrussMotion.Build(useCache: true);
        Assert.Contains(page.Notes, n => n.StartsWith("plan: shipped", StringComparison.Ordinal));
    }

    // ---------------- playback ----------------

    private static List<string> PlannedPhases(BambooTrussMotion.TrussBuild built) => built.Keys.Select(k => k.Label).ToList();

    [Theory]
    [InlineData(0.016)]
    [InlineData(0.11)]
    public void Playback_ShowsEveryPlannedPhaseInOrder_EvenAcrossAStall(double frameSec)
    {
        var built = Build();
        var play = new BambooTrussPlayback(built.Keys);
        var shown = new List<string>();
        var frames = 0;
        var stalled = false;
        while (!play.Done && frames < 400_000)
        {
            var dt = frameSec;
            if (!stalled && play.T > 0.5) { dt = 16.0; stalled = true; }
            var t = play.Advance(dt);
            var label = BambooTrussMotion.SampleState(built.Keys, t).Label;
            if (shown.Count == 0 || shown[^1] != label) shown.Add(label);
            frames++;
        }
        Assert.True(play.Done);
        Assert.True(stalled);
        Assert.Equal(1.0, play.T, 9);
        Assert.Equal(PlannedPhases(built), play.Reached);
        var plan = PlannedPhases(built);
        Assert.Equal(plan.Where((l, i) => i == 0 || plan[i - 1] != l).ToList(), shown);
    }

    [Fact]
    public void Playback_NeverRunsAheadOfMotusFrontier_AndStillReachesEveryPhase()
    {
        var built = Build();
        var scan = Scan(built);
        var play = new BambooTrussPlayback(built.Keys);
        var frames = 0; var waited = 0;
        while (!play.Done && frames < 400_000)
        {
            if (!scan.Complete) scan.Step(3);
            var t = play.Advance(0.05, scan.Frontier);
            Assert.True(t <= scan.Frontier + 1e-12, $"playback t={t:F4} ran past Motus frontier {scan.Frontier:F4}");
            if (play.WaitingForMotus) waited++;
            frames++;
        }
        Assert.Null(scan.FirstHit);
        Assert.True(play.Done);
        Assert.True(waited > 0);
        Assert.Equal(PlannedPhases(built), play.Reached);
    }

    // ---------------- helpers ----------------

    private static bool IsFixed(string? on) =>
        !string.IsNullOrEmpty(on) && on.Split('|').Any(o =>
            o.StartsWith("stake-", StringComparison.Ordinal) || o.StartsWith("placed-", StringComparison.Ordinal));

    /// <summary>Distance (m) of a grip frame (viewer) from a fixed object's axis, slot-axis error (deg), within its length.</summary>
    private static (double Pos, double Axis, bool InRange) OnObject(BambooTrussScan scan, double t, string obj, double[] frameViewer)
    {
        double ox, oy, oz, ux, uy, uz, lo, hi;
        var stake = BambooTrussLayout.Stakes.FirstOrDefault(s => s.Name == obj);
        double[] f;
        if (stake is not null)
        {
            f = frameViewer;
            (ox, oy, oz, ux, uy, uz, lo, hi) = (stake.X, 0, stake.Z, 0, 1, 0, 0, stake.Top - 0.015);
        }
        else
        {
            Assert.StartsWith("placed-", obj);
            var w = scan.World.Select(x => x.Placed.FirstOrDefault(p => p.Name == obj).World)
                .Concat(scan.World.Where(x => "placed-" + x.ActiveId == obj).Select(x => x.Released))
                .First(m => m is not null)!;
            f = BambooTrussPlanner.ToMotus(frameViewer);
            (ox, oy, oz, ux, uy, uz) = (w[3], w[7], w[11], w[2], w[6], w[10]);
            (lo, hi) = (-BambooTrussLayout.StrutLength / 2, BambooTrussLayout.StrutLength / 2);
        }
        var px = f[3] - ox; var py = f[7] - oy; var pz = f[11] - oz;
        var along = px * ux + py * uy + pz * uz;
        var d = Math.Sqrt(Sq(px - along * ux) + Sq(py - along * uy) + Sq(pz - along * uz));
        var dot = Math.Abs(f[2] * ux + f[6] * uy + f[10] * uz);
        return (d, Math.Acos(Math.Min(1, dot)) * 180 / Math.PI, along >= lo && along <= hi);
    }

    private static double Sq(double x) => x * x;

    private static double MaxDiff(double[] a, double[] b)
    {
        var m = 0.0;
        for (var i = 0; i < 12; i++) m = Math.Max(m, Math.Abs(a[i] - b[i]));
        return m;
    }

    private static BambooIcdMotion.MotionKey Clone(BambooIcdMotion.MotionKey k) => new()
    {
        T = k.T, Label = k.Label, Q = k.Q.ToArray(), GR = k.GR, GL = k.GL, Hold = k.Hold, Base = k.Base,
        BaseWorld = k.BaseWorld?.ToArray(), BaseOn = k.BaseOn, LOn = k.LOn, ROn = k.ROn
    };

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
