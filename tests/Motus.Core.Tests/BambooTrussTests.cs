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
    public void Layout_IsSevenMemberWarrenTruss_EquilateralPanels_NoHangRod_NoStakes()
    {
        Assert.Equal(new[] { "AM", "AT1", "T1M", "MT2", "T1T2", "MD", "T2D" }, BambooTrussLayout.Members.Select(m => m.Id).ToArray());
        var nodes = BambooTrussLayout.FootNodes.ToDictionary(n => n.Name);
        Assert.Equal(new[] { "A", "D", "M", "T1", "T2" }, nodes.Keys.OrderBy(n => n));
        var p = BambooTrussLayout.Panel;
        foreach (var m in BambooTrussLayout.Members)
        {
            // Every edge is one panel long (equilateral triangles, 60°) and its end nodes are what the id says.
            var dx = m.EdgeB.X - m.EdgeA.X; var dy = m.EdgeB.Y - m.EdgeA.Y; var dz = m.EdgeB.Z - m.EdgeA.Z;
            Assert.Equal(p, Math.Sqrt(dx * dx + dy * dy + dz * dz), 9);
            Assert.Equal(m.Id, m.NodeA + m.NodeB);
            Assert.Equal((nodes[m.NodeA].X, nodes[m.NodeA].Y), (m.EdgeA.X, m.EdgeA.Y));
        }
        Assert.Equal(nodes["A"].Y, nodes["M"].Y, 12);
        Assert.Equal(nodes["M"].Y, nodes["D"].Y, 12);
        Assert.Equal(p * Math.Sqrt(3) / 2, nodes["T1"].Y - nodes["A"].Y, 12);
        Assert.Equal(nodes["T1"].Y, nodes["T2"].Y, 12);
        Assert.False(Cell.IncludePole);
        // No stakes: the only clamp supports are the anchor columns under A and D, on the node axes.
        Assert.Equal(new[] { "anchor-A", "anchor-D" }, BambooTrussLayout.Stakes.Select(s => s.Name).ToArray());
        Assert.DoesNotContain(BambooTrussLayout.FixtureObjects, o => o.Name.StartsWith("stake", StringComparison.Ordinal));
        Assert.DoesNotContain(Cell.Pads, o => o.Name.Contains("S1") || o.Name.Contains("S3") || o.Name.StartsWith("stake", StringComparison.Ordinal));
        foreach (var (st, node) in new[] { (BambooTrussLayout.StakeNamed("anchor-A"), nodes["A"]), (BambooTrussLayout.StakeNamed("anchor-D"), nodes["D"]) })
        {
            Assert.Equal((node.X, node.Z), (st.X, st.Z));
            Assert.Equal(node.Y - BambooTrussLayout.HubHalf, st.Top, 12);
            Assert.True(st.ClampY + 0.015 < st.Top);
        }
        Assert.Equal("anchor-A", BambooTrussLayout.S1.Name);
        var (pos, axis) = K.Residual(BambooTrussLayout.StartBase, BambooTrussLayout.S1.Grip);
        Assert.True(pos < 1e-12 && axis < 1e-9);
    }

    /// <summary>Lassie: all elements in one plane, each cane = panel − 2 × the same node gap, connectors at every node.</summary>
    [Fact]
    public void Members_AreCoplanar_PanelMinusTwoGaps_WithAConnectorAtEveryNode()
    {
        var p = BambooTrussLayout.Panel; var g = BambooTrussLayout.NodeGap;
        Assert.Equal(p - 2 * g, BambooTrussLayout.StrutLength, 12);
        Assert.True(g > BambooTrussLayout.HubHalf * Math.Sqrt(2) + BambooIcdCell.StrutRadius, "cane ends must clear the connector cube");
        foreach (var m in BambooTrussLayout.Members)
        {
            var pl = m.Place; var u = pl.Axis;
            // Axis in the vertical plane z = PlaneZ (both ends), centred on its edge.
            Assert.Equal(BambooTrussLayout.PlaneZ, pl.Z, 12);
            Assert.Equal(0, u.Z, 12);
            Assert.Equal((m.EdgeA.X + m.EdgeB.X) / 2, pl.X, 12);
            Assert.Equal((m.EdgeA.Y + m.EdgeB.Y) / 2, pl.Y, 12);
        }
        foreach (var n in BambooTrussLayout.FootNodes) Assert.Equal(BambooTrussLayout.PlaneZ, n.Z, 12);
        // Canes in one plane never touch each other.
        var all = BambooTrussLayout.Members.Select(m => (Id: m.Id, P: m.Place)).Append((Id: "store", P: BambooTrussLayout.Store)).ToList();
        for (var i = 0; i < all.Count; i++)
        for (var j = i + 1; j < all.Count; j++)
        {
            var d = SegmentDistance(all[i].P, all[j].P, BambooTrussLayout.StrutLength);
            Assert.True(d > 2 * BambooIcdCell.StrutRadius + 0.001, $"{all[i].Id}–{all[j].Id} canes too close: {d * 1000:F1} mm");
        }
        Assert.Equal(BambooTrussLayout.RackTop + BambooIcdCell.StrutRadius + BambooTrussLayout.RackSeatGap, BambooTrussLayout.Store.Y, 9);
        // Connectors: A and D always (supports), inner nodes once a member reaches them — Motus boxes.
        Assert.Contains(BambooTrussLayout.FixtureObjects, o => o.Name == "hub-A");
        Assert.Contains(BambooTrussLayout.FixtureObjects, o => o.Name == "hub-D");
        Assert.Empty(BambooTrussLayout.HubObjects(Array.Empty<string>()));
        Assert.Equal(new[] { "hub-M", "hub-T1" }, BambooTrussLayout.HubObjects(new[] { "AT1", "T1M" }).Select(o => o.Name).ToArray());
        Assert.Equal(new[] { "M", "T1", "T2" }, BambooTrussLayout.InnerHubNodes(BambooTrussLayout.Members.Select(m => m.Id)));
    }

    /// <summary>
    /// Lassie's pit: ground at support level outside A and D, a U-section pit between them (two walls + floor), every
    /// surface a Motus obstacle; no earth inside the span — the stock and its pegs lie outboard of A.
    /// </summary>
    [Fact]
    public void Pit_IsAMotusObstacle_AndThereIsNoEarthInsideTheSpan()
    {
        var hs = BambooTrussLayout.HalfSpan;
        var all = BambooTrussLayout.GroundXExtents().ToList();
        Assert.Contains(all, g => g.Name == "anchor-A" && g.IsAnchor);
        Assert.Contains(all, g => g.Name == "anchor-D" && g.IsAnchor);
        foreach (var g in all.Where(g => !g.IsAnchor))
            Assert.True(g.MaxX <= -hs + BambooTrussLayout.PitEdgeInset + 1e-9 || g.MinX >= hs - BambooTrussLayout.PitEdgeInset - 1e-9,
                $"{g.Name} spans x [{g.MinX:F3}, {g.MaxX:F3}] — inside the pit");
        Assert.Equal(new[] { "ground-E", "ground-W" }, Cell.Pads.Select(p => p.Name).OrderBy(n => n));
        var pit = BambooTrussLayout.PitBoxes.ToDictionary(b => b.Name);
        Assert.Equal(new[] { "pit-floor", "pit-wall-E", "pit-wall-W" }, pit.Keys.OrderBy(n => n));
        // Walls go from ground level down to the floor, at the pit edges just inboard of A / D; the floor spans them.
        var d = BambooTrussLayout.Current.PitDepth;
        Assert.True(d >= 0.2);
        foreach (var w in new[] { pit["pit-wall-W"], pit["pit-wall-E"] })
        {
            Assert.Equal(0, w.Y + w.HalfY, 12);
            Assert.Equal(-d, w.Y - w.HalfY, 12);
        }
        Assert.Equal(-hs + BambooTrussLayout.PitEdgeInset, pit["pit-wall-W"].X + pit["pit-wall-W"].HalfX, 12);
        Assert.Equal(hs - BambooTrussLayout.PitEdgeInset, pit["pit-wall-E"].X - pit["pit-wall-E"].HalfX, 12);
        Assert.Equal(-d, pit["pit-floor"].Y + pit["pit-floor"].HalfY, 12);
        Assert.True(pit["pit-floor"].HalfX >= hs - BambooTrussLayout.PitEdgeInset);
        foreach (var n in pit.Keys) Assert.Contains(BambooTrussLayout.FixtureObjects, o => o.Name == n);
        // Stock flat outboard of A, outside the pit.
        foreach (var r in BambooTrussLayout.Racks) Assert.True(r.X + r.HalfX < -hs);
        Assert.True(BambooTrussLayout.Store.X + BambooIcdCell.StrutRadius <= -hs, "stock cane lies inside the span");
        Assert.Equal(0, BambooTrussLayout.Store.PitchDeg, 12);
    }

    /// <summary>
    /// "Fixed into the structure": both end nodes in a rigid (triangulated) subframe that contains an anchor. A lone
    /// cantilevered member is not fixed; a closed triangle hinged at A is.
    /// </summary>
    [Fact]
    public void FixedMembers_AreThoseOfAnAnchoredRigidSubframe()
    {
        Assert.Empty(BambooTrussLayout.FixedBothEnds(new[] { "AT1" }));
        Assert.Empty(BambooTrussLayout.FixedBothEnds(new[] { "AT1", "T1M" }));
        Assert.Equal(new[] { "AM", "AT1", "T1M" }, BambooTrussLayout.FixedBothEnds(new[] { "AT1", "T1M", "AM" }).OrderBy(x => x));
        Assert.Equal(new[] { "A", "D", "M", "T1" }, BambooTrussLayout.FixedNodes(new[] { "AT1", "T1M", "AM" }).OrderBy(x => x));
        // Triangle 2 grows the frame (T2 tied by two members), MD alone hangs off M until T2D closes triangle 3.
        Assert.Equal(5, BambooTrussLayout.FixedBothEnds(new[] { "AT1", "T1M", "AM", "MT2", "T1T2" }).Count);
        Assert.DoesNotContain("MD", BambooTrussLayout.FixedBothEnds(new[] { "AT1", "T1M", "AM", "MD" }));
        Assert.Equal(7, BambooTrussLayout.FixedBothEnds(BambooTrussLayout.Members.Select(m => m.Id)).Count);
        Assert.Equal(new[] { "A", "D" }, BambooTrussLayout.ExistingNodes(Array.Empty<string>()).OrderBy(x => x));
    }

    /// <summary>The far members are out of reach from the A column (the reason triangle 1 is built from A, then walked).</summary>
    [Fact]
    public void FarMembers_AreBeyondReach_OfTheAColumn()
    {
        var rng = new Random(11);
        var reach = 0.0;
        for (var n = 0; n < 200_000; n++)
        {
            var q = Enumerable.Range(0, 5).Select(k => BambooIcdMotion.LimitsDeg[k].Min + (BambooIcdMotion.LimitsDeg[k].Max - BambooIcdMotion.LimitsDeg[k].Min) * rng.NextDouble()).ToArray();
            var t = K.Tlr(q);
            reach = Math.Max(reach, Math.Sqrt(t[3] * t[3] + t[7] * t[7] + t[11] * t[11]));
        }
        Assert.InRange(reach, 0.3, 0.45);
        var a = BambooTrussLayout.S1;
        double Far(string id)
        {
            var m = BambooTrussLayout.Members.Single(x => x.Id == id);
            var near = Math.Min(Math.Sqrt(Sq(m.EdgeA.X - a.X) + Sq(m.EdgeA.Y - a.ClampY)), Math.Sqrt(Sq(m.EdgeB.X - a.X) + Sq(m.EdgeB.Y - a.ClampY)));
            return near + BambooTrussLayout.NodeGap + 0.025; // nearest cane grasp point
        }
        Assert.True(Far("T2D") > reach, $"T2D nearest grasp {Far("T2D"):F3} m vs reach {reach:F3} m");
        Assert.True(Far("AT1") < reach && Far("AM") < reach && Far("T1M") < reach);
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
    public void Plan_BuildsTriangle1FromTheAColumn_WalksOntoIt_ReportsTheRest()
    {
        var built = Build();
        // Triangle 1 (A–T1, T1–M, then A–M: with A–M first the in-plane arm hits it placing T1–M).
        Assert.Equal(new[] { "AT1", "T1M", "AM" }, built.Built.ToArray());
        foreach (var id in new[] { "MT2", "T1T2", "MD", "T2D" })
            Assert.Contains(built.Notes, n => n.StartsWith(id + ": not built", StringComparison.Ordinal));
        Assert.Equal(new[] { "AM", "AT1", "T1M" }, BambooTrussLayout.FixedBothEnds(built.Built).OrderBy(x => x));
        Assert.Equal(new[] { "start", "AT1", "T1M", "AM", "walk (node) → AM@-1.00", "walk (node) → anchor-A" },
            built.Cycles.Select(c => c.MemberId).ToArray());
        Assert.Equal(0, built.Cycles[0].T0, 9);
        Assert.Equal(1, built.Cycles[^1].T1, 9);
        var first = built.Keys[0];
        Assert.Equal("L", first.Base);
        Assert.Equal("anchor-A", first.BaseOn);
        Assert.Equal(BambooTrussLayout.StartBase, first.BaseWorld);
        var swaps = new List<(string From, string To, string On)>();
        for (var i = 1; i < built.Keys.Count; i++)
            if (built.Keys[i].Base != built.Keys[i - 1].Base)
                swaps.Add((built.Keys[i - 1].Base!, built.Keys[i].Base!, built.Keys[i].BaseOn!));
        Assert.Equal(new[]
        {
            ("L", "R", "placed-AM"), ("R", "L", "placed-AM"),    // A column → onto the closed triangle (R on A–M)
            ("L", "R", "placed-AM"), ("R", "L", "anchor-A")      // back onto the A column
        }, swaps.ToArray());
        // The L clamp leaves the A column sideways (connector above, no free top).
        Assert.Contains(built.Keys, k => k.Label == "walk (node) → AM@-1.00:L:PTP off anchor-A");
        Assert.DoesNotContain(built.Keys, k => k.BaseOn is { } on && !on.StartsWith("anchor-", StringComparison.Ordinal) && !on.StartsWith("placed-", StringComparison.Ordinal));
        for (var i = 1; i < built.Keys.Count; i++)
            Assert.True(built.Keys[i].T > built.Keys[i - 1].T, $"key {i} ({built.Keys[i].Label}) not after {built.Keys[i - 1].Label}");
    }

    /// <summary>Every base swap is its own labelled key ("swap base L→R" / "swap base R→L"), never merged into an Open.</summary>
    [Fact]
    public void EveryBaseSwap_HasItsOwnLabel()
    {
        var built = Build();
        var n = 0;
        for (var i = 1; i < built.Keys.Count; i++)
        {
            var a = built.Keys[i - 1]; var b = built.Keys[i];
            if (a.Base == b.Base) continue;
            Assert.EndsWith($"swap base {a.Base}→{b.Base}", b.Label);
            Assert.Equal(a.GL, b.GL); Assert.Equal(a.GR, b.GR); // the swap itself opens nothing
            Assert.DoesNotContain("Open", b.Label);
            Assert.Contains("Open", built.Keys[i + 1].Label); // the old base opens on the next key
            n++;
        }
        Assert.Equal(4, n); // two walk steps, two swaps each
    }

    /// <summary>PTP: every placement leaves R a way out — the member cycle ends with R parked (Motus-clear).</summary>
    [Fact]
    public void EveryPlacement_EndsWithRParked()
    {
        var built = Build();
        foreach (var c in built.Cycles.Where(c => built.Built.Contains(c.MemberId)))
            Assert.EndsWith($"{c.MemberId}:R:Park", built.Keys[c.LastKey].Label);
    }

    /// <summary>Each cane is placed with at least one end on an existing node (an anchor or a placed member's node).</summary>
    [Fact]
    public void EveryPlacedMember_HadAnExistingEndNodeWhenPlaced()
    {
        var built = Build();
        var placed = new List<string>();
        foreach (var id in built.Built)
        {
            var m = BambooTrussLayout.Members.Single(x => x.Id == id);
            var nodes = BambooTrussLayout.ExistingNodes(placed);
            Assert.True(nodes.Contains(m.NodeA) || nodes.Contains(m.NodeB), $"{id} placed with neither end node existing");
            placed.Add(id);
        }
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
                var placed = scan.World[BambooTrussMotion.CycleIndexAt(built.Keys, built.Cycles, b.T)].Placed.Select(p => p.Name["placed-".Length..]).ToList();
                Assert.Contains(b.BaseOn["placed-".Length..], BambooTrussLayout.FixedBothEnds(placed));
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
        Assert.NotEmpty(built.Built);
        foreach (var m in BambooTrussLayout.Members.Where(x => built.Built.Contains(x.Id)))
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
        var close = keys.First(k => k.Label == "AT1:Close");
        close.Q = close.Q.ToArray();
        close.Q[2] += 2; // a jaw 2° off the planned grasp no longer encloses the cane on its slot axis
        var scan = new BambooTrussScan(Cell, keys, built.Cycles, BambooTrussLayout.Members, BambooTrussLayout.JawClosedMm, 0.5);
        Assert.NotNull(scan.TrackMiss);
        Assert.Contains("AT1", scan.TrackMiss);
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
        var after = built.Cycles.Single(c => c.MemberId == "T1M");
        var t = built.Keys[after.FirstKey].T;
        var env = scan.EnvironmentAt(t, 0).Select(o => o.Name).ToList();
        Assert.Contains("placed-AT1", env);
        foreach (var st in BambooTrussLayout.Stakes) Assert.Contains(st.Name, env);
        // The pit (walls + floor), the ground surfaces' supports and the connectors are Motus obstacles too.
        foreach (var n in new[] { "pit-wall-W", "pit-wall-E", "pit-floor", "hub-A", "hub-D", "hub-T1" }) Assert.Contains(n, env);
        Assert.Contains("store-peg-1", env);
        Assert.Contains("store-peg-2", env);
        // The place pose of AT1 (same base: L on the A column), replayed in the T1M cycle with the jaws closed and nothing in hand,
        // puts the R jaw around the placed cane — which it may no longer touch — so Motus must report it.
        var lower = built.Keys.Last(k => k.Label == "AT1:PTP Place");
        var contacts = scan.ContactsAt(t, lower.Q, BambooTrussLayout.JawClosedMm, lower.GL, 0);
        Assert.Contains(contacts, c => c.BodyA == "placed-AT1" || c.BodyB == "placed-AT1");
    }

    /// <summary>
    /// The cane in hand is a Motus attached body checked against placed canes: a cane laid across the held cane's far
    /// end (well away from the jaw) is reported with the cane in hand and not without it.
    /// </summary>
    [Fact]
    public void HeldCane_IsMotusCheckedAgainstPlacedCanes()
    {
        var built = Build();
        var scan = Scan(built);
        var cycle = built.Cycles.Single(c => c.MemberId == "AT1");
        var world = scan.World[built.Cycles.IndexOf(cycle)];
        Assert.NotNull(world.HeldOffset);
        var lift = built.Keys.Last(k => k.Label == "AT1:PTP Lift");
        var (lw, rw) = scan.GrippersAt(lift.T);
        var held = Transforms.Multiply(BambooTrussPlanner.ToMotus(rw), Transforms.FromFrame(world.HeldOffset!.Value));
        var grip = BambooTrussPlanner.ToMotus(rw);
        // Far end of the held cane from the jaw.
        var along = (grip[3] - held[3]) * held[2] + (grip[7] - held[7]) * held[6] + (grip[11] - held[11]) * held[10];
        var probe = Transforms.Multiply(held, Transforms.FromAxisAngle(1, 0, 0, Math.PI / 2));
        var e = -Math.Sign(along) * (BambooTrussLayout.StrutLength / 2 - 0.03);
        probe[3] = held[3] + held[2] * e; probe[7] = held[7] + held[6] * e; probe[11] = held[11] + held[10] * e;
        var env = scan.EnvironmentAt(lift.T, 1).Append(Cell.StrutAt(probe, "placed-probe")).ToList();
        Cell.SetMountViewer(lw);
        var withCane = Cell.PoseContacts(lift.Q, lift.GR, lift.GL, world.HeldOffset, env, BambooTrussPlanner.Allowed(lift.LOn, lift.ROn));
        var without = Cell.PoseContacts(lift.Q, BambooTrussLayout.JawOpenMm, lift.GL, null, env, BambooTrussPlanner.Allowed(lift.LOn, null));
        Cell.SetMountViewer(BambooTrussLayout.StartBase);
        Assert.Contains(withCane, c => c.BodyA == "placed-probe" || c.BodyB == "placed-probe");
        Assert.DoesNotContain(without, c => c.BodyA == "placed-probe" || c.BodyB == "placed-probe");
    }

    // ---------------- playback state (world / hold per segment) ----------------

    /// <summary>
    /// Key k's state applies to the segment ending at k. At 1642a3b the hand-over segment from one member's last key
    /// (released, hold 2) into the next member's first key kept hold 2 while the world already belonged to the next
    /// cycle, so the next cane was drawn as already placed and the feed was empty. Hold and world now both follow
    /// the segment-end key.
    /// </summary>
    [Fact]
    public void SampleState_HoldFollowsTheSegmentEndKey_AcrossACycleBoundary()
    {
        var keys = new List<BambooIcdMotion.MotionKey>
        {
            new() { T = 0, Label = "AT1:Retreat", Q = new double[5], Hold = 2 },
            new() { T = 1, Label = "baseR:Approach", Q = new double[] { 10, 0, 0, 0, 0 }, Hold = 0 }
        };
        foreach (var t in new[] { 0.01, 0.3, 0.6, 0.99 })
            Assert.Equal(0, BambooTrussMotion.SampleState(keys, t).Hold);
    }

    [Fact]
    public void EverySample_TakesHoldAndWorld_FromTheSameCycle()
    {
        var built = Build();
        var scan = Scan(built);
        foreach (var t in scan.SampleTimes)
        {
            var s = BambooTrussMotion.SampleState(built.Keys, t);
            Assert.Equal(built.Keys[s.SegEnd].Hold, s.Hold);
            var ci = BambooTrussMotion.CycleIndexAt(built.Keys, built.Cycles, t);
            Assert.InRange(s.SegEnd, built.Cycles[ci].FirstKey, built.Cycles[ci].LastKey);
        }
        // Each member cycle opens with its cane on the feed pegs and not yet placed — including on the hand-over segment.
        for (var i = 1; i < built.Cycles.Count; i++)
        {
            var c = built.Cycles[i];
            var t = (built.Keys[c.FirstKey - 1].T + built.Keys[c.FirstKey].T) / 2;
            var s = BambooTrussMotion.SampleState(built.Keys, t);
            var poses = scan.StrutPoses(t, s.Hold, s.Q).Select(p => p.Name).ToList();
            Assert.DoesNotContain("placed-" + c.MemberId, poses);
            if (scan.World[i].ActiveId is not null)
            {
                Assert.Equal(0, s.Hold);
                Assert.Contains(BambooIcdCell.Strut, poses);
            }
            foreach (var prev in built.Cycles.Take(i).Where(p => built.Built.Contains(p.MemberId)))
                Assert.Contains("placed-" + prev.MemberId, poses);
        }
    }

    /// <summary>
    /// Point-to-point only: no Cartesian approach / lower / slide / back-off / lift segments anywhere in the plan; every
    /// grasp, place and clamp is reached with a joint-space PTP move (labelled "PTP …").
    /// </summary>
    [Fact]
    public void EveryMove_IsPointToPoint()
    {
        var built = Build();
        foreach (var k in built.Keys)
            foreach (var bad in new[] { ":Approach", "Slide up", "Slide down", ":Lower", "Back off", ":Lift", ":Retreat", ":Transit", ":Carry" })
                Assert.False(k.Label.Contains(bad, StringComparison.Ordinal), $"non-PTP segment {k.Label}");
        Assert.Contains(built.Keys, k => k.Label == "AT1:PTP Grasp");
        Assert.Contains(built.Keys, k => k.Label == "AT1:PTP Place");
        Assert.Contains(built.Keys, k => k.Label.EndsWith("L:PTP to placed-AM", StringComparison.Ordinal));
        Assert.True(BambooTrussPlanner.Ptp);
    }

    // ---------------- clamp markers ----------------

    /// <summary>
    /// The viewer draws a ring + label for every clamped gripper. Wherever a gripper is closed on a fixed object the
    /// marker list has it — both when both clamp (e.g. the walk close: R low on S3 as base, L on S3).
    /// </summary>
    [Fact]
    public void ClampMarkers_CoverEveryClampedGripper_IncludingBoth()
    {
        var built = Build();
        var scan = Scan(built);
        var both = 0;
        foreach (var t in scan.SampleTimes)
        {
            var s = BambooTrussMotion.SampleState(built.Keys, t);
            var key = built.Keys[s.SegEnd];
            var clamps = scan.ClampsAt(t, s.Q, s.GR, s.GL);
            Assert.Single(clamps, c => c.IsBase);
            if (s.GL <= Closed && IsFixed(key.LOn)) Assert.Contains(clamps, c => c.Gripper == "L" && !c.Held);
            if (s.GR <= Closed && IsFixed(key.ROn)) Assert.Contains(clamps, c => c.Gripper == "R" && !c.Held);
            if (clamps.Count(c => !c.Held) == 2) both++;
        }
        Assert.True(both > 0);
        // Walking back onto the A column: R (base) on A–M, L closes on the column; both clamps FK-exact.
        var close = built.Keys.Single(k => k.Label == "walk (node) → anchor-A:L:Close anchor-A");
        var at = scan.ClampsAt(close.T, close.Q, close.GR, close.GL);
        Assert.Contains(at, c => c.Gripper == "R" && c.On == "placed-AM" && c.IsBase);
        var lc = Assert.Single(at, c => c.Gripper == "L" && c.On == "anchor-A" && !c.IsBase);
        var (pos, axis) = K.Residual(lc.Frame, BambooTrussLayout.StakeNamed("anchor-A").Grip);
        Assert.True(pos < K.GripPosTolM && axis < K.GripAxisTolDeg, $"L on anchor-A: {pos * 1000:F4} mm / {axis:F4}°");
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

    /// <summary>
    /// The shipped-plan key must not depend on last-ulp floating-point differences between desktop .NET and Blazor
    /// WASM (at e241fad the browser computed a member pitch of −29.999999999999993 vs −29.999999999999996 here, the
    /// fingerprint missed and the page planned live for minutes).
    /// </summary>
    [Fact]
    public void LayoutFingerprint_IsRobustToUlpDifferences()
    {
        Assert.Equal(BambooTrussMotion.FingerprintNumber(-29.999999999999996), BambooTrussMotion.FingerprintNumber(-29.999999999999993));
        Assert.Equal("-30", BambooTrussMotion.FingerprintNumber(-29.999999999999993));
        Assert.Equal(BambooTrussMotion.FingerprintNumber(0.19999999999999998), BambooTrussMotion.FingerprintNumber(0.2));
        Assert.Equal("0", BambooTrussMotion.FingerprintNumber(-1e-12));
        Assert.Equal("-0.3857", BambooTrussMotion.FingerprintNumber(-0.3857));
        // Every number the current layout puts into the key is unchanged by a few-ulp perturbation.
        foreach (var v in new[] { BambooTrussLayout.StrutLength, BambooTrussLayout.HalfSpan, BambooTrussLayout.Store.PitchDeg,
                     BambooTrussLayout.Store.YawDeg }.Concat(BambooTrussLayout.Members.SelectMany(m => new[] { m.Place.X, m.Place.Y, m.Place.Z, m.Place.PitchDeg, m.Place.YawDeg }))
                     .Concat(BambooTrussLayout.Stakes.SelectMany(t => new[] { t.X, t.Z, t.ClampY, t.Top, t.HeadingDeg })))
        {
            var up = v; var down = v;
            for (var k = 0; k < 4; k++) { up = Math.BitIncrement(up); down = Math.BitDecrement(down); }
            Assert.Equal(BambooTrussMotion.FingerprintNumber(v), BambooTrussMotion.FingerprintNumber(up));
            Assert.Equal(BambooTrussMotion.FingerprintNumber(v), BambooTrussMotion.FingerprintNumber(down));
        }
        // A real layout change (≥ 1 µm) still changes it.
        Assert.NotEqual(BambooTrussMotion.FingerprintNumber(-0.3857), BambooTrussMotion.FingerprintNumber(-0.385701));
        // No round-trip-formatted doubles left in the key.
        Assert.DoesNotContain("99999999", BambooTrussMotion.LayoutFingerprint);
        Assert.DoesNotContain("00000001", BambooTrussMotion.LayoutFingerprint);
    }

    /// <summary>
    /// The placed set only grows over the whole plan and is empty before AT1:Open: at 502034e the start/walk
    /// segments drew no stock and the viewer left a cane on A–B until baseL:Transit. Start/walk now carry the
    /// world "nothing placed, stock full", and Motus checks the walk against the stock.
    /// </summary>
    [Fact]
    public void PlacedSet_IsMonotonic_AndEmptyBeforeTheFirstOpen()
    {
        var built = Build();
        var scan = Scan(built);
        // The AT1:Open segment (jaws opening, cane already at its member) is the first with a placed cane.
        var openIdx = built.Keys.FindIndex(k => k.Label == "AT1:Open");
        var openT = built.Keys[openIdx - 1].T;
        var prev = new List<string>();
        foreach (var t in scan.SampleTimes)
        {
            var s = BambooTrussMotion.SampleState(built.Keys, t);
            var poses = scan.StrutPoses(t, s.Hold, s.Q).Select(p => p.Name).ToList();
            var placed = poses.Where(n => n.StartsWith("placed-", StringComparison.Ordinal)).ToList();
            foreach (var p in prev) Assert.Contains(p, placed);
            if (t <= openT + 1e-12) Assert.Empty(placed);
            // Until the last member is in hand, exactly one cane is on the stock or in hand.
            if (t <= openT + 1e-12) Assert.Single(poses, n => n == BambooIcdCell.Strut);
            prev = placed;
        }
        // Walk on triangle 1: the next stock cane and the placed triangle are Motus obstacles.
        var walk = built.Cycles.Single(c => c.MemberId == "walk (node) → AM@-1.00");
        var tw = built.Keys[walk.FirstKey + 1].T;
        Assert.Contains(scan.EnvironmentAt(tw, 0), o => o.Name == BambooIcdCell.Strut);
        foreach (var id in new[] { "AT1", "T1M", "AM" }) Assert.Contains(scan.EnvironmentAt(tw, 0), o => o.Name == "placed-" + id);
    }

    /// <summary>Each base swap is a visible rest pause in playback (both labels are shown for several frames).</summary>
    [Fact]
    public void BaseSwaps_AreVisiblePausesInPlayback()
    {
        var built = Build();
        foreach (var label in new[] { "walk (node) → AM@-1.00:swap base L→R", "walk (node) → AM@-1.00:swap base R→L" })
        {
            var i = built.Keys.FindIndex(k => k.Label == label);
            Assert.True(i > 0, label);
            Assert.True(built.Keys[i].T - built.Keys[i - 1].T >= 0.01, $"{label} lasts only {built.Keys[i].T - built.Keys[i - 1].T:F4}");
        }
        var play = new BambooTrussPlayback(built.Keys);
        var frames = new Dictionary<string, int>();
        while (!play.Done)
        {
            var l = BambooTrussMotion.SampleState(built.Keys, play.Advance(0.016)).Label;
            frames[l] = frames.GetValueOrDefault(l) + 1;
        }
        const string w = "walk (node) → AM@-1.00:";
        Assert.True(frames.GetValueOrDefault(w + "swap base L→R") >= 10, $"L→R shown {frames.GetValueOrDefault(w + "swap base L→R")} frames");
        Assert.True(frames.GetValueOrDefault(w + "swap base R→L") >= 10, $"R→L shown {frames.GetValueOrDefault(w + "swap base R→L")} frames");
    }

    /// <summary>The clamp marker for L on the A column is there as soon as its close completes (before the swap).</summary>
    [Fact]
    public void LClampMarker_ShowsWhenTheCloseCompletes()
    {
        var built = Build();
        var scan = Scan(built);
        const string label = "walk (node) → anchor-A:L:Close anchor-A";
        var close = built.Keys.Single(k => k.Label == label);
        var s = BambooTrussMotion.SampleState(built.Keys, close.T);
        Assert.Equal(label, s.Label);
        Assert.Contains(scan.ClampsAt(close.T, s.Q, s.GR, s.GL), c => c.Gripper == "L" && c.On == "anchor-A");
    }

    /// <summary>
    /// Brachiation onto the chord is attempted with an open-jaw side approach of the L-grip and every candidate is
    /// Motus-checked with the full L-grip / mount geometry. In the ICD URDF the L slot is bounded on both sides (L-grip
    /// body/rail at +y, base tube at −y) with only a 20 mm gap for the 14 mm cane, so no side approach exists and the
    /// planner says so. Any side-approach L clamp that does appear must be on a cane fixed at both ends at that time.
    /// </summary>
    [Fact]
    public void Brachiation_IsHonest_SideClampsOnlyOnDoubleFixedCanes()
    {
        var built = Build();
        var scan = Scan(built);
        foreach (var k in built.Keys.Where(k => k.Label.Contains(":L:Close placed-", StringComparison.Ordinal)))
        {
            var id = k.Label[(k.Label.IndexOf("placed-", StringComparison.Ordinal) + "placed-".Length)..];
            var placed = scan.World[BambooTrussMotion.CycleIndexAt(built.Keys, built.Cycles, k.T)].Placed.Select(p => p.Name["placed-".Length..]);
            Assert.Contains(id, BambooTrussLayout.FixedBothEnds(placed));
        }
        // Every member still missing after brachiation is reported, never silently dropped.
        foreach (var id in BambooTrussLayout.Members.Select(m => m.Id).Where(id => !built.Built.Contains(id)))
            Assert.Contains(built.Notes, n => n.StartsWith(id + ": not built", StringComparison.Ordinal) || n.StartsWith("brachiate", StringComparison.Ordinal));
    }

    /// <summary>
    /// Approved deviation from the ICD URDF: the L-grip is opened on one side (+y) to allow side clamping. The slot axis
    /// is the jaw z; the jaws slide along x. Everything except the jaw plates (carriage, rail, base tube, and the joint-1
    /// housing, whose y extent is invariant under the Y-axis joint 1) lies at y ≤ −r − 0.5 mm, so a cane fixed at both
    /// ends can enter the slot from +y. Joint 1 stays at (0, −0.03, 0): kinematics are unchanged.
    /// </summary>
    [Fact]
    public void LGrip_IsOpenOnPlusY_ForSideClamping()
    {
        var xml = System.Xml.Linq.XDocument.Load(UrdfPath);
        double[] V(string s) => s.Split(' ').Select(x => double.Parse(x, System.Globalization.CultureInfo.InvariantCulture)).ToArray();
        (double Lo, double Hi) Y(string name)
        {
            var c = xml.Descendants("collision").First(e => (string?)e.Attribute("name") == name);
            var y = V(c.Element("origin")!.Attribute("xyz")!.Value)[1];
            var h = V(c.Element("geometry")!.Element("box")!.Attribute("size")!.Value)[1] / 2;
            return (y - h, y + h);
        }
        var r = BambooIcdCell.StrutRadius;
        var j1 = V(xml.Descendants("joint").First(e => (string?)e.Attribute("name") == "joint_1").Element("origin")!.Attribute("xyz")!.Value);
        Assert.Equal(new[] { 0.0, -0.03, 0.0 }, j1);
        foreach (var name in new[] { "L-grip body", "L-grip rail", "base tube" })
            Assert.True(Y(name).Hi <= -r - 0.0005 + 1e-9, $"{name} reaches y {Y(name).Hi * 1000:F1} mm — must stay on the −y side of the slot");
        var t1 = Y("θ1");
        Assert.True(j1[1] + t1.Hi <= -r - 0.0005 + 1e-9, $"joint-1 housing reaches y {(j1[1] + t1.Hi) * 1000:F1} mm");
        // +y is open: only the jaw plates extend there, and they reach past the cane so it is held across its diameter.
        var mount = xml.Descendants("link").First(e => (string?)e.Attribute("name") == "mount");
        var plusY = mount.Elements("collision").Where(c => Y((string)c.Attribute("name")!).Hi > 0).Select(c => (string)c.Attribute("name")!).ToList();
        Assert.Equal(new[] { "L-grip jaw-", "L-grip jaw+" }, plusY);
        Assert.True(Y("L-grip jaw+").Hi >= r, "jaw plates cover the cane");
        Assert.Contains("DEVIATION", System.IO.File.ReadAllText(UrdfPath));
    }

    // ---------------- playback ----------------

    // ---------------- walking on the bamboo ----------------

    private static readonly Dictionary<string, BambooTrussMotion.TrussBuild> GaitPlans = new();

    /// <summary>Build triangle 1 from the A column, then one walk block with <paramref name="gait"/> to the target.</summary>
    private static BambooTrussMotion.TrussBuild GaitPlan(string gait, string target, double f)
    {
        var key = $"{gait}|{target}|{f}";
        lock (GaitPlans)
        {
            if (GaitPlans.TryGetValue(key, out var b)) return b;
            BambooTrussMotion.UseCell(Cell);
            var tasks = new List<BambooIcdMotion.TaskEdit>();
            foreach (var id in new[] { "AT1", "T1M", "AM" }) tasks.AddRange(BambooTrussMotion.DefaultTasks(BambooTrussLayout.Members.Where(m => m.Id == id).ToList()));
            tasks.Add(BambooTrussMotion.WalkTask(target, f, gait));
            return GaitPlans[key] = BambooTrussMotion.RunSequence(tasks);
        }
    }

    /// <summary>
    /// The shipped node-transfer gait walks from the A column onto triangle 1 as a <c>walk</c> task
    /// block: every step keeps one gripper clamped as the base on a stake or a cane fixed at both ends, swaps happen at
    /// rest with their own label, every clamp is FK-exact (0.5 mm / 0.5°) and the whole PTP path is Motus-clear.
    /// </summary>
    [Theory]
    [MemberData(nameof(GaitLegs))]
    public void Gait_WalksItsLeg_WithValidSteps_FkExact_MotusClear(string gait, string target, double f)
    {
        var built = GaitPlan(gait, target, f);
        Assert.True(string.IsNullOrEmpty(built.Error), built.Error);
        Assert.Equal(new[] { "AT1", "T1M", "AM" }, built.Built.ToArray());
        Assert.Contains(built.Notes, n => n.StartsWith($"walk ({gait}) → {target}", StringComparison.Ordinal) && n.Contains("steps —"));
        Assert.Contains(built.Cycles, c => c.MemberId.StartsWith($"walk ({gait}) → ", StringComparison.Ordinal));
        var keys = built.Keys;
        var scan = Scan(built);
        var placedIds = new[] { "AT1", "T1M", "AM" };
        var swaps = 0;
        for (var k = 1; k < keys.Count; k++)
        {
            var a = keys[k - 1]; var b = keys[k];
            if (a.Base != b.Base)
            {
                swaps++;
                Assert.True(K.JointDist(a.Q, b.Q) < 1e-9, $"{b.Label}: swap not at rest");
                Assert.EndsWith($"swap base {a.Base}→{b.Base}", b.Label);
            }
            else Assert.Equal(a.BaseWorld, b.BaseWorld);
            Assert.DoesNotContain("Approach", b.Label); // PTP only: no Cartesian approach / lower / slide
            Assert.DoesNotContain("Lower", b.Label);
            Assert.DoesNotContain("Slide", b.Label);
            var jaw = b.Base == "L" ? b.GL : b.GR;
            Assert.True(jaw <= Closed, $"{b.Label}: base not clamped");
            var on = b.BaseOn!;
            Assert.True(on.StartsWith("anchor-", StringComparison.Ordinal) ||
                        (on.StartsWith("placed-", StringComparison.Ordinal) && BambooTrussLayout.FixedBothEnds(placedIds).Contains(on["placed-".Length..])),
                $"{b.Label}: base on {on}");
            var (pos, axis, inRange) = OnObject(scan, b.T, on, b.BaseWorld!);
            Assert.True(pos < K.GripPosTolM && axis < K.GripAxisTolDeg && inRange, $"{b.Label}: base {pos * 1000:F4} mm / {axis:F4}° off {on}");
        }
        Assert.True(swaps >= 2);
        foreach (var g in built.Grips)
            Assert.True(g.PosM < K.GripPosTolM && g.AxisDeg < K.GripAxisTolDeg, $"{g.Label}: {g.PosM * 1000:F4} mm / {g.AxisDeg:F4}°");
        var full = Scan(built).RunToEnd();
        Assert.Null(full.TrackMiss);
        Assert.True(full.FirstHit is null, $"Motus hit: {full.FirstHit?.Describe()} ({full.FirstHit?.Label})");
        Assert.Equal(1.0, full.LastClearT, 9);
    }

    public static IEnumerable<object[]> GaitLegs() => new[]
    {
        new object[] { "node", "placed-AM", -1.0 },     // A column → onto the closed triangle 1 (node transfer)
    };

    [Fact]
    public void WalkBlock_IsAStringIdentityTask_WithGaitAndTargetPlane()
    {
        var t = BambooTrussMotion.WalkTask("placed-AM", 0.5, "swing");
        Assert.Equal("walk", t.Identity);
        Assert.Equal("placed-AM", t.Object);
        Assert.Equal("swing", t.Gait);
        Assert.Equal("target", Assert.Single(t.Planes).Name);
        Assert.Equal(0.5, BambooTrussMotion.WalkTargetF(t), 9);
        Assert.Equal(5, BambooWalk.Gaits.Select(g => g.Id).Distinct().Count());
        Assert.True(BambooTrussPlanner.Ptp);
    }

    /// <summary>Walk search primitives: deep checkpoints restore the full plan state for branching.</summary>
    [Fact]
    public void DeepCheckpoint_RestoresKeysCyclesAndPlacedCanes()
    {
        BambooTrussMotion.UseCell(Cell);
        var p = new BambooTrussPlanner(Cell);
        Assert.True(p.Start());
        var cp = p.SaveDeep();
        var n = p.Keys.Count;
        Assert.True(p.BuildMember(BambooTrussLayout.Members.Single(m => m.Id == "AT1")), p.Error);
        Assert.True(p.Keys.Count > n);
        var after = p.SaveDeep();
        p.RollbackDeep(cp);
        Assert.Equal(n, p.Keys.Count);
        Assert.Equal(BambooTrussLayout.S1.Name, p.LStake);
        Assert.Empty(p.PlacedIds);
        p.RollbackDeep(after);
        Assert.Equal(after.KeyCount, p.Keys.Count);
        Assert.Equal(new[] { "AT1" }, p.PlacedIds.ToArray());
    }

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
            // One Motus sample per frame: slower than playback, so Play must wait at the frontier at least once.
            if (!scan.Complete) scan.Step(1);
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
            BambooTrussLayout.IsSupport(o) || o.StartsWith("placed-", StringComparison.Ordinal));

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
