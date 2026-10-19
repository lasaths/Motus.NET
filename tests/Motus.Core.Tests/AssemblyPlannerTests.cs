using Motus.Construction;
using Xunit.Abstractions;

namespace Motus.Core.Tests;

public class AssemblyPlannerTests
{
    private readonly ITestOutputHelper _out;
    public AssemblyPlannerTests(ITestOutputHelper output) => _out = output;

    static double Panel => SiteDefaults.Chosen.Length + 2 * SiteDefaults.Chosen.Gap;

    /// <summary>Non-Warren input: one triangle A–M–T on the A column (3 members), the D column stays as a clamp support.</summary>
    static BambooTrussLayout.StructureDefinition Triangle()
    {
        var p = Panel; var y = SiteDefaults.Chosen.NodeY; var z = BambooTrussLayout.PlaneZ;
        return new(
            [new("A", -p, y, z), new("M", 0, y, z), new("T", -p / 2, y + p * Math.Sqrt(3) / 2, z)],
            [new("AT", "diagonal A–T", "/", "A", "T"), new("TM", "diagonal T–M", "\\", "T", "M"), new("AM", "bottom chord A–M", "_", "A", "M")]);
    }

    [Fact]
    public void WarrenExample_StructureMatchesLayoutDefault()
    {
        var ex = AssemblyPlanner.WarrenTrussExample.Structure!;
        Assert.Equal(BambooTrussLayout.FootNodes.Select(n => (n.Name, n.X, n.Y, n.Z)), ex.Nodes.Select(n => (n.Name, n.X, n.Y, n.Z)));
        Assert.Equal(BambooTrussLayout.Members.Select(m => m.Id), ex.Members.Select(m => m.Id));
        Assert.Equal(new[] { "A", "D" }, BambooTrussLayout.SupportNodes);
    }

    [Fact]
    public void WarrenExample_ShippedPlan_IsCollisionFree()
    {
        var plan = AssemblyPlanner.Plan(AssemblyPlanner.WarrenTrussExample with { UseShippedPlan = true });
        Assert.True(plan.CollisionFree, plan.Verdict);
        Assert.Equal(plan.Members.Select(m => m.Id).OrderBy(x => x), plan.Build.Built.OrderBy(x => x));
        Assert.NotEmpty(plan.Keys);
        Assert.Equal(plan.Build.Tasks.Count, plan.Tasks.Count);
    }

    /// <summary>Scene generation from a non-Warren input: ground, relay frames, per-member lengths, obstacles.</summary>
    [Fact]
    public void SceneGeneration_FollowsTheStructureInput()
    {
        var old = BambooTrussLayout.Current;
        try
        {
            // Warren: relay frames are the truss plane (N = +z, X = +x, Y = +y) at every node.
            foreach (var n in BambooTrussLayout.FootNodes)
                Assert.Equal(((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)), BambooTrussLayout.NodePlane(n.Name));

            // A triangle standing in the plane x = −0.3 (normal ±x): relay "0" points along world +y? no: +z (x projected out).
            var p = Panel; var y = SiteDefaults.Chosen.NodeY;
            var tilted = new BambooTrussLayout.StructureDefinition(
                [new("A", -p, y, 0.11), new("B", -p, y, 0.11 + p), new("C", -p, y + p, 0.11 + p / 2)],
                [new("AB", "AB", "_", "A", "B"), new("BC", "BC", "/", "B", "C", 0.20), new("CA", "CA", "\\", "C", "A")]);
            BambooTrussLayout.Use(SiteDefaults.Chosen with { Structure = tilted });
            var (nrm, u, v) = BambooTrussLayout.NodePlane("A");
            Assert.Equal((1.0, 0.0, 0.0), nrm);
            Assert.Equal((0.0, 1.0, 0.0), u);
            Assert.Equal((0.0, 0.0, 1.0), v);
            Assert.Equal(nrm, BambooTrussLayout.RelayDir(BambooTrussLayout.RelayId("AB", "A", "z+")));
            var d90 = BambooTrussLayout.RelayDir(BambooTrussLayout.RelayId("AB", "A", "90"));
            Assert.Equal(0, d90.X, 12); Assert.Equal(0, d90.Y, 12); Assert.Equal(1, d90.Z, 12);

            // Per-member length: default for AB / CA, 0.20 m for BC (and its relay); used for stock grasp offsets.
            Assert.Equal(SiteDefaults.Chosen.Length, BambooTrussLayout.LengthOf("AB"));
            Assert.Equal(0.20, BambooTrussLayout.LengthOf("placed-BC"));
            Assert.Equal(0.20, BambooTrussLayout.Def(BambooTrussLayout.RelayId("BC", "A", "z+"))!.Length);
            Assert.Equal(0.20 / 2 - 0.025, BambooTrussLayout.StockGraspOffsetsFor(0.20)[1], 3);
            Assert.Contains("lengths:", BambooTrussMotion.LayoutFingerprint);

            // Only the A support is a structure node: only its connector is a fixture.
            Assert.Equal(new[] { "A" }, BambooTrussLayout.SupportNodes);
            Assert.Contains(BambooTrussLayout.FixtureObjects, o => o.Name == "hub-A");
            Assert.DoesNotContain(BambooTrussLayout.FixtureObjects, o => o.Name == "hub-D");

            // Ground derived from the supports; explicit ground and obstacles replace / extend it.
            var span = SiteDefaults.Make(0.24, 0.03, 0.19, 0.09, 60, -0.06, 0.32) with { Stakes = [SiteDefaults.Anchor("anchor-A", -0.5, 0.19, 0.09, 60), SiteDefaults.Anchor("anchor-D", 0.1, 0.19, 0.09, 120)] };
            BambooTrussLayout.Use(span);
            var floor = BambooTrussLayout.PitBoxes.Single(b => b.Name == "pit-floor");
            Assert.Equal(-0.2, floor.X, 12);
            Assert.Equal(0.3 - BambooTrussLayout.PitEdgeInset + BambooTrussLayout.EarthWall, floor.HalfX, 12);
            BambooTrussLayout.Use(span with { PitDepth = 0 });
            Assert.Empty(BambooTrussLayout.Pads);
            Assert.Empty(BambooTrussLayout.PitBoxes);
            var box = BambooIcdCell.BoxAt("wall", 0.4, 0.1, 0.4, 0.05, 0.1, 0.05);
            BambooTrussLayout.Use(span with { Obstacles = [box], Ground = new([("deck", 0, 0, 1, 1)], []) });
            Assert.Equal("deck", BambooTrussLayout.Pads.Single().Name);
            Assert.Contains(BambooTrussLayout.FixtureObjects, o => o.Name == "wall");
            Assert.Contains("obstacles:wall", BambooTrussMotion.LayoutFingerprint);
        }
        finally { BambooTrussLayout.Use(old); }
    }

    /// <summary>Non-Warren structure: a single triangle (3 members) plans live to completion and replays collision-free.</summary>
    [Fact]
    public void Triangle_PlansToCompletion_CollisionFree()
    {
        var oldFp = BambooTrussMotion.LayoutFingerprint;
        var plan = AssemblyPlanner.Plan(new AssemblyPlanRequest { Structure = Triangle() });
        _out.WriteLine($"verdict: {plan.Verdict}; built {string.Join(",", plan.Build.Built)}; error '{plan.Build.Error}'; source {BambooTrussMotion.ShippedTasksSource}");
        foreach (var n in plan.Build.Notes.Take(40)) _out.WriteLine(n);
        Assert.Equal("", plan.Build.Error);
        Assert.Equal(new[] { "AM", "AT", "TM" }, plan.Build.Built.OrderBy(x => x, StringComparer.Ordinal));
        Assert.True(plan.CollisionFree, plan.Verdict);
        Assert.Equal(oldFp, BambooTrussMotion.LayoutFingerprint); // state restored
    }
}
