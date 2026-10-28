using Motus.Construction;

namespace Motus.Core.Tests;

/// <summary>Frame check used by physics-aware build sequencing (<see cref="BambooBuildSearch.Options.Stability"/>).</summary>
public class BambooFrameStabilityTests
{
    static BambooStructureGraph Cantilever(double length) => new(
        new[] { new BambooStructureGraph.Node("A", 0, 0, 0, true, "anchor-A"), new BambooStructureGraph.Node("B", length, 0, 0, false, null) },
        new[] { new BambooStructureGraph.Edge("AB", "A", "B") });

    [Fact]
    public void Cantilever_MatchesBeamTheory_ForSelfWeightAndTipLoad()
    {
        const double L = 0.24;
        var s = new BambooFrameStability.Settings { ElementsPerMember = 16 };
        double ro = s.OuterRadiusM, ri = ro - s.WallM;
        var I = Math.PI / 4 * (Math.Pow(ro, 4) - Math.Pow(ri, 4));
        var ei = s.YoungModulusPa * I;
        var w = s.MassPerMetre * 9.81;

        var self = BambooFrameStability.Check(Cantilever(L), new[] { "AB" }, s);
        Assert.True(self.Ok, self.Note);
        Assert.Equal(w * Math.Pow(L, 4) / (8 * ei), self.MaxDeflectionM, 0.03 * self.MaxDeflectionM);

        // Robot load: the worst interior node is the one next to the tip, a (1 − 1/16) L lever.
        var robot = BambooFrameStability.Check(Cantilever(L), new[] { "AB" }, s, "AB");
        var a = L * 15 / 16;
        var p = s.RobotMassKg * 9.81;
        var expected = p * a * a * (3 * L - a) / (6 * ei) + w * Math.Pow(L, 4) / (8 * ei);
        Assert.Equal(expected, robot.MaxDeflectionM, 0.03 * expected);
        Assert.Equal(p * a * ro / I + w * L * L / 2 * ro / I, robot.MaxStressPa, 0.05 * robot.MaxStressPa);
    }

    [Fact]
    public void Limits_AndMissingSupport_Reject()
    {
        var tight = BambooFrameStability.Check(Cantilever(0.24), new[] { "AB" }, new BambooFrameStability.Settings { MaxDeflectionM = 1e-4 }, "AB");
        Assert.False(tight.Ok);
        Assert.Contains("sag over", tight.Note);

        var floating = new BambooStructureGraph(
            new[] { new BambooStructureGraph.Node("A", 0, 0, 0, true, "anchor-A"), new BambooStructureGraph.Node("B", 1, 0, 0, false, null),
                    new BambooStructureGraph.Node("C", 2, 0, 0, false, null) },
            new[] { new BambooStructureGraph.Edge("BC", "B", "C") });
        var r = BambooFrameStability.Check(floating, new[] { "BC" });
        Assert.False(r.Ok);
        Assert.Contains("not connected to an anchor", r.Note);
        Assert.True(BambooFrameStability.Check(floating, Array.Empty<string>()).Ok);
        Assert.Throws<ArgumentOutOfRangeException>(() => BambooFrameStability.Check(floating, new[] { "BC" }, new BambooFrameStability.Settings { WallM = 0 }));
    }

    [Fact]
    public void ClosedTriangle_IsStifferThanTheOpenChain()
    {
        var g = new BambooStructureGraph(
            new[] { new BambooStructureGraph.Node("A", 0, 0, 0, true, "anchor-A"), new BambooStructureGraph.Node("M", 0.27, 0, 0, true, "anchor-M"),
                    new BambooStructureGraph.Node("T", 0.135, 0.234, 0, false, null) },
            new[] { new BambooStructureGraph.Edge("AT", "A", "T"), new BambooStructureGraph.Edge("TM", "T", "M"), new BambooStructureGraph.Edge("AM", "A", "M") });
        var open = BambooFrameStability.Check(g, new[] { "AT" }, null, "AT");
        var closed = BambooFrameStability.Check(g, new[] { "AT", "TM" }, null, "AT");
        Assert.True(closed.MaxDeflectionM < open.MaxDeflectionM / 10, $"{closed.Note} vs {open.Note}");
    }
}
