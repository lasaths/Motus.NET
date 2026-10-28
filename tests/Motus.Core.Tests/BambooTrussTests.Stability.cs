using Motus.Construction;

namespace Motus.Core.Tests;

public partial class BambooTrussTests
{
    /// <summary>
    /// Physics-aware sequencing on the Warren example: with the frame check on (default settings) every partial
    /// structure stands, the search still completes 7/7 and derives the same planner-confirmed sequence.
    /// </summary>
    [Fact]
    public void BuildSearch_WithStabilityCheck_StillDerivesTheShippedSequence()
    {
        BambooTrussMotion.UseCell(Cell);
        var r = BambooBuildSearch.Run(Cell, new BambooBuildSearch.Options { Stability = new BambooFrameStability.Settings() }, _out.WriteLine);
        Assert.True(r.Complete, string.Join("\n", r.Log));
        Assert.DoesNotContain(r.Log, l => l.Contains("stability check", StringComparison.Ordinal));
        Assert.Equal(System.Text.Json.JsonSerializer.Serialize(BambooTrussMotion.ShippedTasks()), System.Text.Json.JsonSerializer.Serialize(r.Tasks));
        var graph = BambooStructureGraph.FromLayout();
        for (var n = 1; n <= r.Order.Count; n++)
        {
            var report = BambooFrameStability.Check(graph, r.Order.Take(n).Select(c => c.MemberId));
            _out.WriteLine($"{n}/7 {report.Note}");
            Assert.True(report.Ok, report.Note);
        }
    }

    /// <summary>An over-tight sag limit makes the search report the stability check instead of planning anyway.</summary>
    [Fact]
    public void BuildSearch_StabilityCheck_DropsStatesOverTheLimit()
    {
        BambooTrussMotion.UseCell(Cell);
        var r = BambooBuildSearch.Run(Cell, new BambooBuildSearch.Options
        { Stability = new BambooFrameStability.Settings { MaxDeflectionM = 1e-9 }, MaxRejected = 2, MaxConfirmations = 4 }, _out.WriteLine);
        Assert.False(r.Complete);
        Assert.Equal(0, r.Confirmed);
        Assert.Contains(r.Log, l => l.Contains("dropped by the stability check", StringComparison.Ordinal));
    }
}
