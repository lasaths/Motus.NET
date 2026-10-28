using Motus.Construction;

namespace Motus.Core.Tests;

public partial class BambooTrussTests
{
    /// <summary>
    /// TAMP aids on the Warren example: lazy preconditions and failure memory still derive the shipped, planner-confirmed
    /// sequence; the outcome log has one line per confirmation; a second run ordered by that log's prior also completes.
    /// </summary>
    [Fact]
    public void BuildSearch_WithTamp_StillDerivesTheShippedSequence_AndLogsEveryConfirmation()
    {
        BambooTrussMotion.UseCell(Cell);
        var logPath = Path.Combine(Path.GetTempPath(), $"bamboo-tamp-{Guid.NewGuid():N}.jsonl");
        try
        {
            var r = BambooBuildSearch.Run(Cell, new BambooBuildSearch.Options { Tamp = new BambooBuildSearch.TampSettings { OutcomeLogPath = logPath } }, _out.WriteLine);
            Assert.True(r.Complete, string.Join("\n", r.Log));
            Assert.Equal(System.Text.Json.JsonSerializer.Serialize(BambooTrussMotion.ShippedTasks()), System.Text.Json.JsonSerializer.Serialize(r.Tasks));
            var outcomes = BambooTampMemory.Read(new[] { logPath });
            Assert.Equal(r.Attempts, outcomes.Count);
            Assert.Equal(r.Confirmed, outcomes.Count(o => o.Ok));

            var again = BambooBuildSearch.Run(Cell, new BambooBuildSearch.Options { Tamp = new BambooBuildSearch.TampSettings { PriorLogPaths = new[] { logPath } } }, _out.WriteLine);
            Assert.True(again.Complete, string.Join("\n", again.Log));
            Assert.Contains(again.Log, l => l.StartsWith("tamp: candidates ordered", StringComparison.Ordinal));
            Assert.True(again.Rejected <= r.Rejected, $"with prior {again.Rejected} rejected, without {r.Rejected}");
            _out.WriteLine($"tamp search {r.Seconds:F0} s ({r.Rejected} rejected); with prior {again.Seconds:F0} s ({again.Rejected} rejected)");
        }
        finally { File.Delete(logPath); }
    }

    /// <summary>The prior is Laplace-smoothed per mode and shrunk toward it per member: more failures rank later, no data is neutral.</summary>
    [Fact]
    public void TampPrior_OrdersByTheSmoothedConfirmRate()
    {
        BambooTampMemory.Outcome O(string member, string mode, bool ok, string layout = "L") => new(layout, member, mode, null, 0, "", 0, ok, 0);
        var log = new List<BambooTampMemory.Outcome>
        {
            O("A", "Direct", false), O("A", "Direct", false), O("B", "Direct", true), O("B", "Direct", true),
            O("C", "Direct", false, layout: "other"), O("A", "Land", true),
        };
        var prior = BambooTampMemory.Prior(log, "L");
        Assert.True(prior("A", BambooBuildSearch.Mode.Direct) > prior("B", BambooBuildSearch.Mode.Direct));
        // Mode rate over every layout: Direct 2/5 → p̂ = 3/7; an unseen member gets −ln(2·(3/7)/2).
        Assert.Equal(-Math.Log(3.0 / 7), prior("Z", BambooBuildSearch.Mode.Direct), 12);
        // Member C only failed on another layout: on this layout it is unseen.
        Assert.Equal(prior("Z", BambooBuildSearch.Mode.Direct), prior("C", BambooBuildSearch.Mode.Direct), 12);
        Assert.Equal(-Math.Log(0.5), BambooTampMemory.Prior(new List<BambooTampMemory.Outcome>(), "L")("A", BambooBuildSearch.Mode.Relay), 12);
        Assert.Throws<ArgumentOutOfRangeException>(() => BambooBuildSearch.Run(Cell, new BambooBuildSearch.Options { Tamp = new BambooBuildSearch.TampSettings { PriorWeight = -1 } }));
    }
}
