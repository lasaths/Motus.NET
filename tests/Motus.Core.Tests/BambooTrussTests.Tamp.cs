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
            var outcomes = Motus.Construction.Search.OutcomeLog.Read(new[] { logPath });
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

    /// <summary>
    /// Frontier ranker step 1 (log only): the outcome log does not change the search. With the log on, the Warren search
    /// gives the same tasks, counts and log lines as with it off. Outcome rows carry Stand, F, FeatureVersion, Node and
    /// Pos; every node writes its sorted candidates (Ok null) before any try; with the log off no file is written.
    /// </summary>
    [Fact]
    public void BuildSearch_OutcomeLog_DoesNotChangeTheSearch_AndLogsSortedCandidates()
    {
        BambooTrussMotion.UseCell(Cell);
        var logPath = Path.Combine(Path.GetTempPath(), $"bamboo-ranker-{Guid.NewGuid():N}.jsonl");
        try
        {
            var off = BambooBuildSearch.Run(Cell, new BambooBuildSearch.Options { Tamp = new BambooBuildSearch.TampSettings() }, _out.WriteLine);
            Assert.False(File.Exists(logPath));
            var on = BambooBuildSearch.Run(Cell, new BambooBuildSearch.Options { Tamp = new BambooBuildSearch.TampSettings { OutcomeLogPath = logPath } }, _out.WriteLine);
            Assert.True(off.Complete, string.Join("\n", off.Log));
            Assert.Equal(System.Text.Json.JsonSerializer.Serialize(off.Tasks), System.Text.Json.JsonSerializer.Serialize(on.Tasks));
            Assert.Equal((off.Confirmed, off.Rejected, off.Deferred), (on.Confirmed, on.Rejected, on.Deferred));
            static IEnumerable<string> NoTimes(IEnumerable<string> log) => log.Select(l => System.Text.RegularExpressions.Regex.Replace(l, @"\d+(\.\d+)? s\b", "# s"));
            Assert.Equal(NoTimes(off.Log), NoTimes(on.Log));

            var rows = File.ReadLines(logPath).Select(l => System.Text.Json.JsonDocument.Parse(l).RootElement).ToList();
            var tried = rows.Where(r => r.GetProperty("Ok").ValueKind != System.Text.Json.JsonValueKind.Null).ToList();
            var sorted = rows.Where(r => r.GetProperty("Ok").ValueKind == System.Text.Json.JsonValueKind.Null).ToList();
            Assert.Equal(on.Attempts, tried.Count);
            Assert.NotEmpty(sorted);
            foreach (var r in rows)
            {
                Assert.Equal(BambooFrontier.FeatureVersion, r.GetProperty("FeatureVersion").GetInt32());
                Assert.Equal(12, r.GetProperty("F").GetArrayLength());
                Assert.Equal(3, r.GetProperty("Stand").GetArrayLength());
                Assert.True(r.GetProperty("Pos").GetInt32() >= 0);
            }
            (int, int) At(System.Text.Json.JsonElement r) => (r.GetProperty("Node").GetInt32(), r.GetProperty("Pos").GetInt32());
            var byAt = sorted.ToDictionary(At);
            // Each confirmation is the sorted candidate at its node and position, with the same rank and features.
            foreach (var t in tried)
            {
                var c = byAt[At(t)];
                Assert.Equal((c.GetProperty("Member").GetString(), c.GetProperty("Mode").GetString(), c.GetProperty("Rank").GetDouble()),
                    (t.GetProperty("Member").GetString(), t.GetProperty("Mode").GetString(), t.GetProperty("Rank").GetDouble()));
                Assert.Equal(c.GetProperty("F").GetRawText(), t.GetProperty("F").GetRawText());
            }
            // No resume: each node is sorted by rank.
            foreach (var g in sorted.GroupBy(r => r.GetProperty("Node").GetInt32()))
                Assert.Equal(g.Select(r => r.GetProperty("Rank").GetDouble()).Order(), g.OrderBy(r => r.GetProperty("Pos").GetInt32()).Select(r => r.GetProperty("Rank").GetDouble()));
            _out.WriteLine($"ranker log: {tried.Count} confirmations, {sorted.Count} sorted candidates at {byAt.Keys.Select(k => k.Item1).Distinct().Count()} nodes");
        }
        finally { File.Delete(logPath); }
    }

    /// <summary>The prior is Laplace-smoothed per mode and shrunk toward it per member: more failures rank later, no data is neutral.</summary>
    [Fact]
    public void TampPrior_OrdersByTheSmoothedConfirmRate()
    {
        Motus.Construction.Search.OutcomeLog.Outcome O(string member, string mode, bool ok, string layout = "L") => new(layout, member, mode, null, 0, "", 0, ok, 0);
        var log = new List<Motus.Construction.Search.OutcomeLog.Outcome>
        {
            O("A", "Direct", false), O("A", "Direct", false), O("B", "Direct", true), O("B", "Direct", true),
            O("C", "Direct", false, layout: "other"), O("A", "Land", true),
        };
        var prior = Motus.Construction.Search.OutcomeLog.Prior(log, "L");
        Assert.True(prior("A", "Direct") > prior("B", "Direct"));
        // Mode rate over every layout: Direct 2/5 → p̂ = 3/7; an unseen member gets −ln(2·(3/7)/2).
        Assert.Equal(-Math.Log(3.0 / 7), prior("Z", "Direct"), 12);
        // Member C only failed on another layout: on this layout it is unseen.
        Assert.Equal(prior("Z", "Direct"), prior("C", "Direct"), 12);
        Assert.Equal(-Math.Log(0.5), Motus.Construction.Search.OutcomeLog.Prior(new List<Motus.Construction.Search.OutcomeLog.Outcome>(), "L")("A", "Relay"), 12);
        Assert.Throws<ArgumentOutOfRangeException>(() => BambooBuildSearch.Run(Cell, new BambooBuildSearch.Options { Tamp = new BambooBuildSearch.TampSettings { PriorWeight = -1 } }));
    }
}
