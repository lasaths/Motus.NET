using System.Text.Json;

namespace Motus.Construction.Search;

/// <summary>
/// Outcome log and learned ordering for a search (<see cref="BambooBuildSearch.Options.Tamp"/>): one JSON line per planner
/// confirmation, and a confirm-rate prior read from earlier logs. The prior only reorders candidates; every choice is
/// still confirmed by the planner.
/// </summary>
internal static class OutcomeLog
{
    /// <summary>
    /// One planner confirmation: what was tried, in which build state, and whether the planner kept it. Trailing fields
    /// (null in old logs): <see cref="Stand"/> = stand origin (x, y, z) in metres, <see cref="F"/> = layout-free features
    /// (<see cref="FeatureVersion"/>, see BambooFrontier.Features), <see cref="Node"/> = search expansion id,
    /// <see cref="Pos"/> = position in that node's sorted order.
    /// </summary>
    internal sealed record Outcome(string Layout, string Member, string Mode, string? Relay, int Placed, string State,
        double Rank, bool Ok, double Seconds, double[]? Stand = null, double[]? F = null, int? FeatureVersion = null,
        int? Node = null, int? Pos = null);

    /// <summary>
    /// One sorted candidate at a node, written before any try (tried or not). <see cref="Ok"/> is always null, so
    /// <see cref="Read"/> (bool Ok) skips these rows and the confirm-rate prior never counts them. Untried = no
    /// <see cref="Outcome"/> row with the same Node and Pos.
    /// </summary>
    internal sealed record Sorted(string Layout, string Member, string Mode, string? Relay, int Placed, string State,
        double Rank, bool? Ok, double[]? Stand, double[]? F, int? FeatureVersion, int Node, int Pos);

    /// <summary>Short stable id of a layout (SHA-256 of its fingerprint, 12 hex digits).</summary>
    internal static string LayoutId(string fingerprint)
    {
        var h = System.Security.Cryptography.SHA256.HashData(System.Text.Encoding.UTF8.GetBytes(fingerprint));
        return Convert.ToHexString(h, 0, 6).ToLowerInvariant();
    }

    internal static void Append<TRow>(string path, TRow o) => File.AppendAllText(path, JsonSerializer.Serialize(o) + "\n");

    internal static void AppendAll<TRow>(string path, IEnumerable<TRow> rows) =>
        File.AppendAllLines(path, rows.Select(o => JsonSerializer.Serialize(o)));

    /// <summary>Outcomes in the given JSONL files; missing files and unreadable lines are skipped.</summary>
    internal static List<Outcome> Read(IEnumerable<string> paths)
    {
        var list = new List<Outcome>();
        foreach (var path in paths)
        {
            if (!File.Exists(path)) continue;
            foreach (var line in File.ReadLines(path))
            {
                try { if (JsonSerializer.Deserialize<Outcome>(line) is { } o) list.Add(o); }
                catch (JsonException) { }
            }
        }
        return list;
    }

    /// <summary>
    /// Rank penalty −ln p̂ for trying a member by a mode. p̂_mode = (s + 1) / (n + 2)
    /// over every logged outcome of that mode (Laplace); p̂ = (s + 2·p̂_mode) / (n + 2) over the outcomes of this member and
    /// mode on this layout, i.e. shrunk toward the mode rate when there is little data. No data gives −ln ½ for both.
    /// </summary>
    internal static Func<string, string, double> Prior(IReadOnlyList<Outcome> log, string layout)
    {
        var byMode = log.GroupBy(o => o.Mode).ToDictionary(g => g.Key, g => (S: g.Count(o => o.Ok), N: g.Count()));
        var byMember = log.Where(o => o.Layout == layout).GroupBy(o => (o.Member, o.Mode))
            .ToDictionary(g => g.Key, g => (S: g.Count(o => o.Ok), N: g.Count()));
        return (member, mode) =>
        {
            var (sm, nm) = byMode.GetValueOrDefault(mode);
            var pm = (sm + 1.0) / (nm + 2.0);
            var (s, n) = byMember.GetValueOrDefault((member, mode));
            return -Math.Log((s + 2 * pm) / (n + 2.0));
        };
    }
}
