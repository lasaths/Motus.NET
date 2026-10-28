using System.Text.Json;

namespace Motus.Construction;

/// <summary>
/// Outcome log and learned ordering for <see cref="BambooBuildSearch.Options.Tamp"/>: one JSON line per planner
/// confirmation, and a confirm-rate prior read from earlier logs. The prior only reorders candidates; every choice is
/// still confirmed by the planner.
/// </summary>
internal static class BambooTampMemory
{
    /// <summary>One planner confirmation: what was tried, in which build state, and whether the planner kept it.</summary>
    internal sealed record Outcome(string Layout, string Member, string Mode, string? Relay, int Placed, string State,
        double Rank, bool Ok, double Seconds);

    /// <summary>Short stable id of the current layout (SHA-256 of the layout fingerprint, 12 hex digits).</summary>
    internal static string LayoutId()
    {
        var h = System.Security.Cryptography.SHA256.HashData(System.Text.Encoding.UTF8.GetBytes(BambooTrussMotion.LayoutFingerprint));
        return Convert.ToHexString(h, 0, 6).ToLowerInvariant();
    }

    internal static void Append(string path, Outcome o) => File.AppendAllText(path, JsonSerializer.Serialize(o) + "\n");

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
    internal static Func<string, BambooBuildSearch.Mode, double> Prior(IReadOnlyList<Outcome> log, string layout)
    {
        var byMode = log.GroupBy(o => o.Mode).ToDictionary(g => g.Key, g => (S: g.Count(o => o.Ok), N: g.Count()));
        var byMember = log.Where(o => o.Layout == layout).GroupBy(o => (o.Member, o.Mode))
            .ToDictionary(g => g.Key, g => (S: g.Count(o => o.Ok), N: g.Count()));
        return (member, mode) =>
        {
            var m = mode.ToString();
            var (sm, nm) = byMode.GetValueOrDefault(m);
            var pm = (sm + 1.0) / (nm + 2.0);
            var (s, n) = byMember.GetValueOrDefault((member, m));
            return -Math.Log((s + 2 * pm) / (n + 2.0));
        };
    }
}
