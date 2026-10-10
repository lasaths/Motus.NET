namespace Motus.Viewer.Services;

/// <summary>
/// Task blocks for the /bamboo-truss bottom timeline: one block per pick / place / walk / park-relay / swap, with its
/// motion range (0..1, the same time as the motion slider) and a short label.
/// </summary>
public static class BambooTrussTimeline
{
    public sealed record Block(string Label, string Kind, double T0, double T1);

    /// <summary>"AT1" → "A–T1"; relay ids "T2D>M" → "T2–D @M".</summary>
    public static string Pretty(string id)
    {
        var park = id.Split('>');
        var m = BambooTrussLayout.Members.FirstOrDefault(x => x.Id == park[0]);
        var name = m is null ? park[0] : $"{m.NodeA}–{m.NodeB}";
        return park.Length > 1 ? $"{name} @{park[1]}" : name;
    }

    public static List<Block> Blocks(IReadOnlyList<BambooIcdMotion.MotionKey> keys, IReadOnlyList<BambooTrussMotion.CycleBound> cycles)
    {
        var list = new List<Block>();
        foreach (var c in cycles)
        {
            if (c.LastKey < c.FirstKey || c.T1 - c.T0 < 1e-9) continue;
            double T(int k) => k <= 0 ? 0 : keys[k - 1].T; // key k's segment starts at key k − 1
            var id = c.MemberId;
            if (id.StartsWith("walk", StringComparison.Ordinal) || id.StartsWith("land", StringComparison.Ordinal))
            {
                // A walk block, split at its base swaps (each swap a thin block of its own).
                var name = id.Contains('→') ? id[(id.IndexOf('→') + 1)..].Trim() : id;
                var pretty = id.StartsWith("walk (land)", StringComparison.Ordinal) ? "Walk for " + Pretty(name) : "Walk → " + PrettyTarget(name);
                var start = c.FirstKey;
                for (var k = c.FirstKey; k <= c.LastKey; k++)
                {
                    if (!keys[k].Label.Contains("swap base", StringComparison.Ordinal)) continue;
                    if (k > start) list.Add(new Block(pretty, "walk", T(start), T(k)));
                    list.Add(new Block(keys[k].Label[(keys[k].Label.LastIndexOf(':') + 1)..], "swap", T(k), keys[k].T));
                    start = k + 1;
                }
                if (start <= c.LastKey) list.Add(new Block(pretty, "walk", T(start), c.T1));
                continue;
            }
            if (id == "start") continue;
            var isPark = id.Contains('>');
            var label = Pretty(id);
            var close = -1; var open = -1;
            for (var k = c.FirstKey; k <= c.LastKey; k++)
            {
                var l = keys[k].Label;
                if (close < 0 && l.EndsWith(":Close", StringComparison.Ordinal)) close = k;
                if (open < 0 && l.EndsWith(":Open", StringComparison.Ordinal) && keys[k].Hold == 2) open = k;
            }
            if (close < 0 || open < 0) { list.Add(new Block(label, isPark ? "park" : "pick", c.T0, c.T1)); continue; }
            var fromRelay = BambooTrussLayout.Members.Any(m => m.Id == id) && cycles.Any(x => x.MemberId.StartsWith(id + ">", StringComparison.Ordinal));
            list.Add(new Block($"{label} {(fromRelay ? "re-grasp" : "pick")}", fromRelay ? "park" : "pick", c.T0, keys[close].T));
            list.Add(new Block($"{label} {(isPark ? "park" : "place")}", isPark ? "park" : "place", keys[close].T, keys[open].T));
            list.Add(new Block($"{label} retreat", "retreat", keys[open].T, c.T1));
        }
        return list;
    }

    static string PrettyTarget(string s)
    {
        var at = s.IndexOf('@');
        var obj = at >= 0 ? s[..at] : s;
        if (obj.StartsWith("placed-", StringComparison.Ordinal)) obj = obj["placed-".Length..];
        return BambooTrussLayout.Members.Any(m => m.Id == obj) ? Pretty(obj) : obj;
    }
}
