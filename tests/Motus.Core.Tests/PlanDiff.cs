using System.Globalization;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;

namespace Motus.Core.Tests;

/// <summary>
/// Tolerant structural diff of two serialized plans: same JSON shape, same strings (numbers embedded in strings,
/// e.g. "-0.080 m" in notes, compared numerically), same discrete values, numbers within <see cref="Tol"/>.
/// </summary>
public static partial class PlanDiff
{
    /// <summary>Absolute tolerance for plan numbers (degrees, metres, normalised time, mm).</summary>
    public const double Tol = 1e-6;

    /// <summary>Counters that are not decisions (number of collision checks): relative tolerance.</summary>
    static readonly HashSet<string> Counters = new(StringComparer.Ordinal) { "MotusChecks" };
    const double CounterRelTol = 1e-3;

    [GeneratedRegex(@"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?")]
    private static partial Regex Number();

    public static List<string> Compare(string expectedJson, string actualJson)
    {
        using var a = JsonDocument.Parse(expectedJson);
        using var b = JsonDocument.Parse(actualJson);
        var diffs = new List<string>();
        Walk(a.RootElement, b.RootElement, "$", diffs, b.RootElement);
        return diffs;
    }

    static string KeyLabel(JsonElement root, string path)
    {
        var m = Regex.Match(path, @"^\$\.Keys\[(\d+)\]");
        if (!m.Success || !root.TryGetProperty("Keys", out var keys)) return "";
        var i = int.Parse(m.Groups[1].Value, CultureInfo.InvariantCulture);
        return i < keys.GetArrayLength() && keys[i].TryGetProperty("Label", out var l) ? $" [live key '{l.GetString()}']" : "";
    }

    static void Walk(JsonElement a, JsonElement b, string path, List<string> diffs, JsonElement liveRoot)
    {
        if (diffs.Count > 200) return;
        if (a.ValueKind != b.ValueKind)
        {
            diffs.Add($"{path}: kind {a.ValueKind} vs {b.ValueKind}{KeyLabel(liveRoot, path)}");
            return;
        }
        switch (a.ValueKind)
        {
            case JsonValueKind.Object:
                var an = a.EnumerateObject().Select(p => p.Name).ToList();
                var bn = b.EnumerateObject().Select(p => p.Name).ToList();
                if (!an.SequenceEqual(bn)) { diffs.Add($"{path}: properties [{string.Join(",", an)}] vs [{string.Join(",", bn)}]"); return; }
                foreach (var n in an)
                {
                    var pa = a.GetProperty(n); var pb = b.GetProperty(n);
                    if (Counters.Contains(n) && pa.ValueKind == JsonValueKind.Number && pb.ValueKind == JsonValueKind.Number)
                    {
                        double x = pa.GetDouble(), y = pb.GetDouble();
                        if (Math.Abs(x - y) > CounterRelTol * Math.Max(1, Math.Abs(x))) diffs.Add($"{path}.{n}: {x} vs {y}");
                        continue;
                    }
                    Walk(pa, pb, $"{path}.{n}", diffs, liveRoot);
                }
                break;
            case JsonValueKind.Array:
                int la = a.GetArrayLength(), lb = b.GetArrayLength();
                if (la != lb) diffs.Add($"{path}: length {la} vs {lb}");
                for (var i = 0; i < Math.Min(la, lb); i++) Walk(a[i], b[i], $"{path}[{i}]", diffs, liveRoot);
                break;
            case JsonValueKind.Number:
                if (!Close(a.GetDouble(), b.GetDouble()))
                    diffs.Add($"{path}: {a.GetRawText()} vs {b.GetRawText()} (Δ {b.GetDouble() - a.GetDouble():G3}){KeyLabel(liveRoot, path)}");
                break;
            case JsonValueKind.String:
                if (!StringsMatch(a.GetString()!, b.GetString()!))
                    diffs.Add($"{path}: \"{a.GetString()}\" vs \"{b.GetString()}\"{KeyLabel(liveRoot, path)}");
                break;
            default:
                if (a.GetRawText() != b.GetRawText()) diffs.Add($"{path}: {a.GetRawText()} vs {b.GetRawText()}");
                break;
        }
    }

    static bool Close(double x, double y) => Math.Abs(x - y) <= Tol;

    /// <summary>Equal text outside numbers; embedded numbers equal within one unit of their last printed digit
    /// (they are rounded for display, so a 1e-12 difference can flip the last digit) or <see cref="Tol"/>.</summary>
    public static bool StringsMatch(string x, string y)
    {
        if (x == y) return true;
        var mx = Number().Matches(x); var my = Number().Matches(y);
        if (mx.Count != my.Count || Number().Replace(x, "#") != Number().Replace(y, "#")) return false;
        for (var i = 0; i < mx.Count; i++)
        {
            var sx = mx[i].Value; var sy = my[i].Value;
            if (sx == sy) continue;
            var dx = double.Parse(sx, CultureInfo.InvariantCulture); var dy = double.Parse(sy, CultureInfo.InvariantCulture);
            var dec = Math.Max(Decimals(sx), Decimals(sy));
            if (Math.Abs(dx - dy) > Math.Max(Tol, 1.0001 * Math.Pow(10, -dec))) return false;
        }
        return true;
    }

    static int Decimals(string s)
    {
        if (s.Contains('e') || s.Contains('E')) return 0;
        var i = s.IndexOf('.');
        return i < 0 ? 0 : s.Length - i - 1;
    }

    /// <summary>Key labels side by side from the first differing label (for the failure message).</summary>
    public static string LabelSummary(string expectedJson, string actualJson)
    {
        static List<string> Labels(string j)
        {
            using var d = JsonDocument.Parse(j);
            return d.RootElement.GetProperty("Keys").EnumerateArray().Select(k => k.GetProperty("Label").GetString() ?? "").ToList();
        }
        var a = Labels(expectedJson); var b = Labels(actualJson);
        var first = 0;
        while (first < Math.Min(a.Count, b.Count) && a[first] == b[first]) first++;
        var sb = new StringBuilder($"key labels: shipped {a.Count}, live {b.Count}; ");
        if (first == a.Count && first == b.Count) return sb.Append("identical").ToString();
        sb.Append($"first label difference at key {first}:\n");
        for (var i = Math.Max(0, first - 2); i < Math.Min(Math.Max(a.Count, b.Count), first + 12); i++)
            sb.Append($"  {i,4}: {(i < a.Count ? a[i] : "—"),-50} | {(i < b.Count ? b[i] : "—")}\n");
        return sb.ToString();
    }
}
