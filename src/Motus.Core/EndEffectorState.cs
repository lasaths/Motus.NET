namespace Motus.Core;

/// <summary>Time-varying end-effector parameter values (parallel to arm joint state).</summary>
public sealed class EndEffectorState
{
    public IReadOnlyDictionary<string, double> Values { get; }

    public EndEffectorState(IReadOnlyDictionary<string, double> values)
    {
        Values = new Dictionary<string, double>(values, StringComparer.Ordinal);
    }

    public double GetValueOrDefault(string name, double fallback = 0) =>
        Values.TryGetValue(name, out var v) ? v : fallback;

    public EndEffectorState With(string name, double value)
    {
        var copy = new Dictionary<string, double>(Values, StringComparer.Ordinal);
        copy[name] = value;
        return new EndEffectorState(copy);
    }

    public static EndEffectorState Lerp(EndEffectorState? from, EndEffectorState? to, double alpha)
    {
        alpha = Math.Clamp(alpha, 0, 1);
        if (from is null && to is null) return new EndEffectorState(new Dictionary<string, double>());
        if (from is null) return to!;
        if (to is null) return from;
        if (alpha <= 0) return from;
        if (alpha >= 1) return to;

        var keys = from.Values.Keys.Union(to.Values.Keys, StringComparer.Ordinal);
        var result = new Dictionary<string, double>(StringComparer.Ordinal);
        foreach (var key in keys)
        {
            var a = from.GetValueOrDefault(key);
            var b = to.GetValueOrDefault(key);
            result[key] = a + alpha * (b - a);
        }
        return new EndEffectorState(result);
    }

    public override string ToString() =>
        Values.Count == 0
            ? "ToolState[]"
            : "ToolState[" + string.Join(", ", Values.Select(kv => $"{kv.Key}={kv.Value:F4}")) + "]";
}

public enum ToolStateMode
{
    Hold,
    Ramp,
    Instant
}
