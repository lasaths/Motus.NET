namespace Motus.Core;

/// <summary>Schema for a single actuated end-effector parameter.</summary>
public sealed class ToolParameter
{
    public string Name { get; }
    public string Unit { get; }
    public double Min { get; }
    public double Max { get; }
    public double Default { get; }

    public ToolParameter(string name, string unit, double min, double max, double defaultValue)
    {
        Name = string.IsNullOrWhiteSpace(name) ? throw new ArgumentException("Parameter name required.", nameof(name)) : name.Trim();
        Unit = unit ?? string.Empty;
        Min = min;
        Max = max;
        Default = Math.Clamp(defaultValue, min, max);
    }
}

/// <summary>Declares which parameters a tool exposes along a trajectory.</summary>
public sealed class ToolCapabilities
{
    public IReadOnlyList<ToolParameter> Parameters { get; }

    public ToolCapabilities(IReadOnlyList<ToolParameter> parameters)
    {
        Parameters = parameters ?? throw new ArgumentNullException(nameof(parameters));
        if (parameters.Count == 0)
            throw new ArgumentException("At least one parameter is required.", nameof(parameters));
    }

    public EndEffectorState DefaultState()
    {
        var values = new Dictionary<string, double>(Parameters.Count, StringComparer.Ordinal);
        foreach (var p in Parameters)
            values[p.Name] = p.Default;
        return new EndEffectorState(values);
    }

    public EndEffectorState Clamp(EndEffectorState state)
    {
        var byName = Parameters.ToDictionary(p => p.Name, p => p, StringComparer.Ordinal);
        var clamped = new Dictionary<string, double>(StringComparer.Ordinal);
        foreach (var (name, value) in state.Values)
        {
            if (!byName.TryGetValue(name, out var spec))
                continue;
            clamped[name] = Math.Clamp(value, spec.Min, spec.Max);
        }
        foreach (var p in Parameters)
        {
            if (!clamped.ContainsKey(p.Name))
                clamped[p.Name] = p.Default;
        }
        return new EndEffectorState(clamped);
    }

    public IReadOnlyList<string> Validate(EndEffectorState state)
    {
        var errors = new List<string>();
        var known = Parameters.ToDictionary(p => p.Name, p => p, StringComparer.Ordinal);
        foreach (var name in state.Values.Keys)
        {
            if (!known.ContainsKey(name))
                errors.Add($"Unknown tool parameter '{name}'.");
        }
        foreach (var p in Parameters)
        {
            if (!state.Values.TryGetValue(p.Name, out var value)) continue;
            if (value < p.Min - 1e-9 || value > p.Max + 1e-9)
                errors.Add($"Parameter '{p.Name}' value {value} is outside [{p.Min}, {p.Max}].");
        }
        return errors;
    }

    /// <summary>Robotiq 2F-85 jaw width (closed mesh is baked at width=0).</summary>
    public static ToolCapabilities Robotiq2F85 { get; } = new(new[]
    {
        new ToolParameter("width", "m", 0, 0.085, 0.085),
        new ToolParameter("speed", "ratio", 0, 1, 0.5),
        new ToolParameter("force", "ratio", 0, 1, 0.5)
    });

    /// <summary>
    /// Custom jaw-width Cap schema (meters). Used by GH Cap=<c>Custom</c>; Tool State / export use <c>width</c>.
    /// </summary>
    public static ToolCapabilities WidthSchema(double minMeters, double maxMeters, double defaultMeters)
    {
        if (!(maxMeters > minMeters))
            throw new ArgumentException("width max must be greater than min.", nameof(maxMeters));
        if (double.IsNaN(minMeters) || double.IsInfinity(minMeters) ||
            double.IsNaN(maxMeters) || double.IsInfinity(maxMeters) ||
            double.IsNaN(defaultMeters) || double.IsInfinity(defaultMeters))
            throw new ArgumentException("width bounds must be finite.");
        return new ToolCapabilities(new[]
        {
            new ToolParameter("width", "m", minMeters, maxMeters, defaultMeters)
        });
    }

    /// <summary>
    /// Declarative Wave-3 <c>width</c>→driver binding for the bundled Robotiq 2F-85 URDF, whose primary
    /// actuated knuckle joint is named exactly <c>robotiq_left_knuckle</c> (URDF mimic joints follow it).
    /// </summary>
    public static IReadOnlyList<ToolDriverBinding> Robotiq2F85DefaultBindings { get; } = new[]
    {
        new ToolDriverBinding(
            Parameter: "width",
            DriverJoint: "robotiq_left_knuckle",
            OpenValue: ToolParameterBinding.Robotiq2F85OpenWidthMeters,
            ClosedDriverValue: ToolParameterBinding.Robotiq2F85ClosedDriverRadians)
    };
}
