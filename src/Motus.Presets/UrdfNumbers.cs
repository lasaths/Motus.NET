using System.Globalization;

namespace Motus.Presets;

/// <summary>
/// Fail-closed URDF number parsing. An absent (or empty) attribute gets the default. An attribute that is present
/// must hold finite numbers, else the load stops with an <see cref="InvalidDataException"/> that names the element.
/// </summary>
internal static class UrdfNumbers
{
    public static double Parse(string? s, double fallback, string what)
    {
        if (string.IsNullOrWhiteSpace(s)) return fallback;
        if (double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) && double.IsFinite(v))
            return v;
        throw new InvalidDataException($"URDF {what}: '{s}' is not a finite number.");
    }

    public static (double x, double y, double z) Triple(string? s, double dx, double dy, double dz, string what)
    {
        if (string.IsNullOrWhiteSpace(s)) return (dx, dy, dz);
        var p = s.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        if (p.Length != 3) throw new InvalidDataException($"URDF {what}: '{s}' must be three numbers.");
        return (Parse(p[0], dx, what), Parse(p[1], dy, what), Parse(p[2], dz, what));
    }
}
