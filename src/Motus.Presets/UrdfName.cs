using System.Text;

namespace Motus.Presets;

/// <summary>Sanitize free-text names into safe URDF/XML identifiers (ASCII alphanumeric + underscore).</summary>
public static class UrdfName
{
    /// <summary>
    /// Replace any character outside <c>[A-Za-z0-9_]</c> with <c>_</c>, and prefix with <c>_</c> if the
    /// result would start with a digit (or be empty). Never returns an empty string.
    /// </summary>
    public static string Sanitize(string? name)
    {
        if (string.IsNullOrEmpty(name))
            return "_";

        var sb = new StringBuilder(name.Length);
        foreach (var c in name)
            sb.Append(IsSafe(c) ? c : '_');

        if (sb.Length == 0)
            return "_";
        if (char.IsDigit(sb[0]))
            sb.Insert(0, '_');

        return sb.ToString();
    }

    private static bool IsSafe(char c) =>
        (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
}
