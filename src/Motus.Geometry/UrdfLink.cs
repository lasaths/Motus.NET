namespace Motus.Geometry;

/// <summary>In-memory URDF link: visual/collision geometry plus optional mass and material tint.</summary>
public sealed class UrdfLink
{
    public UrdfLink(
        string name,
        IReadOnlyList<UrdfGeometry>? visuals = null,
        IReadOnlyList<UrdfGeometry>? collisions = null,
        double? mass = null,
        double? r = null, double? g = null, double? b = null, double? a = null)
    {
        if (string.IsNullOrWhiteSpace(name))
            throw new ArgumentException("Link name is required.", nameof(name));

        Name = name;
        Visuals = visuals ?? [];
        Collisions = collisions ?? [];
        Mass = mass;
        R = r; G = g; B = b; A = a;
    }

    public string Name { get; }
    public IReadOnlyList<UrdfGeometry> Visuals { get; }
    public IReadOnlyList<UrdfGeometry> Collisions { get; }
    public double? Mass { get; }

    /// <summary>Optional RGBA tint (0-1) applied to the first visual's material; null means unspecified.</summary>
    public double? R { get; }
    public double? G { get; }
    public double? B { get; }
    public double? A { get; }
}
