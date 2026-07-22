namespace Motus.Geometry;

public enum UrdfJointKind
{
    Revolute,
    Continuous,
    Prismatic,
    Fixed
}

/// <summary>
/// In-memory URDF joint. Origin is translation-only (meters) in the parent link's frame; the joint's
/// own rotation (revolute/continuous) or translation (prismatic) happens about/along <see cref="AxisX"/>/
/// <see cref="AxisY"/>/<see cref="AxisZ"/>, which may point in any direction. There is no separate origin
/// rotation in this authoring schema — arbitrary joint orientation is expressed entirely via the axis.
/// </summary>
public sealed class UrdfJoint
{
    public UrdfJoint(
        string name,
        string type,
        string parentLink,
        string childLink,
        double originX, double originY, double originZ,
        double axisX, double axisY, double axisZ,
        double lower, double upper,
        string? mimicJoint = null, double mimicMultiplier = 1, double mimicOffset = 0)
    {
        if (string.IsNullOrWhiteSpace(name))
            throw new ArgumentException("Joint name is required.", nameof(name));
        if (string.IsNullOrWhiteSpace(parentLink))
            throw new ArgumentException("Joint parent link is required.", nameof(parentLink));
        if (string.IsNullOrWhiteSpace(childLink))
            throw new ArgumentException("Joint child link is required.", nameof(childLink));

        Name = name;
        Kind = ParseKind(type);
        ParentLink = parentLink;
        ChildLink = childLink;
        OriginX = originX; OriginY = originY; OriginZ = originZ;

        var len = Math.Sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
        if (Kind is UrdfJointKind.Fixed || len < 1e-12)
        {
            AxisX = 0; AxisY = 0; AxisZ = Kind is UrdfJointKind.Fixed ? 0 : 1;
        }
        else
        {
            AxisX = axisX / len; AxisY = axisY / len; AxisZ = axisZ / len;
        }

        Lower = lower;
        Upper = upper;
        MimicJoint = string.IsNullOrWhiteSpace(mimicJoint) ? null : mimicJoint;
        MimicMultiplier = mimicMultiplier;
        MimicOffset = mimicOffset;
    }

    public string Name { get; }
    public UrdfJointKind Kind { get; }
    public string ParentLink { get; }
    public string ChildLink { get; }
    public double OriginX { get; }
    public double OriginY { get; }
    public double OriginZ { get; }
    public double AxisX { get; }
    public double AxisY { get; }
    public double AxisZ { get; }
    public double Lower { get; }
    public double Upper { get; }
    /// <summary>Name of the joint this one mimics (q = Multiplier * q[target] + Offset), or null if not a mimic joint.</summary>
    public string? MimicJoint { get; }
    public double MimicMultiplier { get; }
    public double MimicOffset { get; }

    public bool IsActuated => Kind is UrdfJointKind.Revolute or UrdfJointKind.Continuous or UrdfJointKind.Prismatic;

    private static UrdfJointKind ParseKind(string? type)
    {
        if (string.IsNullOrWhiteSpace(type))
            throw new ArgumentException("Joint type is required.", nameof(type));
        return type.Trim().ToUpperInvariant() switch
        {
            "R" or "REVOLUTE" => UrdfJointKind.Revolute,
            "C" or "CONTINUOUS" => UrdfJointKind.Continuous,
            "P" or "PRISMATIC" => UrdfJointKind.Prismatic,
            "F" or "FIXED" => UrdfJointKind.Fixed,
            _ => throw new ArgumentException($"Unknown joint type '{type}'. Use R, P, C, F or the full URDF name.", nameof(type))
        };
    }
}
