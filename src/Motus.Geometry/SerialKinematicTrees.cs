namespace Motus.Geometry;

/// <summary>
/// Build a serial (optional rail) <see cref="KinematicTree"/> from link lengths.
/// Used by GH Motus Serial Chain; keeps authoring logic in Motus.NET.
/// </summary>
public static class SerialKinematicTrees
{
    /// <summary>
    /// <paramref name="lengths"/> are successive joint origin offsets along +X (meters) after the first.
    /// When <paramref name="rail"/> is true, lengths[0] is prismatic stroke (+Z), remaining are revolute arm links.
    /// </summary>
    public static KinematicTree FromLengths(
        IReadOnlyList<double> lengths,
        bool rail = false,
        string name = "serial_chain",
        IReadOnlyList<string>? types = null)
    {
        if (lengths is null || lengths.Count == 0)
            throw new ArgumentException("At least one length is required.", nameof(lengths));

        var n = lengths.Count;
        var links = new List<KinematicLink>(n + 1) { new("base_link") };
        var joints = new List<KinematicJoint>(n);
        var drivers = new List<int>(n);

        for (var i = 0; i < n; i++)
            links.Add(new KinematicLink(i == n - 1 ? "tool0" : $"link{i + 1}"));

        for (var i = 0; i < n; i++)
        {
            var isRail = rail && i == 0;
            var typeChar = types is not null && i < types.Count ? types[i] : null;
            var type = ResolveType(isRail, typeChar);

            double ox = 0, oy = 0, oz = 0;
            double ax = 0, ay = 0, az = 1;
            double lo = -Math.PI, hi = Math.PI;

            if (type == KinematicJointType.Prismatic)
            {
                az = 1;
                lo = 0;
                hi = Math.Max(lengths[i], 1e-6);
                // prismatic origin at previous tip; stroke applied via q
            }
            else
            {
                // place next revolute at previous link length along +X (first joint at origin height)
                if (i == 0)
                    oz = Math.Max(lengths[i], 0);
                else
                    ox = Math.Max(lengths[i], 0);
                // default industrial Z-up revolute (Axes override later in GH)
                az = 1;
            }

            var driverIndex = drivers.Count;
            drivers.Add(i);
            joints.Add(new KinematicJoint(
                $"j{i}", type, i, i + 1,
                ox, oy, oz, 0, 0, 0,
                ax, ay, az,
                lo, hi, Math.PI,
                driverIndex, mimic: null));
        }

        return new KinematicTree(name, links, joints, rootLinkIndex: 0, drivers);
    }

    private static KinematicJointType ResolveType(bool isRail, string? typeChar)
    {
        if (isRail) return KinematicJointType.Prismatic;
        if (string.IsNullOrWhiteSpace(typeChar)) return KinematicJointType.Revolute;
        return typeChar.Trim().ToUpperInvariant() switch
        {
            "P" or "PRISMATIC" => KinematicJointType.Prismatic,
            "R" or "REVOLUTE" or "C" or "CONTINUOUS" => KinematicJointType.Revolute,
            _ => throw new ArgumentException($"Unknown joint type '{typeChar}'. Use R or P.")
        };
    }
}
