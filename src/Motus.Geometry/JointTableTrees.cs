namespace Motus.Geometry;

/// <summary>One row of a Motus Joint Table (Wave 2 branching authoring).</summary>
public readonly record struct JointTableRow(
    string JointName,
    string ParentLink,
    string ChildLink,
    string Type,
    double OriginX,
    double OriginY,
    double OriginZ,
    double AxisX,
    double AxisY,
    double AxisZ,
    double Lower,
    double Upper);

/// <summary>Build a <see cref="KinematicTree"/> from joint rows (one authoring table, not Link×N).</summary>
public static class JointTableTrees
{
    /// <summary>
    /// Rows in parent-before-child order. Root link = first row's ParentLink (or "base_link").
    /// Actuated R/P/C become drivers; Fixed joints allowed (driverIndex -1).
    /// </summary>
    public static KinematicTree FromRows(
        IReadOnlyList<JointTableRow> rows,
        string name = "joint_table")
    {
        if (rows is null || rows.Count == 0)
            throw new ArgumentException("At least one joint row is required.", nameof(rows));

        var linkIndex = new Dictionary<string, int>(StringComparer.Ordinal);
        var links = new List<KinematicLink>();
        void EnsureLink(string linkName)
        {
            if (linkIndex.ContainsKey(linkName)) return;
            linkIndex[linkName] = links.Count;
            links.Add(new KinematicLink(linkName));
        }

        EnsureLink(string.IsNullOrWhiteSpace(rows[0].ParentLink) ? "base_link" : rows[0].ParentLink.Trim());

        var joints = new List<KinematicJoint>(rows.Count);
        var drivers = new List<int>();

        foreach (var row in rows)
        {
            var parent = string.IsNullOrWhiteSpace(row.ParentLink) ? "base_link" : row.ParentLink.Trim();
            var child = string.IsNullOrWhiteSpace(row.ChildLink)
                ? throw new ArgumentException($"Joint '{row.JointName}' missing ChildLink.")
                : row.ChildLink.Trim();
            EnsureLink(parent);
            EnsureLink(child);

            var type = ParseType(row.Type);
            var ax = row.AxisX;
            var ay = row.AxisY;
            var az = row.AxisZ;
            if (Math.Abs(ax) + Math.Abs(ay) + Math.Abs(az) < 1e-12)
                az = 1;

            var driverIndex = -1;
            if (type is KinematicJointType.Revolute or KinematicJointType.Continuous or KinematicJointType.Prismatic)
            {
                driverIndex = drivers.Count;
                drivers.Add(joints.Count);
            }

            var lo = row.Lower;
            var hi = row.Upper;
            if (type == KinematicJointType.Prismatic && hi <= lo)
                hi = lo + Math.Max(Math.Abs(row.OriginZ) + Math.Abs(row.OriginX), 1e-6);
            if (type is KinematicJointType.Revolute or KinematicJointType.Continuous && hi <= lo)
            {
                lo = -Math.PI;
                hi = Math.PI;
            }

            joints.Add(new KinematicJoint(
                string.IsNullOrWhiteSpace(row.JointName) ? $"j{joints.Count}" : row.JointName.Trim(),
                type,
                linkIndex[parent],
                linkIndex[child],
                row.OriginX, row.OriginY, row.OriginZ,
                0, 0, 0,
                ax, ay, az,
                lo, hi, Math.PI,
                driverIndex, mimic: null));
        }

        if (drivers.Count == 0)
            throw new ArgumentException("Joint table has no actuated joints.");

        return new KinematicTree(name, links, joints, rootLinkIndex: 0, drivers);
    }

    private static KinematicJointType ParseType(string? typeChar)
    {
        if (string.IsNullOrWhiteSpace(typeChar)) return KinematicJointType.Revolute;
        return typeChar.Trim().ToUpperInvariant() switch
        {
            "P" or "PRISMATIC" => KinematicJointType.Prismatic,
            "F" or "FIXED" => KinematicJointType.Fixed,
            "C" or "CONTINUOUS" => KinematicJointType.Continuous,
            "R" or "REVOLUTE" => KinematicJointType.Revolute,
            _ => throw new ArgumentException($"Unknown joint type '{typeChar}'. Use R, P, C, or F.")
        };
    }
}
