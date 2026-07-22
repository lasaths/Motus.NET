using System.Globalization;
using System.Xml.Linq;
using Motus.Core;
using Motus.Geometry;
using System.IO;
using System.Linq;

namespace Motus.Presets;

/// <summary>
/// Serializes <see cref="RobotDescription"/> to URDF XML (+ optional mesh sidecars).
/// Motus.NET owns the format; GH only passes paths and calls Write.
/// </summary>
public static class UrdfWriter
{
    /// <summary>
    /// Write <c>{outputDirectory}/{name}.urdf</c> and optional <c>meshes/*.stl</c>.
    /// Rejects path escape outside <paramref name="outputDirectory"/>.
    /// </summary>
    public static string Write(
        RobotDescription description,
        string outputDirectory,
        string? fileName = null,
        bool writeMeshes = true)
    {
        ArgumentNullException.ThrowIfNull(description);
        if (string.IsNullOrWhiteSpace(outputDirectory))
            throw new ArgumentException("outputDirectory must be provided.", nameof(outputDirectory));
        if (!Path.IsPathRooted(outputDirectory))
            throw new ArgumentException($"outputDirectory must be an absolute path: '{outputDirectory}'.", nameof(outputDirectory));

        var outDirFull = Path.GetFullPath(outputDirectory);
        Directory.CreateDirectory(outDirFull);

        var safeName = UrdfName.Sanitize(fileName ?? description.Name);
        var urdfPath = ResolveSafePath(outDirFull, safeName + ".urdf");

        var meshRefs = new Dictionary<(string Link, int VisualIndex), string>();
        if (writeMeshes)
        {
            var meshDir = ResolveSafePath(outDirFull, "meshes");
            Directory.CreateDirectory(meshDir);
            foreach (var link in description.Links)
            {
                for (var i = 0; i < link.Visuals.Count; i++)
                {
                    var g = link.Visuals[i];
                    if (g.Kind != UrdfGeometryKind.Mesh || g.Vertices is null || g.Indices is null)
                        continue;
                    if (g.Vertices.Count == 0 || g.Indices.Count < 3)
                        continue;

                    var meshFile = $"{UrdfName.Sanitize(link.Name)}_{i}.stl";
                    var meshPath = ResolveSafePath(meshDir, meshFile);
                    StlWriter.WriteBinary(meshPath, g.Vertices, g.Indices);
                    meshRefs[(link.Name, i)] = $"meshes/{meshFile}";
                }
            }
        }

        File.WriteAllText(urdfPath, ToXml(description, "meshes", meshRefs));
        return urdfPath;
    }

    /// <summary>Render URDF XML. Mesh filenames use <paramref name="meshRefs"/> or existing <see cref="UrdfGeometry.FilePath"/>.</summary>
    public static string ToXml(
        RobotDescription description,
        string? meshDirectoryRelative = "meshes",
        IReadOnlyDictionary<(string Link, int VisualIndex), string>? meshRefs = null)
    {
        ArgumentNullException.ThrowIfNull(description);
        _ = meshDirectoryRelative;

        var robotEl = new XElement("robot", new XAttribute("name", UrdfName.Sanitize(description.Name)));
        foreach (var link in description.Links)
            robotEl.Add(BuildLinkElement(link, meshRefs));
        foreach (var joint in description.Joints)
            robotEl.Add(BuildJointElement(joint));

        var doc = new XDocument(new XDeclaration("1.0", "UTF-8", null), robotEl);
        return doc.ToString(SaveOptions.None) + Environment.NewLine;
    }

    private static XElement BuildLinkElement(
        UrdfLink link,
        IReadOnlyDictionary<(string Link, int VisualIndex), string>? meshRefs)
    {
        var el = new XElement("link", new XAttribute("name", UrdfName.Sanitize(link.Name)));

        for (var i = 0; i < link.Visuals.Count; i++)
        {
            var visual = new XElement("visual");
            AddOrigin(visual, link.Visuals[i].Origin);
            visual.Add(new XElement("geometry", BuildGeometryElement(link, i, link.Visuals[i], meshRefs)));
            if (link.R is not null && link.G is not null && link.B is not null && i == 0)
            {
                var a = link.A ?? 1;
                visual.Add(new XElement("material",
                    new XAttribute("name", UrdfName.Sanitize(link.Name) + "_mat"),
                    new XElement("color",
                        new XAttribute("rgba",
                            $"{Fmt(link.R.Value)} {Fmt(link.G.Value)} {Fmt(link.B.Value)} {Fmt(a)}"))));
            }
            el.Add(visual);
        }

        var collisions = link.Collisions.Count > 0 ? link.Collisions : link.Visuals;
        for (var i = 0; i < collisions.Count; i++)
        {
            var collision = new XElement("collision");
            AddOrigin(collision, collisions[i].Origin);
            // Collision mesh: reuse visual mesh ref when collision list is empty (visuals used as fallback).
            var geomIndex = link.Collisions.Count > 0 ? i : i;
            collision.Add(new XElement("geometry",
                BuildGeometryElement(link, geomIndex, collisions[i], meshRefs)));
            el.Add(collision);
        }

        if (link.Mass is { } mass)
        {
            el.Add(new XElement("inertial",
                new XElement("mass", new XAttribute("value", Fmt(mass))),
                new XElement("inertia",
                    new XAttribute("ixx", "1e-6"), new XAttribute("ixy", "0"), new XAttribute("ixz", "0"),
                    new XAttribute("iyy", "1e-6"), new XAttribute("iyz", "0"), new XAttribute("izz", "1e-6"))));
        }

        return el;
    }

    private static XElement BuildGeometryElement(
        UrdfLink link,
        int index,
        UrdfGeometry g,
        IReadOnlyDictionary<(string Link, int VisualIndex), string>? meshRefs)
    {
        return g.Kind switch
        {
            UrdfGeometryKind.Box => new XElement("box",
                new XAttribute("size", FormatTriple(g.SizeX, g.SizeY, g.SizeZ))),
            UrdfGeometryKind.Cylinder => new XElement("cylinder",
                new XAttribute("radius", Fmt(g.Radius)),
                new XAttribute("length", Fmt(g.Length))),
            UrdfGeometryKind.Sphere => new XElement("sphere",
                new XAttribute("radius", Fmt(g.Radius))),
            UrdfGeometryKind.Mesh => BuildMeshElement(link, index, g, meshRefs),
            _ => throw new ArgumentOutOfRangeException(nameof(g), g.Kind, "Unsupported geometry kind.")
        };
    }

    private static XElement BuildMeshElement(
        UrdfLink link,
        int index,
        UrdfGeometry g,
        IReadOnlyDictionary<(string Link, int VisualIndex), string>? meshRefs)
    {
        string filename;
        if (meshRefs is not null && meshRefs.TryGetValue((link.Name, index), out var mapped))
            filename = mapped;
        else if (!string.IsNullOrWhiteSpace(g.FilePath))
            filename = SanitizeMeshFilename(g.FilePath!);
        else
            filename = $"meshes/{UrdfName.Sanitize(link.Name)}_{index}.stl";

        var mesh = new XElement("mesh", new XAttribute("filename", filename));
        if (g.Scale is { Length: >= 3 } s)
            mesh.Add(new XAttribute("scale", FormatTriple(s[0], s[1], s[2])));
        return mesh;
    }

    private static XElement BuildJointElement(UrdfJoint joint)
    {
        var el = new XElement("joint",
            new XAttribute("name", UrdfName.Sanitize(joint.Name)),
            new XAttribute("type", JointTypeString(joint.Kind)),
            new XElement("origin",
                new XAttribute("xyz", FormatTriple(joint.OriginX, joint.OriginY, joint.OriginZ)),
                new XAttribute("rpy", "0 0 0")),
            new XElement("parent", new XAttribute("link", UrdfName.Sanitize(joint.ParentLink))),
            new XElement("child", new XAttribute("link", UrdfName.Sanitize(joint.ChildLink))));

        if (joint.Kind != UrdfJointKind.Fixed)
        {
            el.Add(new XElement("axis",
                new XAttribute("xyz", FormatTriple(joint.AxisX, joint.AxisY, joint.AxisZ))));
            el.Add(new XElement("limit",
                new XAttribute("lower", Fmt(joint.Lower)),
                new XAttribute("upper", Fmt(joint.Upper)),
                new XAttribute("effort", "0"),
                new XAttribute("velocity", Fmt(Math.PI))));

            if (joint.MimicJoint is { } mimicName)
            {
                el.Add(new XElement("mimic",
                    new XAttribute("joint", UrdfName.Sanitize(mimicName)),
                    new XAttribute("multiplier", Fmt(joint.MimicMultiplier)),
                    new XAttribute("offset", Fmt(joint.MimicOffset))));
            }
        }

        return el;
    }

    private static void AddOrigin(XElement parent, Frame origin)
    {
        if (origin.Equals(Frame.Identity))
            return;
        var (roll, pitch, yaw) = FrameToRpy(origin);
        parent.Add(new XElement("origin",
            new XAttribute("xyz", FormatTriple(origin.X, origin.Y, origin.Z)),
            new XAttribute("rpy", FormatTriple(roll, pitch, yaw))));
    }

    /// <summary>URDF fixed-axis XYZ RPY from unit quaternion (w,x,y,z).</summary>
    private static (double Roll, double Pitch, double Yaw) FrameToRpy(Frame f)
    {
        var w = f.Qw; var x = f.Qx; var y = f.Qy; var z = f.Qz;
        var sinr = 2 * (w * x + y * z);
        var cosr = 1 - 2 * (x * x + y * y);
        var roll = Math.Atan2(sinr, cosr);

        var sinp = 2 * (w * y - z * x);
        var pitch = Math.Abs(sinp) >= 1 ? Math.CopySign(Math.PI / 2, sinp) : Math.Asin(sinp);

        var siny = 2 * (w * z + x * y);
        var cosy = 1 - 2 * (y * y + z * z);
        var yaw = Math.Atan2(siny, cosy);
        return (roll, pitch, yaw);
    }

    private static string JointTypeString(UrdfJointKind kind) => kind switch
    {
        UrdfJointKind.Revolute => "revolute",
        UrdfJointKind.Continuous => "continuous",
        UrdfJointKind.Prismatic => "prismatic",
        UrdfJointKind.Fixed => "fixed",
        _ => throw new ArgumentOutOfRangeException(nameof(kind), kind, "Unsupported joint type.")
    };

    /// <summary>Allow only relative mesh paths with no <c>..</c> / rooted / URI forms.</summary>
    private static string SanitizeMeshFilename(string path)
    {
        var normalized = path.Replace('\\', '/').Trim();
        if (string.IsNullOrEmpty(normalized) ||
            normalized.Contains("://", StringComparison.Ordinal) ||
            Path.IsPathRooted(path) ||
            normalized.Split('/', StringSplitOptions.RemoveEmptyEntries).Any(s => s == ".."))
        {
            throw new InvalidOperationException(
                $"Mesh filename '{path}' must be a relative path without '..' or URI scheme.");
        }

        return normalized;
    }

    private static string FormatTriple(double x, double y, double z) => $"{Fmt(x)} {Fmt(y)} {Fmt(z)}";
    private static string Fmt(double v) => v.ToString("G17", CultureInfo.InvariantCulture);

    private static string ResolveSafePath(string baseDirFull, string relativeFileName)
    {
        var combined = Path.GetFullPath(Path.Combine(baseDirFull, relativeFileName));
        if (!IsUnderDirectory(combined, baseDirFull) &&
            !string.Equals(combined, baseDirFull, StringComparison.OrdinalIgnoreCase))
            throw new InvalidOperationException($"Resolved path '{combined}' escapes output directory '{baseDirFull}'.");
        return combined;
    }

    private static bool IsUnderDirectory(string path, string directory)
    {
        if (string.Equals(path, directory, StringComparison.OrdinalIgnoreCase))
            return true;
        var prefix = directory.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar)
                     + Path.DirectorySeparatorChar;
        return path.StartsWith(prefix, StringComparison.OrdinalIgnoreCase);
    }
}
