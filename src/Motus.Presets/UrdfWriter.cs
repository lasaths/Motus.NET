using System.Buffers.Binary;
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
    /// <summary>Hard cap for inline mesh vertices on parse (trust boundary for GH Internalise).</summary>
    public const int MaxInlineMeshVertices = 50_000;
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
        IReadOnlyDictionary<(string Link, int VisualIndex), string>? meshRefs = null,
        bool inlineMeshes = false)
    {
        ArgumentNullException.ThrowIfNull(description);
        _ = meshDirectoryRelative;

        var robotEl = new XElement("robot", new XAttribute("name", UrdfName.Sanitize(description.Name)));
        if (!string.IsNullOrWhiteSpace(description.TipLink))
            robotEl.SetAttributeValue("motus_tip", description.TipLink);
        foreach (var link in description.Links)
            robotEl.Add(BuildLinkElement(link, meshRefs, inlineMeshes));
        foreach (var joint in description.Joints)
            robotEl.Add(BuildJointElement(joint));

        var doc = new XDocument(new XDeclaration("1.0", "UTF-8", null), robotEl);
        return doc.ToString(SaveOptions.None) + Environment.NewLine;
    }

    /// <summary>
    /// Parse Motus <see cref="ToXml"/> output (primitives + optional inline meshes) back into a
    /// <see cref="RobotDescription"/>. Rejects NaN/Inf and oversized inline meshes.
    /// </summary>
    public static bool TryParse(
        string xml,
        out RobotDescription? description,
        out IReadOnlyList<string> errors)
    {
        description = null;
        var errs = new List<string>();
        if (string.IsNullOrWhiteSpace(xml))
        {
            errs.Add("URDF XML is empty.");
            errors = errs;
            return false;
        }

        XDocument doc;
        try
        {
            doc = XDocument.Parse(xml, LoadOptions.None);
        }
        catch (Exception ex)
        {
            errs.Add($"URDF XML parse failed: {ex.Message}");
            errors = errs;
            return false;
        }

        var robot = doc.Root;
        if (robot is null || !string.Equals(robot.Name.LocalName, "robot", StringComparison.OrdinalIgnoreCase))
        {
            errs.Add("URDF root element must be <robot>.");
            errors = errs;
            return false;
        }

        var name = robot.Attribute("name")?.Value ?? "robot";
        var tipLink = robot.Attribute("motus_tip")?.Value;
        var links = new List<UrdfLink>();
        var joints = new List<UrdfJoint>();

        foreach (var linkEl in robot.Elements().Where(e => e.Name.LocalName == "link"))
        {
            if (!TryParseLink(linkEl, out var link, out var linkErr))
            {
                errs.Add(linkErr!);
                continue;
            }
            links.Add(link!);
        }

        foreach (var jointEl in robot.Elements().Where(e => e.Name.LocalName == "joint"))
        {
            if (!TryParseJoint(jointEl, out var joint, out var jointErr))
            {
                errs.Add(jointErr!);
                continue;
            }
            joints.Add(joint!);
        }

        if (errs.Count > 0)
        {
            errors = errs;
            return false;
        }

        if (!RobotDescription.TryAssemble(name, links, joints, tipLink, out description, out var diag))
        {
            errs.AddRange(diag.Errors);
            errors = errs;
            return false;
        }

        errors = errs;
        return true;
    }

    private static bool TryParseLink(XElement linkEl, out UrdfLink? link, out string? error)
    {
        link = null;
        error = null;
        var linkName = linkEl.Attribute("name")?.Value;
        if (string.IsNullOrWhiteSpace(linkName))
        {
            error = "Link missing name.";
            return false;
        }

        var visuals = new List<UrdfGeometry>();
        var collisions = new List<UrdfGeometry>();
        double? mass = null;
        double? r = null, g = null, b = null, a = null;

        foreach (var visual in linkEl.Elements().Where(e => e.Name.LocalName == "visual"))
        {
            if (!TryParseGeometry(visual, out var geom, out error))
                return false;
            visuals.Add(geom!);
            if (r is null)
                TryParseRgba(visual, out r, out g, out b, out a);
        }

        foreach (var collision in linkEl.Elements().Where(e => e.Name.LocalName == "collision"))
        {
            if (!TryParseGeometry(collision, out var geom, out error))
                return false;
            collisions.Add(geom!);
        }

        var inertial = linkEl.Elements().FirstOrDefault(e => e.Name.LocalName == "inertial");
        var massEl = inertial?.Elements().FirstOrDefault(e => e.Name.LocalName == "mass");
        if (massEl?.Attribute("value") is { } mv &&
            double.TryParse(mv.Value, NumberStyles.Float, CultureInfo.InvariantCulture, out var massVal) &&
            IsFinite(massVal))
            mass = massVal;

        link = new UrdfLink(linkName, visuals, collisions, mass, r, g, b, a);
        return true;
    }

    private static bool TryParseGeometry(XElement parent, out UrdfGeometry? geom, out string? error)
    {
        geom = null;
        error = null;
        var origin = ParseOrigin(parent.Elements().FirstOrDefault(e => e.Name.LocalName == "origin"));
        var geometry = parent.Elements().FirstOrDefault(e => e.Name.LocalName == "geometry");
        if (geometry is null)
        {
            error = "Geometry element missing.";
            return false;
        }

        var box = geometry.Elements().FirstOrDefault(e => e.Name.LocalName == "box");
        if (box is not null)
        {
            var size = ParseTriple(box.Attribute("size")?.Value, 0.1, 0.1, 0.1);
            if (!IsFinite(size.x) || !IsFinite(size.y) || !IsFinite(size.z))
            {
                error = "Box size must be finite.";
                return false;
            }
            geom = UrdfGeometry.Box(size.x, size.y, size.z, origin);
            return true;
        }

        var cyl = geometry.Elements().FirstOrDefault(e => e.Name.LocalName == "cylinder");
        if (cyl is not null)
        {
            var radius = ParseDouble(cyl.Attribute("radius")?.Value, 0.05);
            var length = ParseDouble(cyl.Attribute("length")?.Value, 0.1);
            if (!IsFinite(radius) || !IsFinite(length))
            {
                error = "Cylinder radius/length must be finite.";
                return false;
            }
            geom = UrdfGeometry.Cylinder(radius, length, origin);
            return true;
        }

        var sphere = geometry.Elements().FirstOrDefault(e => e.Name.LocalName == "sphere");
        if (sphere is not null)
        {
            var radius = ParseDouble(sphere.Attribute("radius")?.Value, 0.05);
            if (!IsFinite(radius))
            {
                error = "Sphere radius must be finite.";
                return false;
            }
            geom = UrdfGeometry.Sphere(radius, origin);
            return true;
        }

        var mesh = geometry.Elements().FirstOrDefault(e => e.Name.LocalName == "mesh");
        if (mesh is not null)
        {
            var file = mesh.Attribute("filename")?.Value;
            var scaleAttr = mesh.Attribute("scale")?.Value;
            double[]? scale = null;
            if (!string.IsNullOrWhiteSpace(scaleAttr))
            {
                var s = ParseTriple(scaleAttr, 1, 1, 1);
                scale = [s.x, s.y, s.z];
            }

            var vertsEl = mesh.Elements().FirstOrDefault(e => e.Name.LocalName == "motus_vertices");
            var idxEl = mesh.Elements().FirstOrDefault(e => e.Name.LocalName == "motus_indices");
            if (vertsEl is not null && idxEl is not null)
            {
                if (!TryDecodeVertices(vertsEl.Value, out var verts, out error) ||
                    !TryDecodeIndices(idxEl.Value, out var indices, out error))
                    return false;
                if (verts!.Count > MaxInlineMeshVertices)
                {
                    error = $"Inline mesh has {verts.Count} vertices (max {MaxInlineMeshVertices}).";
                    return false;
                }
                geom = UrdfGeometry.Mesh(verts, indices!, file, origin, scale);
                return true;
            }

            // Filename-only mesh (no inline payload) — keep empty hull so topology round-trips.
            geom = UrdfGeometry.Mesh([], [], file, origin, scale);
            return true;
        }

        error = "Unsupported geometry kind (expected box/cylinder/sphere/mesh).";
        return false;
    }

    private static bool TryParseJoint(XElement jointEl, out UrdfJoint? joint, out string? error)
    {
        joint = null;
        error = null;
        var jName = jointEl.Attribute("name")?.Value;
        var type = jointEl.Attribute("type")?.Value;
        var parent = jointEl.Elements().FirstOrDefault(e => e.Name.LocalName == "parent")?.Attribute("link")?.Value;
        var child = jointEl.Elements().FirstOrDefault(e => e.Name.LocalName == "child")?.Attribute("link")?.Value;
        if (string.IsNullOrWhiteSpace(jName) || string.IsNullOrWhiteSpace(type) ||
            string.IsNullOrWhiteSpace(parent) || string.IsNullOrWhiteSpace(child))
        {
            error = "Joint requires name/type/parent/child.";
            return false;
        }

        var origin = ParseTriple(
            jointEl.Elements().FirstOrDefault(e => e.Name.LocalName == "origin")?.Attribute("xyz")?.Value,
            0, 0, 0);
        var axis = ParseTriple(
            jointEl.Elements().FirstOrDefault(e => e.Name.LocalName == "axis")?.Attribute("xyz")?.Value,
            0, 0, 1);
        var limit = jointEl.Elements().FirstOrDefault(e => e.Name.LocalName == "limit");
        var lower = ParseDouble(limit?.Attribute("lower")?.Value, -Math.PI);
        var upper = ParseDouble(limit?.Attribute("upper")?.Value, Math.PI);
        if (!IsFinite(origin.x) || !IsFinite(origin.y) || !IsFinite(origin.z) ||
            !IsFinite(axis.x) || !IsFinite(axis.y) || !IsFinite(axis.z) ||
            !IsFinite(lower) || !IsFinite(upper))
        {
            error = $"Joint '{jName}' has non-finite origin/axis/limits.";
            return false;
        }

        string? mimic = null;
        var mult = 1.0;
        var offset = 0.0;
        var mimicEl = jointEl.Elements().FirstOrDefault(e => e.Name.LocalName == "mimic");
        if (mimicEl is not null)
        {
            mimic = mimicEl.Attribute("joint")?.Value;
            mult = ParseDouble(mimicEl.Attribute("multiplier")?.Value, 1);
            offset = ParseDouble(mimicEl.Attribute("offset")?.Value, 0);
        }

        try
        {
            joint = new UrdfJoint(
                jName, type, parent, child,
                origin.x, origin.y, origin.z,
                axis.x, axis.y, axis.z,
                lower, upper,
                mimic, mult, offset);
            return true;
        }
        catch (Exception ex)
        {
            error = ex.Message;
            return false;
        }
    }

    private static Frame ParseOrigin(XElement? originEl)
    {
        if (originEl is null) return Frame.Identity;
        var xyz = ParseTriple(originEl.Attribute("xyz")?.Value, 0, 0, 0);
        var rpy = ParseTriple(originEl.Attribute("rpy")?.Value, 0, 0, 0);
        if (!IsFinite(xyz.x) || !IsFinite(xyz.y) || !IsFinite(xyz.z) ||
            !IsFinite(rpy.x) || !IsFinite(rpy.y) || !IsFinite(rpy.z))
            return Frame.Identity;
        return Transforms.ToFrame(Transforms.FromRpy(xyz.x, xyz.y, xyz.z, rpy.x, rpy.y, rpy.z));
    }

    private static void TryParseRgba(XElement visual, out double? r, out double? g, out double? b, out double? a)
    {
        r = g = b = a = null;
        var color = visual.Elements()
            .FirstOrDefault(e => e.Name.LocalName == "material")
            ?.Elements().FirstOrDefault(e => e.Name.LocalName == "color")
            ?.Attribute("rgba")?.Value;
        if (string.IsNullOrWhiteSpace(color)) return;
        var p = color.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        if (p.Length < 3) return;
        if (double.TryParse(p[0], NumberStyles.Float, CultureInfo.InvariantCulture, out var rv) &&
            double.TryParse(p[1], NumberStyles.Float, CultureInfo.InvariantCulture, out var gv) &&
            double.TryParse(p[2], NumberStyles.Float, CultureInfo.InvariantCulture, out var bv))
        {
            r = rv; g = gv; b = bv;
            if (p.Length >= 4 &&
                double.TryParse(p[3], NumberStyles.Float, CultureInfo.InvariantCulture, out var av))
                a = av;
        }
    }

    private static bool TryDecodeVertices(string b64, out List<double[]>? verts, out string? error)
    {
        verts = null;
        error = null;
        try
        {
            var bytes = Convert.FromBase64String(b64.Trim());
            if (bytes.Length % 24 != 0)
            {
                error = "Inline mesh vertices payload length invalid.";
                return false;
            }
            var count = bytes.Length / 24;
            verts = new List<double[]>(count);
            for (var i = 0; i < count; i++)
            {
                var o = i * 24;
                var x = BinaryPrimitives.ReadDoubleLittleEndian(bytes.AsSpan(o, 8));
                var y = BinaryPrimitives.ReadDoubleLittleEndian(bytes.AsSpan(o + 8, 8));
                var z = BinaryPrimitives.ReadDoubleLittleEndian(bytes.AsSpan(o + 16, 8));
                if (!IsFinite(x) || !IsFinite(y) || !IsFinite(z))
                {
                    error = "Inline mesh vertices must be finite.";
                    return false;
                }
                verts.Add([x, y, z]);
            }
            return true;
        }
        catch (Exception ex)
        {
            error = $"Inline mesh vertices decode failed: {ex.Message}";
            return false;
        }
    }

    private static bool TryDecodeIndices(string b64, out List<int>? indices, out string? error)
    {
        indices = null;
        error = null;
        try
        {
            var bytes = Convert.FromBase64String(b64.Trim());
            if (bytes.Length % 4 != 0)
            {
                error = "Inline mesh indices payload length invalid.";
                return false;
            }
            var count = bytes.Length / 4;
            indices = new List<int>(count);
            for (var i = 0; i < count; i++)
                indices.Add(BinaryPrimitives.ReadInt32LittleEndian(bytes.AsSpan(i * 4, 4)));
            return true;
        }
        catch (Exception ex)
        {
            error = $"Inline mesh indices decode failed: {ex.Message}";
            return false;
        }
    }

    private static string EncodeVertices(IReadOnlyList<double[]> vertices)
    {
        var bytes = new byte[vertices.Count * 24];
        for (var i = 0; i < vertices.Count; i++)
        {
            var v = vertices[i];
            var o = i * 24;
            BinaryPrimitives.WriteDoubleLittleEndian(bytes.AsSpan(o, 8), v[0]);
            BinaryPrimitives.WriteDoubleLittleEndian(bytes.AsSpan(o + 8, 8), v[1]);
            BinaryPrimitives.WriteDoubleLittleEndian(bytes.AsSpan(o + 16, 8), v[2]);
        }
        return Convert.ToBase64String(bytes);
    }

    private static string EncodeIndices(IReadOnlyList<int> indices)
    {
        var bytes = new byte[indices.Count * 4];
        for (var i = 0; i < indices.Count; i++)
            BinaryPrimitives.WriteInt32LittleEndian(bytes.AsSpan(i * 4, 4), indices[i]);
        return Convert.ToBase64String(bytes);
    }

    private static (double x, double y, double z) ParseTriple(string? s, double dx, double dy, double dz)
    {
        if (string.IsNullOrWhiteSpace(s)) return (dx, dy, dz);
        var p = s.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        if (p.Length < 3) return (dx, dy, dz);
        return (ParseDouble(p[0], dx), ParseDouble(p[1], dy), ParseDouble(p[2], dz));
    }

    private static double ParseDouble(string? s, double fallback) =>
        double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) ? v : fallback;

    private static bool IsFinite(double v) => !double.IsNaN(v) && !double.IsInfinity(v);

    private static XElement BuildLinkElement(
        UrdfLink link,
        IReadOnlyDictionary<(string Link, int VisualIndex), string>? meshRefs,
        bool inlineMeshes)
    {
        var el = new XElement("link", new XAttribute("name", UrdfName.Sanitize(link.Name)));

        for (var i = 0; i < link.Visuals.Count; i++)
        {
            var visual = new XElement("visual");
            AddOrigin(visual, link.Visuals[i].Origin);
            visual.Add(new XElement("geometry", BuildGeometryElement(link, i, link.Visuals[i], meshRefs, inlineMeshes)));
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
                BuildGeometryElement(link, geomIndex, collisions[i], meshRefs, inlineMeshes)));
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
        IReadOnlyDictionary<(string Link, int VisualIndex), string>? meshRefs,
        bool inlineMeshes)
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
            UrdfGeometryKind.Mesh => BuildMeshElement(link, index, g, meshRefs, inlineMeshes),
            _ => throw new ArgumentOutOfRangeException(nameof(g), g.Kind, "Unsupported geometry kind.")
        };
    }

    private static XElement BuildMeshElement(
        UrdfLink link,
        int index,
        UrdfGeometry g,
        IReadOnlyDictionary<(string Link, int VisualIndex), string>? meshRefs,
        bool inlineMeshes)
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
        if (inlineMeshes && g.Vertices is { Count: > 0 } verts && g.Indices is { Count: > 0 } indices)
        {
            mesh.Add(new XElement("motus_vertices", EncodeVertices(verts)));
            mesh.Add(new XElement("motus_indices", EncodeIndices(indices)));
        }
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
