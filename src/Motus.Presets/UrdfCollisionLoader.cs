using System.Globalization;
using System.Xml.Linq;
using Motus.Core;
using Motus.Geometry;

namespace Motus.Presets;

public static class UrdfCollisionLoader
{
    public static RobotCollisionModel? Load(
        XElement robotRoot,
        IReadOnlyList<string> chainLinkNames,
        string urdfDirectory)
    {
        var indexed = new List<(int Index, string Name)>(chainLinkNames.Count);
        for (var i = 0; i < chainLinkNames.Count; i++)
            indexed.Add((i, chainLinkNames[i]));
        return LoadIndexed(robotRoot, indexed, urdfDirectory);
    }

    /// <summary>
    /// Collision for every tree link that declares geometry. LinkIndex = tree link index (TreeFK posing).
    /// </summary>
    public static RobotCollisionModel? LoadTree(
        XElement robotRoot,
        KinematicTree tree,
        string urdfDirectory,
        string? tipLinkForTool = null)
    {
        var indexed = new List<(int Index, string Name)>(tree.Links.Count);
        for (var i = 0; i < tree.Links.Count; i++)
            indexed.Add((i, tree.Links[i].Name));
        var model = LoadIndexed(robotRoot, indexed, urdfDirectory);
        if (string.IsNullOrWhiteSpace(tipLinkForTool))
            return model;
        var tool = LoadTipLinkGeometry(robotRoot, tipLinkForTool, urdfDirectory);
        return WithToolGeometry(model, tool);
    }

    private static RobotCollisionModel? LoadIndexed(
        XElement robotRoot,
        IReadOnlyList<(int Index, string Name)> links,
        string urdfDirectory)
    {
        var linksByName = robotRoot.Elements("link")
            .ToDictionary(l => l.Attribute("name")?.Value ?? "", l => l, StringComparer.OrdinalIgnoreCase);

        var geometries = new List<LinkCollisionGeometry>();
        foreach (var (index, linkName) in links)
        {
            if (!linksByName.TryGetValue(linkName, out var linkEl)) continue;
            var collisionIdx = 0;
            foreach (var collision in linkEl.Elements("collision"))
            {
                var origin = collision.Element("origin");
                var xyz = ParseTriple(origin?.Attribute("xyz")?.Value);
                var rpy = ParseTriple(origin?.Attribute("rpy")?.Value);
                var pose = FrameFromRpy(xyz.x, xyz.y, xyz.z, rpy.x, rpy.y, rpy.z);
                var geom = collision.Element("geometry") ?? throw new InvalidOperationException($"collision on {linkName} missing geometry");
                var objName = $"{linkName}_col{collisionIdx++}";
                var obj = ParseGeometry(objName, pose, geom, urdfDirectory);
                if (obj is not null)
                    geometries.Add(new LinkCollisionGeometry(index, linkName, obj));
            }
        }

        return geometries.Count == 0 ? null : new RobotCollisionModel(geometries);
    }

    /// <summary>Load collision from tip link (e.g. tool0) into TCP-local tool geometry.</summary>
    public static CollisionObject? LoadTipLinkGeometry(
        XElement robotRoot,
        string tipLinkName,
        string urdfDirectory)
    {
        var linkEl = robotRoot.Elements("link")
            .FirstOrDefault(l => string.Equals(l.Attribute("name")?.Value, tipLinkName, StringComparison.OrdinalIgnoreCase));
        if (linkEl is null) return null;
        var collision = linkEl.Elements("collision").FirstOrDefault();
        if (collision is null) return null;
        var origin = collision.Element("origin");
        var xyz = ParseTriple(origin?.Attribute("xyz")?.Value);
        var rpy = ParseTriple(origin?.Attribute("rpy")?.Value);
        var pose = FrameFromRpy(xyz.x, xyz.y, xyz.z, rpy.x, rpy.y, rpy.z);
        var geom = collision.Element("geometry") ?? throw new InvalidOperationException($"collision on {tipLinkName} missing geometry");
        return ParseGeometry($"{tipLinkName}_tool_col", pose, geom, urdfDirectory);
    }

    public static RobotCollisionModel? WithToolGeometry(RobotCollisionModel? model, CollisionObject? toolGeometry)
    {
        if (model is null && toolGeometry is null) return null;
        if (model is null) return new RobotCollisionModel(Array.Empty<LinkCollisionGeometry>(), toolGeometry);
        return new RobotCollisionModel(
            model.Links,
            toolGeometry ?? model.ToolGeometry,
            model.ToolGeometryInFlangeFrame,
            model.ToolGeometryAttachOffset);
    }

    private static CollisionObject? ParseGeometry(string name, Frame pose, XElement geom, string urdfDirectory)
    {
        if (geom.Element("box") is { } box)
        {
            var size = ParseTriple(box.Attribute("size")?.Value, 0.1, 0.1, 0.1);
            return CollisionObject.Box(name, pose, size.x / 2, size.y / 2, size.z / 2);
        }
        if (geom.Element("cylinder") is { } cyl)
        {
            var radius = ParseDouble(cyl.Attribute("radius")?.Value, 0.05);
            var length = ParseDouble(cyl.Attribute("length")?.Value, 0.1);
            return CollisionObject.Capsule(name, pose, radius, length / 2);
        }
        if (geom.Element("sphere") is { } sph)
        {
            var radius = ParseDouble(sph.Attribute("radius")?.Value, 0.05);
            return CollisionObject.Sphere(name, pose, radius);
        }
        if (geom.Element("mesh") is { } mesh)
        {
            var filename = mesh.Attribute("filename")?.Value;
            if (string.IsNullOrWhiteSpace(filename)) return null;
            var path = ResolveMeshPath(filename, urdfDirectory);
            var (vertices, indices) = StlReader.Read(path);
            return CollisionObject.Mesh(name, pose, vertices, indices);
        }
        return null;
    }

    private static string ResolveMeshPath(string filename, string urdfDirectory)
    {
        var cleaned = filename.Replace("package://", "").Replace("file://", "");
        var baseFull = Path.GetFullPath(urdfDirectory);
        var path = Path.IsPathRooted(cleaned)
            ? Path.GetFullPath(cleaned)
            : Path.GetFullPath(Path.Combine(urdfDirectory, cleaned));
        if (!File.Exists(path))
            path = Path.GetFullPath(Path.Combine(urdfDirectory, Path.GetFileName(cleaned)));
        if (!File.Exists(path))
            throw new FileNotFoundException($"URDF mesh not found: {filename}");
        if (!IsUnderDirectory(path, baseFull))
            throw new InvalidOperationException($"URDF mesh path escapes asset directory: {filename}");
        return path;
    }

    private static bool IsUnderDirectory(string path, string directory)
    {
        var fullPath = Path.GetFullPath(path);
        var fullDir = Path.GetFullPath(directory);
        if (string.Equals(fullPath, fullDir, StringComparison.OrdinalIgnoreCase))
            return true;
        var prefix = fullDir.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar)
            + Path.DirectorySeparatorChar;
        return fullPath.StartsWith(prefix, StringComparison.OrdinalIgnoreCase);
    }

    private static Frame FrameFromRpy(double x, double y, double z, double roll, double pitch, double yaw)
    {
        var m = Transforms.FromRpy(x, y, z, roll, pitch, yaw);
        return Transforms.ToFrame(m);
    }

    private static (double x, double y, double z) ParseTriple(string? s, double dx = 0, double dy = 0, double dz = 0)
    {
        if (string.IsNullOrWhiteSpace(s)) return (dx, dy, dz);
        var p = s.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        if (p.Length < 3) return (dx, dy, dz);
        return (ParseDouble(p[0], dx), ParseDouble(p[1], dy), ParseDouble(p[2], dz));
    }

    private static double ParseDouble(string? s, double fallback) =>
        double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) ? v : fallback;
}
