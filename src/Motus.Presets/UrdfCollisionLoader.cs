using System.Globalization;
using System.Xml.Linq;
using Motus.Core;
using Motus.Geometry;

namespace Motus.Presets;

internal static class UrdfCollisionLoader
{
    public static RobotCollisionModel? Load(
        XElement robotRoot,
        IReadOnlyList<string> chainLinkNames,
        string urdfDirectory)
    {
        var linksByName = robotRoot.Elements("link")
            .ToDictionary(l => l.Attribute("name")?.Value ?? "", l => l, StringComparer.OrdinalIgnoreCase);

        var geometries = new List<LinkCollisionGeometry>();
        for (var i = 0; i < chainLinkNames.Count; i++)
        {
            var linkName = chainLinkNames[i];
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
                    geometries.Add(new LinkCollisionGeometry(i, linkName, obj));
            }
        }

        return geometries.Count == 0 ? null : new RobotCollisionModel(geometries);
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
        var path = Path.IsPathRooted(cleaned) ? cleaned : Path.GetFullPath(Path.Combine(urdfDirectory, cleaned));
        if (!File.Exists(path))
            path = Path.GetFullPath(Path.Combine(urdfDirectory, Path.GetFileName(cleaned)));
        if (!File.Exists(path))
            throw new FileNotFoundException($"URDF mesh not found: {filename}");
        return path;
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
