using System.Globalization;
using System.Text.Json;
using Motus.Core;

namespace Motus.Presets;

internal static class CollisionPresetLoader
{
    public static RobotCollisionModel? LoadFromDto(IReadOnlyList<CollisionLinkDto>? links, string presetFilePath)
    {
        if (links is null || links.Count == 0) return null;
        var presetDir = Path.GetDirectoryName(presetFilePath) ?? ".";
        var resourcesRoot = FindResourcesRoot(presetDir);
        var geometries = new List<LinkCollisionGeometry>();

        foreach (var link in links)
        {
            var name = $"link_{link.Link}";
            var obj = link.Shape.ToLowerInvariant() switch
            {
                "sphere" => CollisionObject.Sphere(name, Frame.Identity, link.Radius ?? 0.08),
                "box" => CollisionObject.Box(name, Frame.Identity, link.HalfX ?? 0.05, link.HalfY ?? 0.05, link.HalfZ ?? 0.05),
                "capsule" => CollisionObject.Capsule(name, Frame.Identity, link.Radius ?? 0.08, (link.Length ?? 0.12) * 0.5),
                "mesh" when link.File is not null => LoadMeshCollision(name, link.File, presetDir, resourcesRoot),
                _ => throw new InvalidOperationException($"Unsupported collisionLinks shape '{link.Shape}' on link {link.Link}.")
            };
            geometries.Add(new LinkCollisionGeometry(link.Link, name, obj));
        }

        return new RobotCollisionModel(geometries);
    }

    private static CollisionObject LoadMeshCollision(string name, string file, string presetDir, string? resourcesRoot)
    {
        var path = Path.IsPathRooted(file) ? file : Path.GetFullPath(Path.Combine(presetDir, file));
        if (!File.Exists(path) && resourcesRoot is not null)
            path = Path.GetFullPath(Path.Combine(resourcesRoot, file));
        if (!File.Exists(path))
            throw new FileNotFoundException($"Collision mesh not found: {file}");

        var (vertices, indices) = StlReader.Read(path);
        return CollisionObject.Mesh(name, Frame.Identity, vertices, indices);
    }

    private static string? FindResourcesRoot(string startDir)
    {
        var dir = startDir;
        for (var i = 0; i < 8 && dir is not null; i++)
        {
            var candidate = Path.Combine(dir, "resources", "robots");
            if (Directory.Exists(candidate)) return candidate;
            dir = Directory.GetParent(dir)?.FullName;
        }
        return null;
    }

    internal sealed class CollisionLinkDto
    {
        public int Link { get; set; }
        public string Shape { get; set; } = "capsule";
        public double? Radius { get; set; }
        public double? Length { get; set; }
        public double? HalfX { get; set; }
        public double? HalfY { get; set; }
        public double? HalfZ { get; set; }
        public string? File { get; set; }
    }
}

internal static class StlReader
{
    public static (List<double[]> vertices, List<int> indices) Read(string path)
    {
        if (IsAsciiStl(path))
            return ReadAscii(File.ReadAllLines(path));
        return ReadBinary(File.ReadAllBytes(path));
    }

    private static bool IsAsciiStl(string path)
    {
        using var sr = new StreamReader(path);
        var header = sr.ReadLine() ?? "";
        return header.TrimStart().StartsWith("solid", StringComparison.OrdinalIgnoreCase);
    }

    private static (List<double[]>, List<int>) ReadAscii(string[] lines)
    {
        var vertices = new List<double[]>();
        var indices = new List<int>();
        var map = new Dictionary<string, int>();
        double[]? v0 = null; double[]? v1 = null;

        foreach (var raw in lines)
        {
            var line = raw.Trim();
            if (line.StartsWith("vertex", StringComparison.OrdinalIgnoreCase))
            {
                var p = line.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
                if (p.Length < 4) continue;
                var v = new[] { Parse(p[1]), Parse(p[2]), Parse(p[3]) };
                var key = $"{v[0]:F6},{v[1]:F6},{v[2]:F6}";
                if (!map.TryGetValue(key, out var idx))
                {
                    idx = vertices.Count;
                    vertices.Add(v);
                    map[key] = idx;
                }
                if (v0 is null) v0 = v;
                else if (v1 is null) v1 = v;
                else
                {
                    indices.Add(map[$"{v0[0]:F6},{v0[1]:F6},{v0[2]:F6}"]);
                    indices.Add(map[$"{v1[0]:F6},{v1[1]:F6},{v1[2]:F6}"]);
                    indices.Add(idx);
                    v0 = v1 = null;
                }
            }
            else if (line.StartsWith("endloop", StringComparison.OrdinalIgnoreCase))
                v0 = v1 = null;
        }
        return (vertices, indices);
    }

    private static (List<double[]>, List<int>) ReadBinary(byte[] data)
    {
        if (data.Length < 84) throw new InvalidOperationException("Invalid binary STL.");
        var triCount = BitConverter.ToUInt32(data, 80);
        var vertices = new List<double[]>();
        var indices = new List<int>();
        var map = new Dictionary<string, int>();
        var offset = 84;
        for (var t = 0; t < triCount; t++)
        {
            offset += 12;
            for (var v = 0; v < 3; v++)
            {
                var x = BitConverter.ToSingle(data, offset); offset += 4;
                var y = BitConverter.ToSingle(data, offset); offset += 4;
                var z = BitConverter.ToSingle(data, offset); offset += 4;
                var key = $"{x:F6},{y:F6},{z:F6}";
                if (!map.TryGetValue(key, out var idx))
                {
                    idx = vertices.Count;
                    vertices.Add(new[] { (double)x, (double)y, (double)z });
                    map[key] = idx;
                }
                indices.Add(idx);
            }
            offset += 2;
        }
        return (vertices, indices);
    }

    private static double Parse(string s) =>
        double.Parse(s, CultureInfo.InvariantCulture);
}
