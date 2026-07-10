using System.Globalization;
using System.Text.Json;
using Motus.Core;

namespace Motus.Presets;

internal static class CollisionPresetLoader
{
    public static RobotCollisionModel? LoadFromDto(
        IReadOnlyList<CollisionLinkDto>? links,
        ToolCollisionDto? tool,
        string presetFilePath)
    {
        if ((links is null || links.Count == 0) && tool is null) return null;
        var presetDir = Path.GetDirectoryName(presetFilePath) ?? ".";
        var resourcesRoot = FindResourcesRoot(presetDir);
        var geometries = new List<LinkCollisionGeometry>();

        if (links is not null)
        {
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
        }

        var toolGeom = tool is not null ? LoadToolCollision(tool, presetDir, resourcesRoot) : null;
        return new RobotCollisionModel(geometries, toolGeom);
    }

    private static CollisionObject LoadToolCollision(ToolCollisionDto tool, string presetDir, string? resourcesRoot)
    {
        const string name = "tool_collision";
        return tool.Shape.ToLowerInvariant() switch
        {
            "sphere" => CollisionObject.Sphere(name, Frame.Identity, tool.Radius ?? 0.04),
            "box" => CollisionObject.Box(name, Frame.Identity, tool.HalfX ?? 0.03, tool.HalfY ?? 0.03, tool.HalfZ ?? 0.04),
            "capsule" => CollisionObject.Capsule(name, Frame.Identity, tool.Radius ?? 0.03, (tool.Length ?? 0.08) * 0.5),
            "mesh" when tool.File is not null => LoadMeshCollision(name, tool.File, presetDir, resourcesRoot),
            _ => throw new InvalidOperationException($"Unsupported toolCollision shape '{tool.Shape}'.")
        };
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

    internal sealed class ToolCollisionDto
    {
        public string Shape { get; set; } = "box";
        public double? Radius { get; set; }
        public double? Length { get; set; }
        public double? HalfX { get; set; }
        public double? HalfY { get; set; }
        public double? HalfZ { get; set; }
        public string? File { get; set; }
    }
}
