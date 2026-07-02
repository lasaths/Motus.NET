using System.Xml.Linq;
using Motus.Core;

namespace Motus.Presets;

/// <summary>Load SRDF disable_collisions pairs into CollisionScene.AllowedPairs.</summary>
public static class SrdfLoader
{
    public static IReadOnlyList<(string A, string B)> LoadAllowedPairs(string srdfPath)
    {
        var doc = XDocument.Load(srdfPath);
        return LoadAllowedPairs(doc);
    }

    public static IReadOnlyList<(string A, string B)> LoadAllowedPairs(XDocument doc)
    {
        var root = doc.Root ?? throw new InvalidOperationException("SRDF has no root.");
        return root.Elements("disable_collisions")
            .Select(el => (
                el.Attribute("link1")?.Value ?? "",
                el.Attribute("link2")?.Value ?? ""))
            .Where(p => !string.IsNullOrEmpty(p.Item1) && !string.IsNullOrEmpty(p.Item2))
            .ToList();
    }

    public static CollisionScene MergeAllowedPairs(CollisionScene scene, IReadOnlyList<(string A, string B)> srdfPairs, IReadOnlyDictionary<string, int>? linkNameToIndex = null)
    {
        var pairs = scene.AllowedPairs.ToList();
        foreach (var (a, b) in srdfPairs)
        {
            var bodyA = linkNameToIndex is not null && linkNameToIndex.TryGetValue(a, out var ia)
                ? CollisionBodies.RobotLink(ia) : a;
            var bodyB = linkNameToIndex is not null && linkNameToIndex.TryGetValue(b, out var ib)
                ? CollisionBodies.RobotLink(ib) : b;
            pairs.Add((bodyA, bodyB));
        }
        return new CollisionScene(scene.Objects.ToList(), pairs);
    }
}
