using System.Globalization;
using System.Xml.Linq;
using Motus.Core;
using Motus.Geometry;

namespace Motus.Presets;

/// <summary>Fixed-joint transforms along a URDF chain (e.g. wrist_3 → tool0).</summary>
public static class UrdfFixedChain
{
    /// <summary>Fixed URDF chain from the last actuated link to <paramref name="tipLink"/>.</summary>
    public static Frame? TryTipAttachOffset(string urdfPath, string baseLink = "base_link", string tipLink = "tool0")
    {
        if (!File.Exists(urdfPath))
            return null;
        return TryTipAttachOffset(XDocument.Load(urdfPath).Root, baseLink, tipLink);
    }

    /// <summary>Fixed URDF chain from the last actuated link to <paramref name="tipLink"/>.</summary>
    public static Frame? TryTipAttachOffset(XElement? robotRoot, string baseLink = "base_link", string tipLink = "tool0")
    {
        if (robotRoot is null)
            return null;

        var chain = BuildActuatedChainLinkNames(robotRoot, baseLink, tipLink);
        if (chain.Count == 0)
            return null;

        return ComposeFixedForwardChain(robotRoot, chain[^1], tipLink);
    }

    private static List<string> BuildActuatedChainLinkNames(XElement robotRoot, string baseLink, string tipLink)
    {
        var joints = robotRoot.Elements("joint")
            .Select(j => new
            {
                Type = (j.Attribute("type")?.Value ?? "fixed").Trim(),
                Parent = (j.Element("parent")?.Attribute("link")?.Value ?? "").Trim(),
                Child = (j.Element("child")?.Attribute("link")?.Value ?? "").Trim()
            })
            .Where(j => !string.IsNullOrWhiteSpace(j.Parent) && !string.IsNullOrWhiteSpace(j.Child))
            .ToList();

        var byChild = joints.ToDictionary(j => j.Child, j => j, StringComparer.OrdinalIgnoreCase);
        var path = new List<string>();
        var link = tipLink;
        var guard = 0;
        while (!string.Equals(link, baseLink, StringComparison.OrdinalIgnoreCase))
        {
            if (++guard > 128) return new List<string>();
            if (!byChild.TryGetValue(link, out var joint)) return new List<string>();
            if (!string.Equals(joint.Type, "fixed", StringComparison.OrdinalIgnoreCase))
                path.Add(joint.Child);
            link = joint.Parent;
        }

        path.Reverse();
        return path;
    }

    private static Frame ComposeFixedForwardChain(XElement robotRoot, string fromLink, string toLink)
    {
        if (string.Equals(fromLink, toLink, StringComparison.OrdinalIgnoreCase))
            return new Frame(0, 0, 0, 1, 0, 0, 0);

        var joints = robotRoot.Elements("joint")
            .Where(j => string.Equals(j.Attribute("type")?.Value, "fixed", StringComparison.OrdinalIgnoreCase))
            .Select(j => new
            {
                Parent = j.Element("parent")?.Attribute("link")?.Value ?? "",
                Child = j.Element("child")?.Attribute("link")?.Value ?? "",
                Origin = j.Element("origin")
            })
            .Where(j => !string.IsNullOrWhiteSpace(j.Parent) && !string.IsNullOrWhiteSpace(j.Child))
            .GroupBy(j => j.Parent, StringComparer.OrdinalIgnoreCase)
            .ToDictionary(g => g.Key, g => g.ToList(), StringComparer.OrdinalIgnoreCase);

        var queue = new Queue<(string Link, Frame Pose)>();
        queue.Enqueue((fromLink, new Frame(0, 0, 0, 1, 0, 0, 0)));
        var visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase) { fromLink };

        while (queue.Count > 0)
        {
            var (link, pose) = queue.Dequeue();
            if (string.Equals(link, toLink, StringComparison.OrdinalIgnoreCase))
                return pose;
            if (!joints.TryGetValue(link, out var children)) continue;
            foreach (var joint in children)
            {
                if (!visited.Add(joint.Child)) continue;
                var xyz = UrdfNumbers.Triple(joint.Origin?.Attribute("xyz")?.Value, 0, 0, 0, $"fixed joint {joint.Parent} → {joint.Child} origin xyz");
                var rpy = UrdfNumbers.Triple(joint.Origin?.Attribute("rpy")?.Value, 0, 0, 0, $"fixed joint {joint.Parent} → {joint.Child} origin rpy");
                var step = FrameFromRpy(xyz.x, xyz.y, xyz.z, rpy.x, rpy.y, rpy.z);
                queue.Enqueue((joint.Child, ComposeFrames(pose, step)));
            }
        }

        return new Frame(0, 0, 0, 1, 0, 0, 0);
    }

    private static Frame ComposeFrames(Frame parent, Frame local) =>
        Transforms.ToFrame(Transforms.Multiply(Transforms.FromFrame(parent), Transforms.FromFrame(local)));

    private static Frame FrameFromRpy(double x, double y, double z, double roll, double pitch, double yaw) =>
        Transforms.ToFrame(Transforms.FromRpy(x, y, z, roll, pitch, yaw));
}
