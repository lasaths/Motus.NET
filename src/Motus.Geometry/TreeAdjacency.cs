namespace Motus.Geometry;

/// <summary>Link pairs of a <see cref="KinematicTree"/> that may touch by design (input for <see cref="TreeSelfCollision"/>).</summary>
public static class TreeAdjacency
{
    /// <summary>
    /// Each geometry link with its nearest geometry ancestor (links with no geometry, for example a turntable frame,
    /// are walked through). With <paramref name="fixedSiblingsTouch"/>, two geometry children of the same geometry
    /// ancestor are also allowed when at least one of them is on a fixed joint (a clamp fixed beside a moving jaw);
    /// two moving siblings are never allowed. Each pair is listed once.
    /// </summary>
    public static IReadOnlyList<(string A, string B)> AllowedPairs(
        KinematicTree tree, IReadOnlySet<string> geometryLinks, bool fixedSiblingsTouch)
    {
        var parentOf = new int[tree.Links.Count];
        Array.Fill(parentOf, -1);
        foreach (var j in tree.Joints)
            parentOf[j.ChildLinkIndex] = j.ParentLinkIndex;

        string? GeometryAncestor(int link)
        {
            for (var i = link; i >= 0; i = parentOf[i])
            {
                if (geometryLinks.Contains(tree.Links[i].Name))
                    return tree.Links[i].Name;
                if (parentOf[i] == i)
                    break;
            }
            return null;
        }

        var pairs = new List<(string A, string B)>();
        var kids = new Dictionary<string, List<(string Name, bool Fixed)>>(StringComparer.Ordinal);
        foreach (var j in tree.Joints)
        {
            var child = tree.Links[j.ChildLinkIndex].Name;
            if (!geometryLinks.Contains(child))
                continue;
            var parent = GeometryAncestor(j.ParentLinkIndex);
            if (parent is null || parent == child)
                continue;
            if (!pairs.Exists(p => p.A == parent && p.B == child || p.A == child && p.B == parent))
                pairs.Add((parent, child));
            if (!kids.TryGetValue(parent, out var list))
                kids[parent] = list = new List<(string, bool)>();
            list.Add((child, j.Type == KinematicJointType.Fixed));
        }

        if (!fixedSiblingsTouch)
            return pairs;
        foreach (var list in kids.Values)
            for (var a = 0; a < list.Count; a++)
                for (var b = a + 1; b < list.Count; b++)
                    if (list[a].Fixed || list[b].Fixed)
                        pairs.Add((list[a].Name, list[b].Name));
        return pairs;
    }
}
