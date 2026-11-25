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

    /// <summary>
    /// The factory default (<see cref="CollisionCheckerFactory.DefaultTreeSelfCollision"/>): <see cref="AllowedPairs"/>
    /// with fixed siblings, plus pairs that touch by design: links joined only by fixed joints (they never move against
    /// each other, for example two bases fixed to the world), and links moved by one mimic group (gripper fingers).
    /// </summary>
    internal static IReadOnlyList<(string A, string B)> DefaultPairs(KinematicTree tree, IReadOnlySet<string> geometryLinks)
    {
        var parentJoint = new int[tree.Links.Count];
        Array.Fill(parentJoint, -1);
        for (var j = 0; j < tree.Joints.Count; j++)
            parentJoint[tree.Joints[j].ChildLinkIndex] = j;

        // Nearest moving joint above a link (-1 = the root's rigid body), and its mimic master.
        int Mover(int link)
        {
            for (var j = parentJoint[link]; j >= 0; j = parentJoint[tree.Joints[j].ParentLinkIndex])
                if (tree.Joints[j].Type != KinematicJointType.Fixed)
                    return j;
            return -1;
        }
        int Master(int j) => j >= 0 && tree.Joints[j].Mimic is { } m ? m.JointIndex : j;

        var links = tree.Links.Select((l, i) => (l.Name, i)).Where(l => geometryLinks.Contains(l.Name))
            .Select(l => (l.Name, Mover: Mover(l.i))).ToArray();
        var pairs = AllowedPairs(tree, geometryLinks, fixedSiblingsTouch: true).ToList();
        for (var a = 0; a < links.Length; a++)
            for (var b = a + 1; b < links.Length; b++)
            {
                int ma = links[a].Mover, mb = links[b].Mover;
                var rigid = ma == mb;
                var mimic = ma >= 0 && mb >= 0 && Master(ma) == Master(mb) && (tree.Joints[ma].Mimic is not null || tree.Joints[mb].Mimic is not null);
                if (rigid || mimic)
                    pairs.Add((links[a].Name, links[b].Name));
            }
        return pairs;
    }
}
