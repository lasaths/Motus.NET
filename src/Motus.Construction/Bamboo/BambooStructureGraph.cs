namespace Motus.Construction;

/// <summary>
/// The truss as a graph: nodes (with connectors), members (edges between nodes) and anchors (nodes fixed to the
/// ground, with a clamp column). "Fixed / walkable" is computed from it: a member may be clamped once both its end
/// nodes lie in one rigid group that reaches an anchor (a closed triangle grown by nodes tied to it by two members).
/// </summary>
public sealed class BambooStructureGraph
{
    public sealed record Node(string Name, double X, double Y, double Z, bool IsAnchor, string? AnchorColumn);
    public sealed record Edge(string Id, string A, string B);

    public IReadOnlyList<Node> Nodes { get; }
    public IReadOnlyList<Edge> Edges { get; }
    public IReadOnlySet<string> Anchors { get; }

    public BambooStructureGraph(IEnumerable<Node> nodes, IEnumerable<Edge> edges)
    {
        Nodes = nodes.ToList();
        Edges = edges.ToList();
        Anchors = Nodes.Where(n => n.IsAnchor).Select(n => n.Name).ToHashSet(StringComparer.Ordinal);
        foreach (var e in Edges)
            if (!Nodes.Any(n => n.Name == e.A) || !Nodes.Any(n => n.Name == e.B))
                throw new ArgumentException($"member {e.Id}: unknown end node");
    }

    /// <summary>The current layout's graph: foot nodes, members, and the anchor columns ("anchor-X" ⇒ node X).</summary>
    public static BambooStructureGraph FromLayout()
    {
        var columns = BambooTrussLayout.Stakes.ToDictionary(s => s.Name.StartsWith("anchor-", StringComparison.Ordinal) ? s.Name["anchor-".Length..] : s.Name, s => s.Name);
        return new BambooStructureGraph(
            BambooTrussLayout.FootNodes.Select(n => new Node(n.Name, n.X, n.Y, n.Z, columns.ContainsKey(n.Name), columns.GetValueOrDefault(n.Name))),
            BambooTrussLayout.Members.Select(m => new Edge(m.Id, m.NodeA, m.NodeB)));
    }

    public Edge EdgeOf(string id) => Edges.Single(e => e.Id == id);

    /// <summary>Nodes that exist: the anchors and every end node of a placed member.</summary>
    public HashSet<string> ExistingNodes(IEnumerable<string> placed)
    {
        var set = new HashSet<string>(Anchors, StringComparer.Ordinal);
        var ids = placed.ToHashSet(StringComparer.Ordinal);
        foreach (var e in Edges.Where(e => ids.Contains(e.Id))) { set.Add(e.A); set.Add(e.B); }
        return set;
    }

    /// <summary>A member can be started (one end on something that exists).</summary>
    public bool CanStart(string id, IEnumerable<string> placed)
    {
        var e = EdgeOf(id);
        var ex = ExistingNodes(placed);
        return ex.Contains(e.A) || ex.Contains(e.B);
    }

    /// <summary>
    /// Rigid groups of the placed members that reach an anchor. Seeded by every closed triangle, grown by every node tied
    /// to the group by two placed members; groups sharing two nodes merge. Returns their node sets.
    /// </summary>
    public List<HashSet<string>> AnchoredRigidGroups(IEnumerable<string> placed)
    {
        var ids = placed.ToHashSet(StringComparer.Ordinal);
        var edges = Edges.Where(e => ids.Contains(e.Id)).Select(e => (e.A, e.B)).ToList();
        bool Has(string a, string b) => edges.Any(e => (e.A == a && e.B == b) || (e.A == b && e.B == a));
        var nodes = edges.SelectMany(e => new[] { e.A, e.B }).Distinct().ToList();
        var groups = new List<HashSet<string>>();
        for (var i = 0; i < nodes.Count; i++)
        for (var j = i + 1; j < nodes.Count; j++)
        for (var k = j + 1; k < nodes.Count; k++)
            if (Has(nodes[i], nodes[j]) && Has(nodes[j], nodes[k]) && Has(nodes[i], nodes[k]))
                groups.Add(new HashSet<string>(StringComparer.Ordinal) { nodes[i], nodes[j], nodes[k] });
        for (var changed = true; changed;)
        {
            changed = false;
            foreach (var c in groups)
                foreach (var n in nodes.Where(n => !c.Contains(n)))
                    if (c.Count(x => Has(x, n)) >= 2) { c.Add(n); changed = true; }
            for (var a = 0; a < groups.Count && !changed; a++)
            for (var b = a + 1; b < groups.Count && !changed; b++)
                if (groups[a].Count(groups[b].Contains) >= 2) { groups[a].UnionWith(groups[b]); groups.RemoveAt(b); changed = true; }
        }
        return groups.Where(c => c.Overlaps(Anchors)).ToList();
    }

    /// <summary>Fixed nodes: the anchors plus every node of an anchored rigid group.</summary>
    public HashSet<string> FixedNodes(IEnumerable<string> placed)
    {
        var set = new HashSet<string>(Anchors, StringComparer.Ordinal);
        foreach (var c in AnchoredRigidGroups(placed)) set.UnionWith(c);
        return set;
    }

    /// <summary>Walkable (clampable) members: placed, both end nodes in one anchored rigid group. Graph order.</summary>
    public IReadOnlyList<string> Fixed(IEnumerable<string> placed)
    {
        var list = placed.ToList();
        var groups = AnchoredRigidGroups(list);
        return Edges.Where(e => list.Contains(e.Id)).Where(e => groups.Any(c => c.Contains(e.A) && c.Contains(e.B))).Select(e => e.Id).ToList();
    }

    /// <summary>Members not yet placed that can be started now (graph order).</summary>
    public IReadOnlyList<string> Frontier(IEnumerable<string> placed)
    {
        var list = placed.ToList();
        return Edges.Where(e => !list.Contains(e.Id) && CanStart(e.Id, list)).Select(e => e.Id).ToList();
    }
}
