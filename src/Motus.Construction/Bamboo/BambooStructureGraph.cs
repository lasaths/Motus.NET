namespace Motus.Construction;

/// <summary>
/// The truss as a graph: nodes (with connectors), members (edges between nodes) and anchors (nodes fixed to the
/// ground, with a clamp column). "Fixed / walkable" is computed from it: a member may be clamped once both its end
/// nodes lie in one rigid group that reaches an anchor. Planar structure (<see cref="IsPlanar"/>): a closed triangle
/// grown by nodes tied to it by two members. Spatial structure: the anchors are pinned ground and a node is fixed when the
/// placed bars hold it to first order (rigidity-matrix rank: a node tied to fixed nodes by three non-coplanar members,
/// and also rings that only brace each other).
/// </summary>
public sealed class BambooStructureGraph
{
    public sealed record Node(string Name, double X, double Y, double Z, bool IsAnchor, string? AnchorColumn);
    public sealed record Edge(string Id, string A, string B);

    public IReadOnlyList<Node> Nodes { get; }
    public IReadOnlyList<Edge> Edges { get; }
    public IReadOnlySet<string> Anchors { get; }
    /// <summary>Every node within <see cref="PlanarTolM"/> of one plane: the planar (2D) rigidity rule applies, else the spatial (3D) one.</summary>
    public bool IsPlanar { get; }

    /// <summary>Distance from the nodes' plane up to which a structure counts as planar (metres).</summary>
    public const double PlanarTolM = 1e-3;

    public BambooStructureGraph(IEnumerable<Node> nodes, IEnumerable<Edge> edges)
    {
        Nodes = nodes.ToList();
        Edges = edges.ToList();
        Anchors = Nodes.Where(n => n.IsAnchor).Select(n => n.Name).ToHashSet(StringComparer.Ordinal);
        foreach (var e in Edges)
            if (!Nodes.Any(n => n.Name == e.A) || !Nodes.Any(n => n.Name == e.B))
                throw new ArgumentException($"member {e.Id}: unknown end node");
        IsPlanar = Planar(Nodes);
    }

    static bool Planar(IReadOnlyList<Node> nodes)
    {
        if (nodes.Count < 4) return true;
        static (double X, double Y, double Z) Sub(Node a, Node b) => (a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        static (double X, double Y, double Z) Cross((double X, double Y, double Z) a, (double X, double Y, double Z) b) =>
            (a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
        static double Norm((double X, double Y, double Z) a) => Math.Sqrt(a.X * a.X + a.Y * a.Y + a.Z * a.Z);
        var o = nodes[0];
        var far = nodes.MaxBy(n => Norm(Sub(n, o)))!;
        var normal = nodes.Select(n => Cross(Sub(far, o), Sub(n, o))).MaxBy(Norm);
        var len = Norm(normal);
        if (len < 1e-12) return true; // collinear
        return nodes.All(n => Math.Abs((Sub(n, o).X * normal.X + Sub(n, o).Y * normal.Y + Sub(n, o).Z * normal.Z) / len) <= PlanarTolM);
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
    /// Rigid groups of the placed members that reach an anchor. Planar: seeded by every closed triangle, grown by every
    /// node tied to the group by two placed members; groups sharing two nodes merge. Spatial: one group, the anchors
    /// and every node the placed bars hold to first order (<see cref="SpatialFixed"/>). Returns their node sets.
    /// </summary>
    public List<HashSet<string>> AnchoredRigidGroups(IEnumerable<string> placed)
    {
        var ids = placed.ToHashSet(StringComparer.Ordinal);
        if (!IsPlanar) return new List<HashSet<string>> { SpatialFixed(ids) };
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

    /// <summary>
    /// Spatial rule: anchors are pinned ground; a node is fixed when every first-order motion of the placed bar
    /// framework leaves it still, i.e. its three coordinates lie in the row space of the rigidity matrix (exact Gaussian
    /// elimination, pivots below <see cref="RankTol"/> count as zero).
    /// </summary>
    HashSet<string> SpatialFixed(HashSet<string> ids)
    {
        var at = Nodes.ToDictionary(n => n.Name, StringComparer.Ordinal);
        var edges = Edges.Where(e => ids.Contains(e.Id)).ToList();
        var free = edges.SelectMany(e => new[] { e.A, e.B }).Distinct().Where(n => !Anchors.Contains(n)).ToList();
        var col = free.Select((n, i) => (n, i)).ToDictionary(x => x.n, x => 3 * x.i, StringComparer.Ordinal);
        var rows = new List<double[]>();
        foreach (var e in edges)
        {
            var (a, b) = (at[e.A], at[e.B]);
            double x = b.X - a.X, y = b.Y - a.Y, z = b.Z - a.Z, l = Math.Sqrt(x * x + y * y + z * z);
            var r = new double[3 * free.Count];
            if (col.TryGetValue(e.A, out var ca)) { r[ca] = -x / l; r[ca + 1] = -y / l; r[ca + 2] = -z / l; }
            if (col.TryGetValue(e.B, out var cb)) { r[cb] = x / l; r[cb + 1] = y / l; r[cb + 2] = z / l; }
            rows.Add(r);
        }
        var basis = Echelon(rows);
        var set = new HashSet<string>(Anchors, StringComparer.Ordinal);
        foreach (var n in free)
            if (Enumerable.Range(0, 3).All(k => { var v = new double[3 * free.Count]; v[col[n] + k] = 1; return InSpan(basis, v); }))
                set.Add(n);
        return set;
    }

    /// <summary>Pivot magnitude below which a rigidity-matrix row counts as dependent.</summary>
    public const double RankTol = 1e-9;

    /// <summary>Row echelon basis (pivot column, normalised row) of <paramref name="rows"/>.</summary>
    static List<(int Pivot, double[] Row)> Echelon(List<double[]> rows)
    {
        var basis = new List<(int, double[])>();
        foreach (var r0 in rows)
        {
            var r = Reduce(basis, (double[])r0.Clone());
            var p = Array.FindIndex(r, v => Math.Abs(v) > RankTol);
            if (p < 0) continue;
            var piv = r[p];
            for (var k = 0; k < r.Length; k++) r[k] /= piv;
            for (var i = 0; i < basis.Count; i++)
            {
                var (bp, br) = basis[i];
                var f = br[p];
                if (f != 0) for (var k = 0; k < r.Length; k++) br[k] -= f * r[k];
            }
            basis.Add((p, r));
        }
        return basis;
    }

    static double[] Reduce(List<(int Pivot, double[] Row)> basis, double[] v)
    {
        foreach (var (p, row) in basis)
        {
            var f = v[p];
            if (f != 0) for (var k = 0; k < v.Length; k++) v[k] -= f * row[k];
        }
        return v;
    }

    static bool InSpan(List<(int Pivot, double[] Row)> basis, double[] v) => Reduce(basis, v).All(x => Math.Abs(x) <= 1e-7);

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
