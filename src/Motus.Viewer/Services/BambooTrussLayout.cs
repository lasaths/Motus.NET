using Motus.Core;
using K = Motus.Viewer.Services.BambooGripKinematics;

namespace Motus.Viewer.Services;

/// <summary>
/// Store/truss member poses and fixed ground fixtures for /bamboo-truss (viewer coordinates, Y-up).
/// <para>
/// Three-triangle Warren truss over a U-shaped pit, every member in ONE vertical plane (z = <see cref="PlaneZ"/>):
/// <code>
///        T1 ———— T2        bottom chord A–M, M–D; diagonals A–T1, T1–M, M–T2, T2–D; top chord T1–T2
///       /  \    /  \       all triangles equilateral with panel length P (node to node)
///      A ——— M ——— D       pin support at A, roller at D (support columns at the pit edges)
///  ____|           |____   ground at support level outside A and D
///      |___________|       pit below the span (two walls + floor): no earth inside the span
/// </code>
/// Node connectors (cubes, <see cref="HubHalf"/>) sit at every node; each cane is P − 2·<see cref="Site.Gap"/> long and
/// ends a fixed gap short of the node centre, so canes never touch each other or the hubs.
/// </para>
/// <para>
/// <b>No stakes.</b> The robot starts with its L-grip clamped on the pin-support column under node A. The only fixed
/// things it may clamp are the anchor columns at A and D and members fixed into the structure (both end nodes in a
/// rigid, triangulated subframe that contains an anchor, see <see cref="FixedBothEnds"/>).
/// </para>
/// </summary>
public static class BambooTrussLayout
{
    // ---------------- site (cane length, supports, stock) ----------------

    /// <summary>A vertical clamp support (slot axis = support axis = viewer +Y): the anchor columns at A and D.</summary>
    public sealed record Stake(string Name, double X, double Z, double ClampY, double Top, double HeadingDeg)
    {
        /// <summary>Grip target on the support at <see cref="ClampY"/>.</summary>
        public K.GripTarget Grip => new(X, ClampY, Z, 0, 1, 0, Name);
        public K.GripTarget At(double y) => new(X, y, Z, 0, 1, 0, Name);
    }

    /// <summary>
    /// Everything that depends on the chosen cane length: cane length, node gap, node height (support level), pit
    /// depth, the clamp supports (anchor columns), the stock and the walk.
    /// </summary>
    public sealed record Site(
        double Length,
        double Gap,
        double NodeY,
        double PitDepth,
        IReadOnlyList<Stake> Stakes,
        string StartStake,
        double FeedX, double FeedZ, double FeedYawDeg, double FeedTop,
        string WalkRAnchor, string WalkLTarget,
        IReadOnlyDictionary<string, string> Unreachable);

    public static Site Default { get; } = SiteDefaults.Chosen;

    public static Site Current { get; private set; } = Default;

    /// <summary>Switch the site (offline sweeps / tests). Clears every derived cache.</summary>
    public static void Use(Site site)
    {
        Current = site;
        _fixtures = null; _members = null; _hubs.Clear();
    }

    /// <summary>Cane length (metres): panel − 2 × node gap, the same for every member.</summary>
    public static double StrutLength => Current.Length;
    /// <summary>Node gap: each cane end stops this far short of the node centre (the connector bridges it).</summary>
    public static double NodeGap => Current.Gap;
    /// <summary>Panel length (node to node) of every equilateral triangle.</summary>
    public static double Panel => Current.Length + 2 * Current.Gap;
    public static double Side => Panel;
    /// <summary>Truss height: P·√3/2.</summary>
    public static double Height => Panel * Math.Sqrt(3) / 2;

    public const double JawClosedMm = 14;
    public const double JawOpenMm = 40;
    /// <summary>L-grip fully open (jaw plates 15 mm off a 14 mm cane / column).</summary>
    public const double LJawOpenMm = 45;

    /// <summary>The one vertical plane of every member axis (viewer Z).</summary>
    public const double PlaneZ = 0.11;
    /// <summary>Support level: node height of A, M, D above the ground outside the pit.</summary>
    public static double FootY0 => Current.NodeY;
    /// <summary>|x| of A and D (= one panel).</summary>
    public static double HalfSpan => Panel;
    /// <summary>Node connector cube half-size.</summary>
    public const double HubHalf = 0.015;
    /// <summary>Anchor (pin / roller support) column radius: 14 mm, the L clamp's closed width.</summary>
    public const double AnchorPostRadius = 0.007;
    /// <summary>Pit edge: the pit starts this far inboard of the A / D column axis (the column stands on the ground).</summary>
    public const double PitEdgeInset = 0.012;
    /// <summary>Ground slabs outside the pit: how far they reach outboard of the pit edge and either side of the plane.</summary>
    public const double GroundReach = 0.55, GroundHalfZ = 0.45;
    public const double EarthWall = 0.03;

    public const double StakeRadius = AnchorPostRadius;
    /// <summary>How far above a support's free top an L clamp clears (not used by the anchor columns: hub above).</summary>
    public const double StakeSlideClear = 0.018;
    public const double StakeTopAbove = 0.02;
    public const double StakePadHalf = 0.0;

    public static IReadOnlyList<Stake> Stakes => Current.Stakes;
    /// <summary>True for a clamp support (anchor column) rather than a cane.</summary>
    public static bool IsSupport(string? name) => name is not null && (name.StartsWith("anchor-", StringComparison.Ordinal) || name.StartsWith("stake-", StringComparison.Ordinal));
    public static Stake StakeNamed(string name) => Current.Stakes.Single(s => s.Name == name);
    /// <summary>Start support (L-grip clamped at t = 0): the anchor column at A.</summary>
    public static Stake S1 => StakeNamed(Current.StartStake);
    public static Stake WalkTarget => StakeNamed(Current.WalkLTarget);

    /// <summary>Exact L-grip clamp frame on <see cref="S1"/> (viewer 4×4): the IK base at t = 0.</summary>
    public static double[] StartBase => K.StakeClampFrame(S1.X, S1.ClampY, S1.Z, S1.HeadingDeg);

    /// <summary>Stock: the cane lies flat next to A (outside the pit) on two narrow pegs; 0.5 mm seat gap.</summary>
    public static double RackTop => Current.FeedTop;
    public const double RackHalfAlong = 0.015;
    public const double RackHalfAcross = 0.004;
    public static double RackPegOffset => StrutLength / 2 - 0.085;
    public static double[] StockGraspOffsets
    {
        get
        {
            var e = StrutLength / 2;
            var inner = RackPegOffset - 0.035;
            return new[] { -(e - 0.025), e - 0.025, -(e - 0.045), e - 0.045, 0.0, -inner, inner }.Select(v => Math.Round(v, 3)).ToArray();
        }
    }
    public static double FeedX => Current.FeedX;
    public static double FeedZ => Current.FeedZ;
    public const double RackSeatGap = 0.0005;
    public static double StrutY => RackTop + 0.007 + RackSeatGap;
    static (double X, double Z) FeedAxis => (Math.Sin(Current.FeedYawDeg * Math.PI / 180), Math.Cos(Current.FeedYawDeg * Math.PI / 180));
    static bool FeedAlongX => Math.Abs(FeedAxis.X) > 0.5;

    static double PitEdge => HalfSpan - PitEdgeInset;

    /// <summary>Ground surfaces at support level (top y = 0) outside the pit, west of A and east of D — no earth between.</summary>
    public static IReadOnlyList<(string Name, double X, double Z, double HalfX, double HalfZ)> Pads =>
    [
        ("ground-W", -PitEdge - GroundReach / 2, PlaneZ, GroundReach / 2, GroundHalfZ),
        ("ground-E", PitEdge + GroundReach / 2, PlaneZ, GroundReach / 2, GroundHalfZ)
    ];

    /// <summary>Pit walls and floor (and the earth under the ground surfaces) as Motus boxes: name, centre, half extents.</summary>
    public static IReadOnlyList<(string Name, double X, double Y, double Z, double HalfX, double HalfY, double HalfZ)> PitBoxes
    {
        get
        {
            var d = Current.PitDepth;
            return
            [
                ("pit-wall-W", -PitEdge - EarthWall / 2, -d / 2, PlaneZ, EarthWall / 2, d / 2, GroundHalfZ),
                ("pit-wall-E", PitEdge + EarthWall / 2, -d / 2, PlaneZ, EarthWall / 2, d / 2, GroundHalfZ),
                ("pit-floor", 0, -d - EarthWall / 2, PlaneZ, PitEdge + EarthWall, EarthWall / 2, GroundHalfZ)
            ];
        }
    }

    /// <summary>Anchor columns (pin at A, roller at D) as vertical cylinders: (name, x, z, top, radius) — clampable.</summary>
    public static IReadOnlyList<(string Name, double X, double Z, double Top, double Radius)> Posts =>
        Stakes.Select(st => (st.Name, st.X, st.Z, st.Top, AnchorPostRadius)).ToList();

    public static IReadOnlyList<(string Name, double X, double Z, double HalfX, double HalfZ, double Top)> Racks
    {
        get
        {
            var (ax, az) = FeedAxis; var o = RackPegOffset;
            var hx = FeedAlongX ? RackHalfAlong : RackHalfAcross; var hz = FeedAlongX ? RackHalfAcross : RackHalfAlong;
            return
            [
                ("store-peg-1", FeedX - ax * o, FeedZ - az * o, hx, hz, RackTop),
                ("store-peg-2", FeedX + ax * o, FeedZ + az * o, hx, hz, RackTop)
            ];
        }
    }

    /// <summary>
    /// Every ground object (ground surfaces, columns, pegs) as an x-extent. Only the A/D anchor columns may touch the
    /// span edge; everything else lies at x ≤ −PitEdge or x ≥ +PitEdge (the pit floor/walls are below ground, not earth
    /// the robot could stand on).
    /// </summary>
    public static IEnumerable<(string Name, double MinX, double MaxX, bool IsAnchor)> GroundXExtents() =>
        Pads.Select(p => (p.Name, p.X - p.HalfX, p.X + p.HalfX, false))
            .Concat(Posts.Select(p => (p.Name, p.X - p.Radius, p.X + p.Radius, true)))
            .Concat(Racks.Select(r => (r.Name, r.X - r.HalfX, r.X + r.HalfX, false)));

    static IReadOnlyList<CollisionObject>? _fixtures;
    /// <summary>Motus objects for the anchor columns, stock pegs, pit walls/floor and the A / D connectors (cached per site).</summary>
    public static IReadOnlyList<CollisionObject> FixtureObjects =>
        _fixtures ??= Posts.Select(p => BambooIcdCell.PostAt(p.Name, p.X, p.Z, 0, p.Top, p.Radius))
            .Concat(Racks.Select(r => BambooIcdCell.BoxAt(r.Name, r.X, r.Top / 2, r.Z, r.HalfX, r.Top / 2, r.HalfZ)))
            .Concat(PitBoxes.Select(b => BambooIcdCell.BoxAt(b.Name, b.X, b.Y, b.Z, b.HalfX, b.HalfY, b.HalfZ)))
            .Concat(new[] { "A", "D" }.Select(HubObject))
            .ToList();

    static CollisionObject HubObject(string node)
    {
        var n = FootNodes.First(f => f.Name == node);
        return BambooIcdCell.BoxAt("hub-" + node, n.X, n.Y, n.Z, HubHalf, HubHalf, HubHalf);
    }

    static readonly Dictionary<string, IReadOnlyList<CollisionObject>> _hubs = new();
    /// <summary>
    /// Connectors at the inner nodes (M, T1, T2) that exist once a placed member reaches them (A / D are part of the
    /// supports, in <see cref="FixtureObjects"/>). Motus obstacles; the cached list is per node set.
    /// </summary>
    public static IReadOnlyList<CollisionObject> HubObjects(IEnumerable<string> placedIds)
    {
        var nodes = InnerHubNodes(placedIds);
        var key = string.Join(",", nodes);
        if (!_hubs.TryGetValue(key, out var list)) _hubs[key] = list = nodes.Select(HubObject).ToList();
        return list;
    }

    /// <summary>Inner nodes with a connector for <paramref name="placedIds"/> (sorted).</summary>
    public static List<string> InnerHubNodes(IEnumerable<string> placedIds)
    {
        var ids = placedIds.ToHashSet(StringComparer.Ordinal);
        return Members.Where(m => ids.Contains(m.Id)).SelectMany(m => new[] { m.NodeA, m.NodeB })
            .Where(n => n is not "A" and not "D").Distinct().OrderBy(n => n, StringComparer.Ordinal).ToList();
    }

    /// <summary>Configure a cell for /bamboo-truss: no hang rod, the two ground surfaces as pads.</summary>
    public static void Configure(BambooIcdCell cell)
    {
        cell.IncludePole = false;
        cell.SetGroundPads(Pads);
    }

    public sealed record MemberDef(
        string Id,
        string Label,
        string Glyph,
        BambooIcdCell.StrutPlacement Store,
        BambooIcdCell.StrutPlacement Place,
        (double X, double Y, double Z) EdgeA,
        (double X, double Y, double Z) EdgeB,
        string NodeA = "",
        string NodeB = "");

    /// <summary>Stock feed: cane flat on its pegs next to anchor A.</summary>
    public static BambooIcdCell.StrutPlacement Store => new(FeedX, StrutY, FeedZ, 0, Current.FeedYawDeg);

    /// <summary>Truss nodes: A / M / D at support level, apexes T1 / T2; all at z = <see cref="PlaneZ"/>.</summary>
    public static (string Name, double X, double Y, double Z)[] FootNodes =>
    [
        ("A", -Panel, FootY0, PlaneZ),
        ("M", 0, FootY0, PlaneZ),
        ("D", Panel, FootY0, PlaneZ),
        ("T1", -Panel / 2, FootY0 + Height, PlaneZ),
        ("T2", Panel / 2, FootY0 + Height, PlaneZ)
    ];

    static IReadOnlyList<MemberDef>? _members;
    /// <summary>The seven members (Ids name their end nodes), in the default build order.</summary>
    public static IReadOnlyList<MemberDef> Members => _members ??= BuildMembers();

    static IReadOnlyList<MemberDef> BuildMembers()
    {
        (double, double, double) At(string node)
        {
            var n = FootNodes.First(f => f.Name == node);
            return (n.X, n.Y, n.Z);
        }
        var edges = new (string Id, string Label, string Glyph, string P, string Q)[]
        {
            ("AM", "bottom chord A–M", "_", "A", "M"),
            ("AT1", "diagonal A–T1", "/", "A", "T1"),
            ("T1M", "diagonal T1–M", "\\", "T1", "M"),
            ("MT2", "diagonal M–T2", "/", "M", "T2"),
            ("T1T2", "top chord T1–T2", "‾", "T1", "T2"),
            ("MD", "bottom chord M–D", "_", "M", "D"),
            ("T2D", "diagonal T2–D", "\\", "T2", "D")
        };
        return edges.Select(e => new MemberDef(e.Id, e.Label, e.Glyph, Store, AlongEdge(At(e.P), At(e.Q)), At(e.P), At(e.Q), e.P, e.Q)).ToList();
    }

    /// <summary>
    /// Cane of <see cref="StrutLength"/> centred on edge P→Q, axis along the edge.
    /// </summary>
    public static BambooIcdCell.StrutPlacement AlongEdge(
        (double X, double Y, double Z) p, (double X, double Y, double Z) q)
    {
        var dx = q.X - p.X; var dy = q.Y - p.Y; var dz = q.Z - p.Z;
        var len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        if (len < StrutLength - 1e-6)
            throw new InvalidOperationException($"edge length {len:F4} m < strut {StrutLength} m");
        dx /= len; dy /= len; dz /= len;
        var (pitch, yaw) = PitchYawForAxis(dx, dy, dz);
        var alt = PitchYawForAxis(-dx, -dy, -dz);
        if (PreferAlt(pitch, yaw, alt.Pitch, alt.Yaw))
            (pitch, yaw) = alt;
        return new BambooIcdCell.StrutPlacement((p.X + q.X) / 2, (p.Y + q.Y) / 2, (p.Z + q.Z) / 2, pitch, yaw);
    }

    static (double Pitch, double Yaw) PitchYawForAxis(double dx, double dy, double dz)
    {
        if (dx * dx + dz * dz < 1e-12)
            return (dy >= 0 ? -90 : 90, 0);
        var pitch = Math.Atan2(-dy, Math.Sqrt(dx * dx + dz * dz)) * 180 / Math.PI;
        var yaw = Math.Atan2(dx, dz) * 180 / Math.PI;
        return (pitch, yaw);
    }

    static bool PreferAlt(double p0, double y0, double p1, double y1)
    {
        if (Math.Abs(Math.Abs(y1) - 90) < 1 && p1 <= 0 && p0 > 0) return true;
        if (Math.Abs(y1) < 1e-9 && Math.Abs(p1 + 90) < 1) return true;
        return false;
    }

    /// <summary>Members no allowed support can reach, with the reason (reported, never faked).</summary>
    public static IReadOnlyDictionary<string, string> UnreachableMembers => Current.Unreachable;

    /// <summary>Nodes a new member may start from: the anchors A, D and every node a placed member reaches.</summary>
    public static HashSet<string> ExistingNodes(IEnumerable<string> placed)
    {
        var set = new HashSet<string>(StringComparer.Ordinal) { "A", "D" };
        var ids = placed.ToHashSet(StringComparer.Ordinal);
        foreach (var m in Members.Where(m => ids.Contains(m.Id))) { set.Add(m.NodeA); set.Add(m.NodeB); }
        return set;
    }

    /// <summary>
    /// Rigid, anchored subframes of the placed members. A rigid cluster is seeded by any closed triangle of placed
    /// members and grows by every node tied to it by two placed members (a triangulated, statically determinate
    /// joint); clusters sharing two nodes merge. A cluster counts once it contains an anchor (A or D) — e.g. a
    /// closed triangle hinged at A. Returns the node sets of the anchored clusters.
    /// </summary>
    public static List<HashSet<string>> AnchoredRigidClusters(IEnumerable<string> placed)
    {
        var ids = placed.ToHashSet(StringComparer.Ordinal);
        var edges = Members.Where(m => ids.Contains(m.Id)).Select(m => (m.NodeA, m.NodeB)).ToList();
        bool Has(string a, string b) => edges.Any(e => (e.NodeA == a && e.NodeB == b) || (e.NodeA == b && e.NodeB == a));
        var nodes = edges.SelectMany(e => new[] { e.NodeA, e.NodeB }).Distinct().ToList();
        var clusters = new List<HashSet<string>>();
        for (var i = 0; i < nodes.Count; i++)
        for (var j = i + 1; j < nodes.Count; j++)
        for (var k = j + 1; k < nodes.Count; k++)
            if (Has(nodes[i], nodes[j]) && Has(nodes[j], nodes[k]) && Has(nodes[i], nodes[k]))
                clusters.Add(new HashSet<string>(StringComparer.Ordinal) { nodes[i], nodes[j], nodes[k] });
        for (var changed = true; changed;)
        {
            changed = false;
            foreach (var c in clusters)
                foreach (var n in nodes.Where(n => !c.Contains(n)))
                    if (c.Count(x => Has(x, n)) >= 2) { c.Add(n); changed = true; }
            for (var a = 0; a < clusters.Count && !changed; a++)
            for (var b = a + 1; b < clusters.Count && !changed; b++)
                if (clusters[a].Count(clusters[b].Contains) >= 2) { clusters[a].UnionWith(clusters[b]); clusters.RemoveAt(b); changed = true; }
        }
        return clusters.Where(c => c.Contains("A") || c.Contains("D")).ToList();
    }

    /// <summary>Fixed nodes: the anchors plus every node of an anchored rigid cluster.</summary>
    public static HashSet<string> FixedNodes(IEnumerable<string> placed)
    {
        var set = new HashSet<string>(StringComparer.Ordinal) { "A", "D" };
        foreach (var c in AnchoredRigidClusters(placed)) set.UnionWith(c);
        return set;
    }

    /// <summary>
    /// Members fixed into the structure — the only canes the robot may clamp: both end nodes lie in one anchored
    /// rigid cluster (<see cref="AnchoredRigidClusters"/>).
    /// </summary>
    public static IReadOnlyList<string> FixedBothEnds(IEnumerable<string> placed)
    {
        var list = placed.ToList();
        var clusters = AnchoredRigidClusters(list);
        return Members.Where(m => list.Contains(m.Id))
            .Where(m => clusters.Any(c => c.Contains(m.NodeA) && c.Contains(m.NodeB)))
            .Select(m => m.Id).ToList();
    }

    /// <summary>Members fixed into the structure after the first <paramref name="placedCount"/> members are placed.</summary>
    public static IReadOnlyList<string> WalkableMemberIdsAfter(int placedCount) =>
        FixedBothEnds(Members.Take(placedCount).Select(m => m.Id));

    public static object FootprintDescription() => new
    {
        nodes = FootNodes.Select(n => new { name = n.Name, x = n.X, y = n.Y, z = n.Z }),
        edges = Members.Select(m => new
        {
            id = m.Id,
            glyph = m.Glyph,
            ax = m.EdgeA.X, ay = m.EdgeA.Y, az = m.EdgeA.Z,
            bx = m.EdgeB.X, by = m.EdgeB.Y, bz = m.EdgeB.Z
        }),
        anchors = Pads.Select(a => new { name = a.Name, x = a.X, z = a.Z, halfX = a.HalfX, halfZ = a.HalfZ }),
        pit = PitBoxes.Select(b => new { name = b.Name, x = b.X, y = b.Y, z = b.Z, halfX = b.HalfX, halfY = b.HalfY, halfZ = b.HalfZ }),
        posts = Posts.Select(p => new { name = p.Name, x = p.X, z = p.Z, top = p.Top, radius = p.Radius }),
        racks = Racks.Select(r => new { name = r.Name, x = r.X, z = r.Z, halfX = r.HalfX, halfZ = r.HalfZ, top = r.Top }),
        motusLimit = $"{StrutLength:F3} m canes, panel {Panel:F3} m, node gap {NodeGap:F3} m; pit between A and D; L-grip on the anchor columns or fixed members; unreachable: " +
                     (UnreachableMembers.Count == 0 ? "none" : string.Join(", ", UnreachableMembers.Keys))
    };

    public static object StoreDescription() => new
    {
        slots = new[] { new { id = "feed", x = Store.X, y = Store.Y, z = Store.Z, pitch = Store.PitchDeg, yaw = Store.YawDeg } }
    };
}
