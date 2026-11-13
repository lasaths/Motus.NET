using Motus.Construction;
using static Motus.Construction.BambooTrussLayout;

/// <summary>
/// 1 × 2 bay rectangular tower, 3 stories: 2 × 3 nodes per level, 4 node levels (ground + 3). Viewer frame: Y up, metres.
/// Ground nodes are anchors. Per story: 6 posts, 7 diagonals (one per wall panel and one in the middle cross frame, alternating), and at each upper
/// level 7 beams (4 along Z, 3 along X). Cane = node distance − 2 gaps. Geometry from env: BAY (node spacing, both plan
/// directions), STORY (story height), FEEDTABLE (JSON {member: [x, z, yaw]}).
/// </summary>
public static class TowerInputs
{
    static double Env(string k, double d) => double.TryParse(Environment.GetEnvironmentVariable(k), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var v) ? v : d;
    public static readonly double Bay = Env("BAY", 0.30);
    public static readonly double Story = Env("STORY", 0.30);
    public const double Gap = 0.03, SupportY = 0.19, ClampBelow = 0.10;
    public static readonly double FeedTop = Env("FEEDTOP", 0.12);
    public const int Levels = 3, NX = 2, NZ = 3;

    public static string Name(int k, int i, int j) => $"N{k}{i}{j}";
    public static (double X, double Y, double Z) At(int k, int i, int j) => ((i - 0.5) * Bay, SupportY + k * Story, (j - 1) * Bay);

    public static Site Build()
    {
        var nodes = new List<StructureNode>();
        for (var k = 0; k <= Levels; k++) for (var i = 0; i < NX; i++) for (var j = 0; j < NZ; j++) { var p = At(k, i, j); nodes.Add(new(Name(k, i, j), p.X, p.Y, p.Z)); }
        var members = new List<StructureMember>();
        void Add((int k, int i, int j) a, (int k, int i, int j) b, string glyph)
        {
            var (p, q) = (At(a.k, a.i, a.j), At(b.k, b.i, b.j));
            var d = Math.Sqrt((q.X - p.X) * (q.X - p.X) + (q.Y - p.Y) * (q.Y - p.Y) + (q.Z - p.Z) * (q.Z - p.Z));
            var (na, nb) = (Name(a.k, a.i, a.j), Name(b.k, b.i, b.j));
            members.Add(new(na + nb, $"{na}–{nb}", glyph, na, nb, Math.Round(d - 2 * Gap, 4)));
        }
        for (var k = 0; k < Levels; k++)
        {
            for (var i = 0; i < NX; i++) for (var j = 0; j < NZ; j++) Add((k, i, j), (k + 1, i, j), "|");          // posts
            // v2 bracing ("corner"): every long-face diagonal runs from the middle node of level k up to a corner of level
            // k+1; the short faces brace N(k,1,0)→N(k+1,0,0) and N(k,0,2)→N(k+1,1,2); the middle cross frame N(k,0,1)→N(k+1,1,1).
            // So each corner of level k+1 on the x=0 side gets post + long-face + short-face diagonal: fixed after 3 canes
            // (three non-coplanar bars to fixed nodes), a foothold early in the story.
            for (var i = 0; i < NX; i++) { Add((k, i, 1), (k + 1, i, 0), "\\"); Add((k, i, 1), (k + 1, i, 2), "/"); }
            Add((k, 1, 0), (k + 1, 0, 0), "/"); Add((k, 0, 2), (k + 1, 1, 2), "/"); Add((k, 0, 1), (k + 1, 1, 1), "/");
            var u = k + 1; // beams of the level above
            for (var i = 0; i < NX; i++) for (var j = 0; j < NZ - 1; j++) Add((u, i, j), (u, i, j + 1), "-");
            for (var j = 0; j < NZ; j++) Add((u, 0, j), (u, 1, j), "-");
        }
        if (Environment.GetEnvironmentVariable("FEEDTABLE") is { Length: > 0 } ft)
        {
            var t = System.Text.Json.JsonSerializer.Deserialize<Dictionary<string, double[]>>(File.ReadAllText(ft))!;
            members = members.Select(m => t.TryGetValue(m.Id, out var v) ? m with { Feed = new FeedSpot(v[0], v[1], v[2]) } : m).ToList();
        }
        // Anchors: the 6 ground nodes, heading toward the plan centre; start on N000 (heading 30° off inward, as the dome).
        var anchors = new List<Stake>();
        for (var i = 0; i < NX; i++) for (var j = 0; j < NZ; j++)
        {
            var p = At(0, i, j);
            var heading = Math.Atan2(-p.X, -p.Z) * 180 / Math.PI;
            if (Math.Abs(p.Z) < 1e-9) heading = p.X < 0 ? 90 : -90; // middle anchors: inward is ±X
            if (i == 0 && j == 0) heading = Env("STARTHEAD", heading - 30); // v2: start heading as data (default: 30° off inward, as the dome)
            anchors.Add(new Stake("anchor-" + Name(0, i, j), p.X, p.Z, p.Y - ClampBelow, p.Y - HubHalf, heading));
        }
        // Site stock as the dome: relative to the start anchor's inward direction θ: 0.21 m to the −side, 0.06 m outboard.
        var s = At(0, 0, 0);
        var th = Math.Atan2(-s.X, -s.Z);
        double[]? stock = Environment.GetEnvironmentVariable("STOCK") is { Length: > 0 } sv ? sv.Split(',').Select(v => double.Parse(v, System.Globalization.CultureInfo.InvariantCulture)).ToArray() : null;
        (double X, double Z) xh = (Math.Cos(th), -Math.Sin(th)), zh = (Math.Sin(th), Math.Cos(th));
        var feedX = s.X - 0.21 * xh.X - 0.06 * zh.X; var feedZ = s.Z - 0.21 * xh.Z - 0.06 * zh.Z;
        var feedYaw = 90 + th * 180 / Math.PI;
        return new Site(
            Length: Env("SITELEN", members.Max(m => m.Length!.Value)), /* v2: SITELEN = the post/beam cane, so the one stock's pegs fit it */ Gap: Gap, NodeY: SupportY, PitDepth: 0,
            Stakes: anchors, StartStake: "anchor-" + Name(0, 0, 0),
            FeedX: stock?[0] ?? Math.Round(feedX, 4), FeedZ: stock?[1] ?? Math.Round(feedZ, 4), FeedYawDeg: stock?[2] ?? Math.Round(feedYaw, 2), FeedTop: FeedTop,
            WalkRAnchor: "placed-" + members[0].Id, WalkLTarget: "anchor-" + Name(0, 0, 0),
            Unreachable: new Dictionary<string, string>(),
            Structure: new StructureDefinition(nodes, members));
    }
}
