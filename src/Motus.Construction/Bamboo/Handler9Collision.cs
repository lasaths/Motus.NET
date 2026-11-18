using System.Xml.Linq;
using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Construction;

/// <summary>
/// EXPERIMENTAL opt-in self-collision for the bamboo handler (docs/handler9.md).
/// Exact mesh tests of the CAD STLs, with the same clearance as <see cref="MotionConstraints.ClearanceMarginM"/>
/// (the other body is grown by it, default 2 mm). Allowed contacts are adjacent collision links from the URDF tree,
/// plus each clamp against a cane when one is given. The planner defaults and the URDF collision boxes do not change.
/// The check uses <see cref="TreeFkCollisionChecker"/> with self-collision and clearance turned on.
/// </summary>
public sealed class Handler9Collision
{
    public const string CaneName = "cane";
    public const double CaneDiameterM = 0.06;

    private readonly KinematicTree _tree;
    private readonly TreeFkCollisionChecker _checker;
    private readonly IReadOnlyList<(string A, string B)> _scenePairs;
    private readonly CollisionScene _selfScene;
    private readonly double _clearance;

    public Handler9Collision(KinematicTree tree, string urdfPath, double? clearanceMarginM = null)
    {
        _tree = tree;
        _clearance = clearanceMarginM ?? new MotionConstraints().ClearanceMarginM;
        if (!double.IsFinite(_clearance) || _clearance < 0)
            throw new ArgumentException("handler9: clearance must be a finite value >= 0.");

        var meshes = LoadCadMeshes(urdfPath, tree);
        if (meshes.Count == 0)
            throw new ArgumentException("handler9: no CAD meshes in the URDF.");

        AllowedAdjacentPairs = AdjacentPairs(tree, meshes.Select(m => m.LinkName).ToHashSet(StringComparer.Ordinal));
        AllowedCaneContactPairs = AllowedCanePairs();
        _scenePairs = ScenePairs(tree, AllowedAdjacentPairs, AllowedCaneContactPairs);
        _selfScene = new CollisionScene(allowedPairs: _scenePairs);

        var names = tree.DriverJointIndices.Select(i => tree.Joints[i].Name).ToArray();
        var limits = tree.DriverJointIndices.Select(i =>
        {
            var j = tree.Joints[i];
            return new JointLimit(j.Lower, j.Upper);
        }).ToArray();
        var robot = new RobotModel(
            new RobotPreset
            {
                Manufacturer = RobotManufacturer.Unknown,
                ModelName = "handler9",
                Family = "handler9",
                AxisCount = tree.DriverCount,
                JointLimits = limits,
            },
            new RobotCollisionModel(meshes),
            names);
        _checker = new TreeFkCollisionChecker(
            robot,
            tree,
            tipChain: null,
            planJointNames: names,
            checkSelfCollision: true,
            clearanceMarginM: _clearance);
    }

    /// <summary>Adjacent collision-bearing links from the URDF tree. Empty links (turntables) are walked through.</summary>
    public IReadOnlyList<(string A, string B)> AllowedAdjacentPairs { get; }

    /// <summary>Each clamp frame against the cane. The cane is not a robot part.</summary>
    public IReadOnlyList<(string A, string B)> AllowedCaneContactPairs { get; }

    public double ClearanceMarginM => _clearance;

    /// <summary>Each clamp frame against the cane name.</summary>
    public static IReadOnlyList<(string A, string B)> AllowedCanePairs(string caneName = CaneName) =>
        Handler9Kinematics.ClampLinks.Select(c => (c, caneName)).ToArray();

    /// <summary>
    /// Straight cane (or rod) of CAD diameter, a capsule along <paramref name="axis"/> centred on <paramref name="point"/>.
    /// </summary>
    public static CollisionObject Cane(double[] point, double[] axis, double lengthM, double diameterM = CaneDiameterM, string name = CaneName)
    {
        if (point.Length != 3 || axis.Length != 3 || !point.Concat(axis).All(double.IsFinite))
            throw new ArgumentException("handler9: cane point[3] and axis[3] must be finite.");
        var n = Math.Sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
        if (n < 1e-12 || !double.IsFinite(lengthM) || lengthM <= 0 || !double.IsFinite(diameterM) || diameterM <= 0)
            throw new ArgumentException("handler9: cane axis must be non-zero and length and diameter must be > 0.");
        double zx = axis[0] / n, zy = axis[1] / n, zz = axis[2] / n;
        // x = tmp × z, tmp is +Z unless z is almost +Z
        double tx = Math.Abs(zz) < 0.9 ? 0 : 1, ty = 0, tz = Math.Abs(zz) < 0.9 ? 1 : 0;
        double xx = ty * zz - tz * zy, xy = tz * zx - tx * zz, xz = tx * zy - ty * zx;
        var xn = Math.Sqrt(xx * xx + xy * xy + xz * xz);
        xx /= xn; xy /= xn; xz /= xn;
        double yx = zy * xz - zz * xy, yy = zz * xx - zx * xz, yz = zx * xy - zy * xx;
        var m = new double[16];
        m[0] = xx; m[1] = yx; m[2] = zx; m[3] = point[0];
        m[4] = xy; m[5] = yy; m[6] = zy; m[7] = point[1];
        m[8] = xz; m[9] = yz; m[10] = zz; m[11] = point[2];
        m[15] = 1;
        return CollisionObject.Capsule(name, Transforms.ToFrame(m), diameterM / 2, lengthM / 2);
    }

    public static IReadOnlyList<LinkCollisionGeometry> LoadCadMeshes(string urdfPath, KinematicTree tree)
    {
        var path = Path.GetFullPath(urdfPath);
        var dir = Path.GetDirectoryName(path) ?? ".";
        var doc = XDocument.Load(path);
        var list = new List<LinkCollisionGeometry>();
        foreach (var linkEl in doc.Root!.Elements("link"))
        {
            var name = linkEl.Attribute("name")?.Value;
            if (string.IsNullOrWhiteSpace(name)) continue;
            int index;
            try { index = tree.IndexOfLink(name); }
            catch (ArgumentException) { continue; }
            var meshEl = linkEl.Element("visual")?.Element("geometry")?.Element("mesh");
            var filename = meshEl?.Attribute("filename")?.Value;
            if (string.IsNullOrWhiteSpace(filename)) continue;
            var scale = ParseScale(meshEl!.Attribute("scale")?.Value);
            var cleaned = filename.Replace("package://", "", StringComparison.Ordinal).Replace("file://", "", StringComparison.Ordinal);
            var file = Path.GetFullPath(Path.Combine(dir, cleaned));
            if (!File.Exists(file))
                file = Path.GetFullPath(Path.Combine(dir, Path.GetFileName(cleaned)));
            if (!File.Exists(file))
                throw new FileNotFoundException($"handler9: CAD mesh not found: {filename}");
            var (vertices, indices) = StlReader.Read(file, scale);
            list.Add(new LinkCollisionGeometry(index, name, CollisionObject.Mesh(name, Frame.Identity, vertices, indices)));
        }
        return list;
    }

    /// <summary>
    /// True when no two non-allowed bodies are closer than the clearance. <paramref name="cane"/> is optional;
    /// when given, every clamp may touch it and every other robot part is checked against it.
    /// </summary>
    public bool IsSelfCollisionFree(IReadOnlyList<double> q, out string? hit, CollisionObject? cane = null)
    {
        hit = null;
        if (q.Count != _tree.DriverCount || !q.All(double.IsFinite))
        {
            hit = "Expected a finite full driver vector.";
            return false;
        }
        var scene = cane is null ? _selfScene : new CollisionScene(new[] { cane }, _scenePairs);
        var ok = _checker.IsCollisionFree(new JointState(q), scene);
        hit = ok ? null : _checker.LastHit;
        return ok;
    }

    internal static IReadOnlyList<(string A, string B)> AdjacentPairs(KinematicTree tree, IReadOnlySet<string> meshLinks)
    {
        var parentOf = new int[tree.Links.Count];
        Array.Fill(parentOf, -1);
        foreach (var j in tree.Joints)
            parentOf[j.ChildLinkIndex] = j.ParentLinkIndex;

        string? MeshAncestor(int link)
        {
            for (var i = link; i >= 0; i = parentOf[i])
            {
                var name = tree.Links[i].Name;
                if (meshLinks.Contains(name))
                    return name;
                if (parentOf[i] == i)
                    break;
            }
            return null;
        }

        var pairs = new List<(string A, string B)>();
        foreach (var j in tree.Joints)
        {
            var child = tree.Links[j.ChildLinkIndex].Name;
            if (!meshLinks.Contains(child))
                continue;
            var parent = MeshAncestor(j.ParentLinkIndex);
            if (parent is null || parent == child)
                continue;
            if (pairs.Exists(p => p.A == parent && p.B == child || p.A == child && p.B == parent))
                continue;
            pairs.Add((parent, child));
        }

        // A fixed mesh child may contact its siblings on the same mesh parent
        // (outer-station clamp + feed jaw). Two moving siblings (the MainClamp sleds) are not allowed.
        var kids = new Dictionary<string, List<(string Name, bool Fixed)>>(StringComparer.Ordinal);
        foreach (var j in tree.Joints)
        {
            var child = tree.Links[j.ChildLinkIndex].Name;
            if (!meshLinks.Contains(child))
                continue;
            var parent = MeshAncestor(j.ParentLinkIndex);
            if (parent is null || parent == child)
                continue;
            if (!kids.TryGetValue(parent, out var list))
                kids[parent] = list = new List<(string, bool)>();
            list.Add((child, j.Type == KinematicJointType.Fixed));
        }
        foreach (var list in kids.Values)
        {
            for (var i = 0; i < list.Count; i++)
            {
                for (var j = i + 1; j < list.Count; j++)
                {
                    if (!list[i].Fixed && !list[j].Fixed)
                        continue;
                    pairs.Add((list[i].Name, list[j].Name));
                }
            }
        }
        return pairs;
    }

    static IReadOnlyList<(string A, string B)> ScenePairs(
        KinematicTree tree,
        IReadOnlyList<(string A, string B)> adjacent,
        IReadOnlyList<(string A, string B)> cane)
    {
        var list = new List<(string A, string B)>(adjacent.Count * 2 + cane.Count * 2);
        foreach (var (a, b) in adjacent)
        {
            list.Add((a, b));
            list.Add((CollisionBodies.RobotLink(tree.IndexOfLink(a)), CollisionBodies.RobotLink(tree.IndexOfLink(b))));
        }
        foreach (var (a, b) in cane)
        {
            list.Add((a, b));
            list.Add((CollisionBodies.RobotLink(tree.IndexOfLink(a)), b));
        }
        return list;
    }

    private static double ParseScale(string? s)
    {
        if (string.IsNullOrWhiteSpace(s))
            return 1;
        var p = s.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        return double.Parse(p[0], System.Globalization.CultureInfo.InvariantCulture);
    }
}
