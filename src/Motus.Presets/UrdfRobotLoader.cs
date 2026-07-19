using System.Globalization;
using System.Xml.Linq;
using Motus.Core;
using Motus.Geometry;

namespace Motus.Presets;

public sealed class UrdfLoadOptions
{
    public string BaseLink { get; set; } = "base_link";
    public string TipLink { get; set; } = "tool0";
    public string ModelName { get; set; } = "urdf_robot";
}

public sealed class UrdfRobot
{
    public RobotPreset Preset { get; }
    public SerialJointChain Chain { get; }
    public IReadOnlyList<string> JointNames { get; }
    public RobotCollisionModel? CollisionModel { get; }
    public KinematicTree? Tree { get; }

    public UrdfRobot(
        RobotPreset preset,
        SerialJointChain chain,
        IReadOnlyList<string> jointNames,
        RobotCollisionModel? collisionModel = null,
        KinematicTree? tree = null)
    {
        Preset = preset;
        Chain = chain;
        JointNames = jointNames;
        CollisionModel = collisionModel;
        Tree = tree;
    }

    public RobotModel ToModel() => new(Preset, CollisionModel, JointNames);
}

/// <summary>Load URDF into Motus preset + serial tip chain, or full <see cref="KinematicTree"/>.</summary>
public static class UrdfRobotLoader
{
    public static UrdfRobot Load(string urdfPath, UrdfLoadOptions? options = null)
    {
        var doc = XDocument.Load(urdfPath);
        return Load(doc, options ?? new UrdfLoadOptions(), Path.GetDirectoryName(Path.GetFullPath(urdfPath)) ?? ".");
    }

    /// <summary>Expand a .xacro file then load as URDF.</summary>
    public static UrdfRobot LoadXacro(string xacroPath, UrdfLoadOptions? options = null, XacroOptions? xacroOptions = null)
    {
        var doc = XacroPreprocessor.ExpandDocument(xacroPath, xacroOptions);
        return Load(doc, options ?? new UrdfLoadOptions(), Path.GetDirectoryName(Path.GetFullPath(xacroPath)) ?? ".");
    }

    public static UrdfRobot Load(XDocument doc, UrdfLoadOptions options, string? urdfDirectory = null)
    {
        urdfDirectory ??= ".";
        var tree = LoadTree(doc, options, urdfDirectory);
        var tip = tree.ExtractSerialTip(options.BaseLink, options.TipLink);

        var limits = new List<JointLimit>(tip.JointNames.Count);
        for (var i = 0; i < tip.JointNames.Count; i++)
        {
            var name = tip.JointNames[i];
            KinematicJoint? kj = null;
            for (var j = 0; j < tree.Joints.Count; j++)
            {
                if (string.Equals(tree.Joints[j].Name, name, StringComparison.OrdinalIgnoreCase))
                {
                    kj = tree.Joints[j];
                    break;
                }
            }
            var lo = kj?.Lower ?? -Math.PI;
            var hi = kj?.Upper ?? Math.PI;
            var vel = kj?.Velocity ?? Math.PI;
            limits.Add(new JointLimit(lo, hi, vel, vel * 2));
        }

        var robotName = tree.Name;
        var modelName = options.ModelName;
        if (string.Equals(modelName, "urdf_robot", StringComparison.OrdinalIgnoreCase)
            && !string.IsNullOrWhiteSpace(robotName))
            modelName = robotName;

        var preset = new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = modelName,
            Family = "urdf",
            AxisCount = tip.Chain.Joints.Length,
            JointLimits = limits,
            BaseFrame = BaseFrame.Identity,
            ToolFrame = tip.TipToolOffset is { } tipFrame ? new ToolFrame(tipFrame, options.TipLink) : ToolFrame.Identity,
            SourceNote = "Imported from URDF",
        };

        var linkNames = new List<string>(tip.JointNames.Count);
        for (var i = 0; i < tip.JointNames.Count; i++)
        {
            var name = tip.JointNames[i];
            for (var j = 0; j < tree.Joints.Count; j++)
            {
                if (string.Equals(tree.Joints[j].Name, name, StringComparison.OrdinalIgnoreCase))
                {
                    linkNames.Add(tree.Links[tree.Joints[j].ChildLinkIndex].Name);
                    break;
                }
            }
        }

        var robotEl = doc.Root ?? throw new InvalidOperationException("URDF has no root element.");
        var linkCollision = UrdfCollisionLoader.Load(robotEl, linkNames, urdfDirectory);
        var toolGeom = UrdfCollisionLoader.LoadTipLinkGeometry(robotEl, options.TipLink, urdfDirectory);
        var collision = UrdfCollisionLoader.WithToolGeometry(linkCollision, toolGeom);

        return new UrdfRobot(preset, tip.Chain, tip.JointNames, collision, tree);
    }

    public static KinematicTree LoadTree(string urdfPath, UrdfLoadOptions? options = null)
    {
        var doc = XDocument.Load(urdfPath);
        return LoadTree(doc, options ?? new UrdfLoadOptions(), Path.GetDirectoryName(Path.GetFullPath(urdfPath)) ?? ".");
    }

    public static KinematicTree LoadTree(XDocument doc, UrdfLoadOptions? options = null, string? urdfDirectory = null)
    {
        _ = options;
        urdfDirectory ??= ".";
        var robot = doc.Root ?? throw new InvalidOperationException("URDF has no root element.");
        var robotName = robot.Attribute("name")?.Value ?? "urdf_robot";

        var linkEls = robot.Elements("link").ToList();
        var parsedJoints = robot.Elements("joint").Select(ParseJoint).ToList();

        var linkNames = new List<string>(linkEls.Count);
        var meshNames = new List<string?>(linkEls.Count);
        var meshPaths = new List<string?>(linkEls.Count);
        var linkIndex = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);

        foreach (var el in linkEls)
        {
            var name = el.Attribute("name")?.Value ?? "";
            if (string.IsNullOrWhiteSpace(name)) continue;
            if (linkIndex.ContainsKey(name)) continue;
            linkIndex[name] = linkNames.Count;
            linkNames.Add(name);
            var (meshName, meshPath) = TryMeshRef(el);
            meshNames.Add(meshName);
            meshPaths.Add(meshPath);
        }

        // Ensure joint parent/child links exist even if omitted as <link/> stubs.
        foreach (var j in parsedJoints)
        {
            EnsureLink(j.ParentLink);
            EnsureLink(j.ChildLink);
        }

        void EnsureLink(string name)
        {
            if (string.IsNullOrWhiteSpace(name) || linkIndex.ContainsKey(name)) return;
            linkIndex[name] = linkNames.Count;
            linkNames.Add(name);
            meshNames.Add(null);
            meshPaths.Add(null);
        }

        var childSet = new HashSet<int>();
        foreach (var j in parsedJoints)
        {
            if (linkIndex.TryGetValue(j.ChildLink, out var ci))
                childSet.Add(ci);
        }

        var root = -1;
        for (var i = 0; i < linkNames.Count; i++)
        {
            if (!childSet.Contains(i))
            {
                if (root >= 0)
                    throw new InvalidOperationException("URDF has multiple root links; expected a single tree.");
                root = i;
            }
        }
        if (root < 0)
            throw new InvalidOperationException("URDF has no root link.");

        var nameToJointIndex = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
        for (var i = 0; i < parsedJoints.Count; i++)
            nameToJointIndex[parsedJoints[i].Name] = i;

        var driverIndices = new List<int>();
        var joints = new KinematicJoint[parsedJoints.Count];
        for (var i = 0; i < parsedJoints.Count; i++)
        {
            var p = parsedJoints[i];
            if (!linkIndex.TryGetValue(p.ParentLink, out var pi) || !linkIndex.TryGetValue(p.ChildLink, out var ci))
                throw new InvalidOperationException($"Joint '{p.Name}' references unknown link.");

            var type = ParseJointType(p.Type);
            KinematicMimic? mimic = null;
            var driverIndex = -1;
            if (type != KinematicJointType.Fixed)
            {
                if (!string.IsNullOrWhiteSpace(p.MimicJoint))
                {
                    if (!nameToJointIndex.TryGetValue(p.MimicJoint, out var mi))
                        throw new InvalidOperationException($"Mimic joint '{p.Name}' references unknown joint '{p.MimicJoint}'.");
                    mimic = new KinematicMimic(mi, p.MimicMultiplier, p.MimicOffset);
                }
                else
                {
                    driverIndex = driverIndices.Count;
                    driverIndices.Add(i);
                }
            }

            joints[i] = new KinematicJoint(
                p.Name, type, pi, ci,
                p.OriginX, p.OriginY, p.OriginZ,
                p.Roll, p.Pitch, p.Yaw,
                p.AxisX, p.AxisY, p.AxisZ,
                p.Lower, p.Upper, p.Velocity,
                driverIndex, mimic);
        }

        var links = new KinematicLink[linkNames.Count];
        for (var i = 0; i < links.Length; i++)
            links[i] = new KinematicLink(linkNames[i], meshNames[i], meshPaths[i]);

        _ = urdfDirectory; // reserved for future mesh path resolution
        return new KinematicTree(robotName, links, joints, root, driverIndices);
    }

    private static (string? name, string? path) TryMeshRef(XElement linkEl)
    {
        foreach (var section in linkEl.Elements().Where(e => e.Name.LocalName is "visual" or "collision"))
        {
            var mesh = section.Element("geometry")?.Element("mesh");
            var filename = mesh?.Attribute("filename")?.Value;
            if (string.IsNullOrWhiteSpace(filename)) continue;
            var name = Path.GetFileName(filename.Replace('\\', '/'));
            return (name, filename);
        }
        return (null, null);
    }

    private static KinematicJointType ParseJointType(string type) => type.ToLowerInvariant() switch
    {
        "revolute" => KinematicJointType.Revolute,
        "continuous" => KinematicJointType.Continuous,
        "prismatic" => KinematicJointType.Prismatic,
        "fixed" => KinematicJointType.Fixed,
        _ => throw new InvalidOperationException($"Unsupported joint type '{type}'.")
    };

    private static ParsedJoint ParseJoint(XElement el)
    {
        var origin = el.Element("origin");
        var axis = el.Element("axis");
        var limit = el.Element("limit");
        var mimic = el.Element("mimic");
        var xyz = ParseTriple(origin?.Attribute("xyz")?.Value, 0, 0, 0);
        var rpy = ParseTriple(origin?.Attribute("rpy")?.Value, 0, 0, 0);
        var ax = ParseTriple(axis?.Attribute("xyz")?.Value, 0, 0, 1);
        var len = Math.Sqrt(ax.x * ax.x + ax.y * ax.y + ax.z * ax.z);
        if (len > 1e-12) { ax = (ax.x / len, ax.y / len, ax.z / len); }

        return new ParsedJoint(
            el.Attribute("name")?.Value ?? "",
            el.Attribute("type")?.Value ?? "fixed",
            el.Element("parent")?.Attribute("link")?.Value ?? "",
            el.Element("child")?.Attribute("link")?.Value ?? "",
            xyz.x, xyz.y, xyz.z, rpy.x, rpy.y, rpy.z,
            ax.x, ax.y, ax.z,
            ParseDouble(limit?.Attribute("lower")?.Value, -Math.PI),
            ParseDouble(limit?.Attribute("upper")?.Value, Math.PI),
            limit?.Attribute("velocity") is { } v ? ParseDouble(v.Value, Math.PI) : null,
            mimic?.Attribute("joint")?.Value,
            ParseDouble(mimic?.Attribute("multiplier")?.Value, 1),
            ParseDouble(mimic?.Attribute("offset")?.Value, 0));
    }

    private static (double x, double y, double z) ParseTriple(string? s, double dx, double dy, double dz)
    {
        if (string.IsNullOrWhiteSpace(s)) return (dx, dy, dz);
        var p = s.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        if (p.Length < 3) return (dx, dy, dz);
        return (ParseDouble(p[0], dx), ParseDouble(p[1], dy), ParseDouble(p[2], dz));
    }

    private static double ParseDouble(string? s, double fallback) =>
        double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) ? v : fallback;

    private sealed record ParsedJoint(
        string Name, string Type, string ParentLink, string ChildLink,
        double OriginX, double OriginY, double OriginZ,
        double Roll, double Pitch, double Yaw,
        double AxisX, double AxisY, double AxisZ,
        double Lower, double Upper, double? Velocity,
        string? MimicJoint, double MimicMultiplier, double MimicOffset);
}
