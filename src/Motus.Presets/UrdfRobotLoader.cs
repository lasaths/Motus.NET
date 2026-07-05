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

    public UrdfRobot(RobotPreset preset, SerialJointChain chain, IReadOnlyList<string> jointNames, RobotCollisionModel? collisionModel = null)
    {
        Preset = preset;
        Chain = chain;
        JointNames = jointNames;
        CollisionModel = collisionModel;
    }

    public RobotModel ToModel() => new(Preset, CollisionModel, JointNames);
}

/// <summary>Load revolute serial-chain URDF into Motus preset + joint chain.</summary>
public static class UrdfRobotLoader
{
    public static UrdfRobot Load(string urdfPath, UrdfLoadOptions? options = null)
    {
        var doc = XDocument.Load(urdfPath);
        return Load(doc, options ?? new UrdfLoadOptions(), Path.GetDirectoryName(Path.GetFullPath(urdfPath)) ?? ".");
    }

    public static UrdfRobot Load(XDocument doc, UrdfLoadOptions options, string? urdfDirectory = null)
    {
        urdfDirectory ??= ".";
        var robot = doc.Root ?? throw new InvalidOperationException("URDF has no root element.");
        var joints = robot.Elements("joint").Select(ParseJoint).ToList();
        var (chainJoints, tipOffset) = BuildSerialChain(joints, options.BaseLink, options.TipLink);

        var limits = chainJoints.Select(j => new JointLimit(
            j.Lower, j.Upper,
            j.Velocity ?? Math.PI,
            j.Velocity ?? Math.PI * 2)).ToList();

        var preset = new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = options.ModelName,
            Family = "urdf",
            AxisCount = chainJoints.Count,
            JointLimits = limits,
            BaseFrame = BaseFrame.Identity,
            ToolFrame = tipOffset is { } tip ? new ToolFrame(tip, options.TipLink) : ToolFrame.Identity,
            SourceNote = "Imported from URDF",
        };

        var defs = chainJoints.Select(j => new JointDefinition(
            j.OriginX, j.OriginY, j.OriginZ,
            j.Roll, j.Pitch, j.Yaw,
            j.AxisX, j.AxisY, j.AxisZ,
            Motion: j.Type == "prismatic" ? JointMotionType.Prismatic : JointMotionType.Revolute)).ToArray();

        var jointNames = chainJoints.Select(j => j.Name).ToList();
        var linkNames = chainJoints.Select(j => j.ChildLink).ToList();
        var linkCollision = UrdfCollisionLoader.Load(robot, linkNames, urdfDirectory);
        var toolGeom = UrdfCollisionLoader.LoadTipLinkGeometry(robot, options.TipLink, urdfDirectory);
        var collision = UrdfCollisionLoader.WithToolGeometry(linkCollision, toolGeom);

        return new UrdfRobot(preset, new SerialJointChain(defs), jointNames, collision);
    }

    private static (List<ParsedJoint> Joints, Frame? TipOffset) BuildSerialChain(List<ParsedJoint> all, string baseLink, string tipLink)
    {
        var byChild = all.ToDictionary(j => j.ChildLink, StringComparer.OrdinalIgnoreCase);
        var path = new List<ParsedJoint>();
        var link = tipLink;
        var guard = 0;

        while (!string.Equals(link, baseLink, StringComparison.OrdinalIgnoreCase))
        {
            if (++guard > 64) throw new InvalidOperationException("URDF chain walk exceeded depth limit.");
            if (!byChild.TryGetValue(link, out var joint))
                throw new InvalidOperationException($"No joint with child link '{link}'.");

            path.Add(joint);
            link = joint.ParentLink;
        }

        path.Reverse();

        var merged = new List<ParsedJoint>();
        ParsedJoint? pendingFixed = null;

        foreach (var j in path)
        {
            if (j.Type == "fixed")
            {
                pendingFixed = pendingFixed is null ? j : MergeFixed(pendingFixed, j);
                continue;
            }

            if (j.Type != "revolute" && j.Type != "continuous" && j.Type != "prismatic")
                throw new InvalidOperationException($"Unsupported joint type '{j.Type}' on '{j.Name}'.");

            if (pendingFixed is not null)
            {
                merged.Add(MergeFixedIntoRevolute(pendingFixed, j));
                pendingFixed = null;
            }
            else
            {
                merged.Add(j);
            }
        }

        if (merged.Count == 0)
            throw new InvalidOperationException($"No actuated joints between '{baseLink}' and '{tipLink}'.");

        Frame? tipOffset = null;
        if (pendingFixed is not null)
        {
            var t = Transforms.FromRpy(
                pendingFixed.OriginX, pendingFixed.OriginY, pendingFixed.OriginZ,
                pendingFixed.Roll, pendingFixed.Pitch, pendingFixed.Yaw);
            tipOffset = Transforms.ToFrame(t);
        }

        return (merged, tipOffset);
    }

    private static ParsedJoint MergeFixed(ParsedJoint a, ParsedJoint b)
    {
        var ta = Transforms.FromRpy(a.OriginX, a.OriginY, a.OriginZ, a.Roll, a.Pitch, a.Yaw);
        var tb = Transforms.FromRpy(b.OriginX, b.OriginY, b.OriginZ, b.Roll, b.Pitch, b.Yaw);
        var t = Transforms.Multiply(ta, tb);
        var (x, y, z, roll, pitch, yaw) = MatrixToXyzRpy(t);
        return a with { OriginX = x, OriginY = y, OriginZ = z, Roll = roll, Pitch = pitch, Yaw = yaw };
    }

    private static ParsedJoint MergeFixedIntoRevolute(ParsedJoint fixedJ, ParsedJoint revolute)
    {
        var tf = Transforms.FromRpy(fixedJ.OriginX, fixedJ.OriginY, fixedJ.OriginZ, fixedJ.Roll, fixedJ.Pitch, fixedJ.Yaw);
        var tr = Transforms.FromRpy(revolute.OriginX, revolute.OriginY, revolute.OriginZ, revolute.Roll, revolute.Pitch, revolute.Yaw);
        var t = Transforms.Multiply(tf, tr);
        var (x, y, z, roll, pitch, yaw) = MatrixToXyzRpy(t);
        return revolute with { OriginX = x, OriginY = y, OriginZ = z, Roll = roll, Pitch = pitch, Yaw = yaw };
    }

    private static (double x, double y, double z, double roll, double pitch, double yaw) MatrixToXyzRpy(double[] m)
    {
        var x = m[3]; var y = m[7]; var z = m[11];
        var pitch = Math.Asin(Math.Clamp(-m[8], -1, 1));
        double roll, yaw;
        if (Math.Abs(Math.Cos(pitch)) > 1e-6)
        {
            roll = Math.Atan2(m[9], m[10]);
            yaw = Math.Atan2(m[4], m[0]);
        }
        else
        {
            roll = Math.Atan2(-m[6], m[5]);
            yaw = 0;
        }
        return (x, y, z, roll, pitch, yaw);
    }

    private static ParsedJoint ParseJoint(XElement el)
    {
        var origin = el.Element("origin");
        var axis = el.Element("axis");
        var limit = el.Element("limit");
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
            limit?.Attribute("velocity") is { } v ? ParseDouble(v.Value, Math.PI) : null);
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
        double Lower, double Upper, double? Velocity);
}
