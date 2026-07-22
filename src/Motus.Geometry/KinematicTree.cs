using Motus.Core;

namespace Motus.Geometry;

public enum KinematicJointType
{
    Revolute,
    Continuous,
    Prismatic,
    Fixed
}

/// <summary>URDF <c>mimic</c>: q = multiplier * q[JointIndex] + offset.</summary>
public readonly record struct KinematicMimic(int JointIndex, double Multiplier, double Offset);

public sealed class KinematicLink
{
    public KinematicLink(string name, string? meshName = null, string? meshPath = null)
    {
        Name = name;
        MeshName = meshName;
        MeshPath = meshPath;
    }

    public string Name { get; }
    /// <summary>Optional mesh basename (not hashed into <see cref="KinematicTree.Fingerprint"/>).</summary>
    public string? MeshName { get; }
    /// <summary>Optional URDF mesh path/filename as declared (not hashed into fingerprint).</summary>
    public string? MeshPath { get; }
}

public sealed class KinematicJoint
{
    public KinematicJoint(
        string name,
        KinematicJointType type,
        int parentLinkIndex,
        int childLinkIndex,
        double originX, double originY, double originZ,
        double roll, double pitch, double yaw,
        double axisX, double axisY, double axisZ,
        double lower, double upper,
        double? velocity,
        int driverIndex,
        KinematicMimic? mimic)
    {
        Name = name;
        Type = type;
        ParentLinkIndex = parentLinkIndex;
        ChildLinkIndex = childLinkIndex;
        OriginX = originX; OriginY = originY; OriginZ = originZ;
        Roll = roll; Pitch = pitch; Yaw = yaw;
        AxisX = axisX; AxisY = axisY; AxisZ = axisZ;
        Lower = lower; Upper = upper;
        Velocity = velocity;
        DriverIndex = driverIndex;
        Mimic = mimic;
    }

    public string Name { get; }
    public KinematicJointType Type { get; }
    public int ParentLinkIndex { get; }
    public int ChildLinkIndex { get; }
    public double OriginX { get; }
    public double OriginY { get; }
    public double OriginZ { get; }
    public double Roll { get; }
    public double Pitch { get; }
    public double Yaw { get; }
    public double AxisX { get; }
    public double AxisY { get; }
    public double AxisZ { get; }
    public double Lower { get; }
    public double Upper { get; }
    public double? Velocity { get; }
    /// <summary>Index into driver q for non-mimic actuated joints; -1 if fixed or mimic.</summary>
    public int DriverIndex { get; }
    public KinematicMimic? Mimic { get; }

    public bool IsActuated => Type is KinematicJointType.Revolute
        or KinematicJointType.Continuous
        or KinematicJointType.Prismatic;
}

/// <summary>Legacy TCP-compatible serial tip extraction (fixed joints merged; trailing fixed → tip tool offset).</summary>
public readonly record struct SerialTipExtraction(
    SerialJointChain Chain,
    Frame? TipToolOffset,
    IReadOnlyList<string> JointNames);

/// <summary>
/// Kinematic tree (URDF-style). Link transforms are indexed in <see cref="Links"/> order.
/// Driver q order = non-mimic actuated joints in <see cref="Joints"/> order.
/// </summary>
public sealed class KinematicTree
{
    private readonly Dictionary<string, int> _linkIndex;

    public KinematicTree(
        string name,
        IReadOnlyList<KinematicLink> links,
        IReadOnlyList<KinematicJoint> joints,
        int rootLinkIndex,
        IReadOnlyList<int> driverJointIndices)
    {
        Name = name;
        Links = links;
        Joints = joints;
        RootLinkIndex = rootLinkIndex;
        DriverJointIndices = driverJointIndices;
        _linkIndex = new Dictionary<string, int>(links.Count, StringComparer.OrdinalIgnoreCase);
        for (var i = 0; i < links.Count; i++)
            _linkIndex[links[i].Name] = i;
        Fingerprint = ComputeFingerprint(name, links, joints, rootLinkIndex);
    }

    public string Name { get; }
    public IReadOnlyList<KinematicLink> Links { get; }
    public IReadOnlyList<KinematicJoint> Joints { get; }
    public int RootLinkIndex { get; }
    /// <summary>Indices into <see cref="Joints"/> for non-mimic actuated drivers (driver-q order).</summary>
    public IReadOnlyList<int> DriverJointIndices { get; }
    public int DriverCount => DriverJointIndices.Count;
    /// <summary>Cheap structural hash O(links+joints). Mesh paths are not included.</summary>
    public long Fingerprint { get; }

    public int IndexOfLink(string linkName) =>
        _linkIndex.TryGetValue(linkName, out var i)
            ? i
            : throw new InvalidOperationException($"Unknown link '{linkName}'.");

    public SerialJointChain ExtractSerialChain(string baseLink, string tipLink) =>
        ExtractSerialTip(baseLink, tipLink).Chain;

    /// <summary>
    /// Graft <paramref name="mechanism"/> (e.g. a tool mechanism projected from a
    /// <see cref="RobotDescription"/>) onto this tree at <paramref name="ontoLink"/> via a new fixed
    /// joint to <paramref name="mechanism"/>'s <paramref name="mechanismRootLink"/>, offset by
    /// <paramref name="attachFrame"/>. Unlike <see cref="RobotDescription.Attach"/> (translation-only
    /// joint origins — fine for from-scratch mechanisms, not for URDF-loaded arms with rotated joint
    /// origins), this supports full rotation, since <see cref="KinematicJoint"/> carries roll/pitch/yaw.
    /// Driver order is this tree's drivers followed by the mechanism's (existing driver-index callers —
    /// e.g. an arm's <c>JointNames</c> — are unaffected); <see cref="ExtractSerialTip"/> from this tree's
    /// own links is likewise unaffected since the graft only adds descendants below <paramref name="ontoLink"/>.
    /// </summary>
    public KinematicTree Attach(string ontoLink, KinematicTree mechanism, string mechanismRootLink, Frame attachFrame, string? name = null)
    {
        if (mechanism is null) throw new ArgumentNullException(nameof(mechanism));
        var ontoIndex = IndexOfLink(ontoLink);
        var mechRootIndex = mechanism.IndexOfLink(mechanismRootLink);
        var mechanismRootName = mechanism.Links[mechanism.RootLinkIndex].Name;
        if (!string.Equals(mechanismRootLink, mechanismRootName, StringComparison.OrdinalIgnoreCase))
            throw new ArgumentException(
                $"mechanismRootLink must be the mechanism root ('{mechanismRootName}'), not '{mechanismRootLink}'.",
                nameof(mechanismRootLink));

        var clashingLinks = mechanism.Links.Select(l => l.Name)
            .Where(n => _linkIndex.ContainsKey(n)).ToList();
        if (clashingLinks.Count > 0)
            throw new InvalidOperationException($"Attach: link name clash with '{Name}': {string.Join(", ", clashingLinks)}.");

        var ownJointNames = new HashSet<string>(Joints.Select(j => j.Name), StringComparer.OrdinalIgnoreCase);
        var mechJointNames = new HashSet<string>(mechanism.Joints.Select(j => j.Name), StringComparer.OrdinalIgnoreCase);
        var clashingJoints = mechanism.Joints.Select(j => j.Name)
            .Where(ownJointNames.Contains).ToList();
        if (clashingJoints.Count > 0)
            throw new InvalidOperationException($"Attach: joint name clash with '{Name}': {string.Join(", ", clashingJoints)}.");

        var joinName = $"{ontoLink}_to_{mechanismRootLink}_fixed";
        if (ownJointNames.Contains(joinName) || mechJointNames.Contains(joinName))
            throw new InvalidOperationException($"Attach: joint name '{joinName}' already exists.");

        var linkOffset = Links.Count;
        var jointOffset = Joints.Count + 1;

        var combinedLinks = new List<KinematicLink>(Links.Count + mechanism.Links.Count);
        combinedLinks.AddRange(Links);
        combinedLinks.AddRange(mechanism.Links);

        var (ox, oy, oz, roll, pitch, yaw) = FrameToXyzRpy(attachFrame);
        var attachJoint = new KinematicJoint(
            joinName, KinematicJointType.Fixed, ontoIndex, mechRootIndex + linkOffset,
            ox, oy, oz, roll, pitch, yaw,
            0, 0, 1, 0, 0, velocity: null, driverIndex: -1, mimic: null);

        var combinedJoints = new List<KinematicJoint>(Joints.Count + mechanism.Joints.Count + 1);
        combinedJoints.AddRange(Joints);
        combinedJoints.Add(attachJoint);
        var driverQOffset = DriverCount;
        foreach (var j in mechanism.Joints)
        {
            var mimic = j.Mimic is { } m ? new KinematicMimic(m.JointIndex + jointOffset, m.Multiplier, m.Offset) : (KinematicMimic?)null;
            // DriverIndex indexes the combined driver-q (arm drivers first, then mechanism).
            var driverIndex = j.DriverIndex < 0 ? -1 : j.DriverIndex + driverQOffset;
            combinedJoints.Add(new KinematicJoint(
                j.Name, j.Type,
                j.ParentLinkIndex + linkOffset, j.ChildLinkIndex + linkOffset,
                j.OriginX, j.OriginY, j.OriginZ,
                j.Roll, j.Pitch, j.Yaw,
                j.AxisX, j.AxisY, j.AxisZ,
                j.Lower, j.Upper, j.Velocity,
                driverIndex, mimic));
        }

        var driverIndices = new List<int>(DriverJointIndices.Count + mechanism.DriverJointIndices.Count);
        driverIndices.AddRange(DriverJointIndices);
        foreach (var di in mechanism.DriverJointIndices)
            driverIndices.Add(di + jointOffset);

        return new KinematicTree(name ?? $"{Name}+{mechanism.Name}", combinedLinks, combinedJoints, RootLinkIndex, driverIndices);
    }

    private static (double x, double y, double z, double roll, double pitch, double yaw) FrameToXyzRpy(Frame f) =>
        MatrixToXyzRpy(Transforms.FromFrame(f));

    public SerialTipExtraction ExtractSerialTip(string baseLink, string tipLink)
    {
        var byChild = new Dictionary<int, KinematicJoint>(Joints.Count);
        for (var i = 0; i < Joints.Count; i++)
        {
            var j = Joints[i];
            if (!byChild.TryAdd(j.ChildLinkIndex, j))
                throw new InvalidOperationException($"Multiple parents for link '{Links[j.ChildLinkIndex].Name}'.");
        }

        var baseIdx = IndexOfLink(baseLink);
        var tipIdx = IndexOfLink(tipLink);
        var path = new List<KinematicJoint>(8);
        var link = tipIdx;
        var guard = 0;
        while (link != baseIdx)
        {
            if (++guard > 256)
                throw new InvalidOperationException("URDF chain walk exceeded depth limit.");
            if (!byChild.TryGetValue(link, out var joint))
                throw new InvalidOperationException($"No joint with child link '{Links[link].Name}'.");
            path.Add(joint);
            link = joint.ParentLinkIndex;
        }

        path.Reverse();

        var merged = new List<JointDefinition>(path.Count);
        var names = new List<string>(path.Count);
        KinematicJoint? pendingFixed = null;

        for (var i = 0; i < path.Count; i++)
        {
            var j = path[i];
            if (j.Type == KinematicJointType.Fixed)
            {
                pendingFixed = pendingFixed is null ? j : MergeFixed(pendingFixed, j);
                continue;
            }

            if (!j.IsActuated)
                throw new InvalidOperationException($"Unsupported joint type '{j.Type}' on '{j.Name}'.");
            if (j.Mimic is not null)
                throw new InvalidOperationException($"Serial tip extract does not support mimic joint '{j.Name}' on the tip path.");

            if (pendingFixed is not null)
            {
                merged.Add(ToDefinition(MergeFixedIntoActuated(pendingFixed, j)));
                names.Add(j.Name);
                pendingFixed = null;
            }
            else
            {
                merged.Add(ToDefinition(j));
                names.Add(j.Name);
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

        return new SerialTipExtraction(new SerialJointChain(merged.ToArray()), tipOffset, names);
    }

    private static JointDefinition ToDefinition(KinematicJoint j) => new(
        j.OriginX, j.OriginY, j.OriginZ,
        j.Roll, j.Pitch, j.Yaw,
        j.AxisX, j.AxisY, j.AxisZ,
        Motion: j.Type == KinematicJointType.Prismatic ? JointMotionType.Prismatic : JointMotionType.Revolute);

    private static KinematicJoint MergeFixed(KinematicJoint a, KinematicJoint b)
    {
        var ta = Transforms.FromRpy(a.OriginX, a.OriginY, a.OriginZ, a.Roll, a.Pitch, a.Yaw);
        var tb = Transforms.FromRpy(b.OriginX, b.OriginY, b.OriginZ, b.Roll, b.Pitch, b.Yaw);
        var t = Transforms.Multiply(ta, tb);
        var (x, y, z, roll, pitch, yaw) = MatrixToXyzRpy(t);
        return CloneOrigin(a, x, y, z, roll, pitch, yaw);
    }

    private static KinematicJoint MergeFixedIntoActuated(KinematicJoint fixedJ, KinematicJoint actuated)
    {
        var tf = Transforms.FromRpy(fixedJ.OriginX, fixedJ.OriginY, fixedJ.OriginZ, fixedJ.Roll, fixedJ.Pitch, fixedJ.Yaw);
        var tr = Transforms.FromRpy(actuated.OriginX, actuated.OriginY, actuated.OriginZ, actuated.Roll, actuated.Pitch, actuated.Yaw);
        var t = Transforms.Multiply(tf, tr);
        var (x, y, z, roll, pitch, yaw) = MatrixToXyzRpy(t);
        return CloneOrigin(actuated, x, y, z, roll, pitch, yaw);
    }

    private static KinematicJoint CloneOrigin(KinematicJoint src, double x, double y, double z, double roll, double pitch, double yaw) =>
        new(src.Name, src.Type, src.ParentLinkIndex, src.ChildLinkIndex,
            x, y, z, roll, pitch, yaw,
            src.AxisX, src.AxisY, src.AxisZ,
            src.Lower, src.Upper, src.Velocity, src.DriverIndex, src.Mimic);

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

    // ponytail: FNV-1a structural hash; mesh refs intentionally omitted
    private static long ComputeFingerprint(
        string name,
        IReadOnlyList<KinematicLink> links,
        IReadOnlyList<KinematicJoint> joints,
        int rootLinkIndex)
    {
        unchecked
        {
            long h = unchecked((long)14695981039346656037UL);
            h = Mix(h, name);
            h = Mix(h, rootLinkIndex);
            h = Mix(h, links.Count);
            for (var i = 0; i < links.Count; i++)
                h = Mix(h, links[i].Name);
            h = Mix(h, joints.Count);
            for (var i = 0; i < joints.Count; i++)
            {
                var j = joints[i];
                h = Mix(h, j.Name);
                h = Mix(h, (int)j.Type);
                h = Mix(h, j.ParentLinkIndex);
                h = Mix(h, j.ChildLinkIndex);
                h = Mix(h, j.OriginX); h = Mix(h, j.OriginY); h = Mix(h, j.OriginZ);
                h = Mix(h, j.Roll); h = Mix(h, j.Pitch); h = Mix(h, j.Yaw);
                h = Mix(h, j.AxisX); h = Mix(h, j.AxisY); h = Mix(h, j.AxisZ);
                h = Mix(h, j.Lower); h = Mix(h, j.Upper);
                if (j.Mimic is { } m)
                {
                    h = Mix(h, 1);
                    h = Mix(h, m.JointIndex);
                    h = Mix(h, m.Multiplier);
                    h = Mix(h, m.Offset);
                }
                else
                {
                    h = Mix(h, 0);
                }
            }
            return h;
        }
    }

    private static long Mix(long h, int v)
    {
        unchecked
        {
            h ^= v;
            return h * unchecked((long)1099511628211UL);
        }
    }

    private static long Mix(long h, string s)
    {
        unchecked
        {
            for (var i = 0; i < s.Length; i++)
            {
                h ^= s[i];
                h *= unchecked((long)1099511628211UL);
            }
            h ^= 0xFF;
            return h * unchecked((long)1099511628211UL);
        }
    }

    private static long Mix(long h, double v)
    {
        var bits = BitConverter.DoubleToInt64Bits(v);
        h = Mix(h, (int)bits);
        return Mix(h, (int)(bits >> 32));
    }
}
