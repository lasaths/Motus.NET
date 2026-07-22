using Motus.Core;

namespace Motus.Geometry;

/// <summary>Result of validating a candidate <see cref="RobotDescription"/> link/joint graph.</summary>
public sealed class AssembleDiagnostics
{
    public AssembleDiagnostics(IReadOnlyList<string> errors, IReadOnlyList<string> warnings)
    {
        Errors = errors;
        Warnings = warnings;
    }

    public IReadOnlyList<string> Errors { get; }
    public IReadOnlyList<string> Warnings { get; }
    public bool Ok => Errors.Count == 0;
}

/// <summary>
/// In-memory URDF-style robot description: a validated tree of <see cref="UrdfLink"/> connected by
/// <see cref="UrdfJoint"/>. Units are meters; joint origin/axis are expressed in the parent link's frame.
/// Build via <see cref="TryAssemble"/> or <see cref="Assemble"/>; convert to a <see cref="KinematicTree"/>
/// via <see cref="ToKinematicTree"/> for FK/IK/collision.
/// </summary>
public sealed class RobotDescription
{
    public string Name { get; }
    public IReadOnlyList<UrdfLink> Links { get; }
    public IReadOnlyList<UrdfJoint> Joints { get; }
    public string? TipLink { get; }
    /// <summary>Optional home joint values in driver order (non-mimic, actuated joints, <see cref="Joints"/> order).</summary>
    public IReadOnlyList<double>? HomeQ { get; }
    /// <summary>Cheap structural hash (names/topology/limits). Mesh vertex/index buffers are not hashed.</summary>
    public long Fingerprint { get; }
    /// <summary>The single link with no parent joint.</summary>
    public string RootLinkName { get; }

    private RobotDescription(
        string name,
        IReadOnlyList<UrdfLink> links,
        IReadOnlyList<UrdfJoint> joints,
        string? tipLink,
        IReadOnlyList<double>? homeQ,
        string rootLinkName)
    {
        Name = name;
        Links = links;
        Joints = joints;
        TipLink = tipLink;
        HomeQ = homeQ;
        RootLinkName = rootLinkName;
        Fingerprint = ComputeFingerprint(name, links, joints, tipLink, rootLinkName);
    }

    /// <summary>
    /// Validate and build a <see cref="RobotDescription"/> from links/joints. Requires exactly one root
    /// link (no incoming joint), no cycles, all parent/child/mimic references resolved, and unique names.
    /// </summary>
    public static bool TryAssemble(
        string name,
        IReadOnlyList<UrdfLink> links,
        IReadOnlyList<UrdfJoint> joints,
        string? tipLink,
        out RobotDescription? description,
        out AssembleDiagnostics diagnostics,
        IReadOnlyList<double>? homeQ = null)
    {
        description = null;
        var errors = new List<string>();
        var warnings = new List<string>();

        links ??= [];
        joints ??= [];

        if (links.Count == 0)
        {
            errors.Add("RobotDescription requires at least one link.");
            diagnostics = new AssembleDiagnostics(errors, warnings);
            return false;
        }

        var linkIndex = new Dictionary<string, int>(links.Count, StringComparer.Ordinal);
        for (var i = 0; i < links.Count; i++)
        {
            var linkName = links[i].Name;
            if (string.IsNullOrWhiteSpace(linkName))
            {
                errors.Add($"Link at index {i} has an empty name.");
                continue;
            }
            if (!linkIndex.TryAdd(linkName, i))
                errors.Add($"Duplicate link name '{linkName}'.");
        }

        var jointNames = new Dictionary<string, int>(joints.Count, StringComparer.Ordinal);
        for (var i = 0; i < joints.Count; i++)
        {
            var jointName = joints[i].Name;
            if (string.IsNullOrWhiteSpace(jointName))
            {
                errors.Add($"Joint at index {i} has an empty name.");
                continue;
            }
            if (!jointNames.TryAdd(jointName, i))
                errors.Add($"Duplicate joint name '{jointName}'.");
        }

        var incomingJointByChild = new Dictionary<string, string>(StringComparer.Ordinal);
        var childJointsByParent = new Dictionary<string, List<int>>(StringComparer.Ordinal);
        foreach (var linkName in linkIndex.Keys)
            childJointsByParent[linkName] = [];

        for (var i = 0; i < joints.Count; i++)
        {
            var j = joints[i];
            var parentOk = linkIndex.ContainsKey(j.ParentLink);
            var childOk = linkIndex.ContainsKey(j.ChildLink);
            if (!parentOk)
                errors.Add($"Joint '{j.Name}' references unknown parent link '{j.ParentLink}'.");
            if (!childOk)
                errors.Add($"Joint '{j.Name}' references unknown child link '{j.ChildLink}'.");
            if (!parentOk || !childOk)
                continue;

            if (string.Equals(j.ParentLink, j.ChildLink, StringComparison.Ordinal))
            {
                errors.Add($"Joint '{j.Name}' has identical parent and child link '{j.ParentLink}'.");
                continue;
            }

            if (incomingJointByChild.TryGetValue(j.ChildLink, out var existing))
                errors.Add($"Link '{j.ChildLink}' has multiple parent joints ('{existing}' and '{j.Name}'); only one parent is allowed.");
            else
                incomingJointByChild[j.ChildLink] = j.Name;

            childJointsByParent[j.ParentLink].Add(i);
        }

        foreach (var j in joints)
        {
            if (j.MimicJoint is null) continue;
            if (!jointNames.ContainsKey(j.MimicJoint))
            {
                errors.Add($"Joint '{j.Name}' mimics unknown joint '{j.MimicJoint}'.");
                continue;
            }
            if (string.Equals(j.MimicJoint, j.Name, StringComparison.Ordinal))
                errors.Add($"Joint '{j.Name}' cannot mimic itself.");
        }

        if (errors.Count > 0)
        {
            diagnostics = new AssembleDiagnostics(errors, warnings);
            return false;
        }

        var roots = linkIndex.Keys.Where(l => !incomingJointByChild.ContainsKey(l)).ToList();
        if (roots.Count == 0)
        {
            errors.Add("No root link found — every link has a parent joint, so the joint graph contains a cycle.");
            diagnostics = new AssembleDiagnostics(errors, warnings);
            return false;
        }
        if (roots.Count > 1)
        {
            errors.Add($"Multiple root links found ({string.Join(", ", roots)}); expected a single connected tree. Use Attach to merge separate descriptions.");
            diagnostics = new AssembleDiagnostics(errors, warnings);
            return false;
        }

        var root = roots[0];

        var visited = new HashSet<string>(StringComparer.Ordinal);
        var stack = new Stack<string>();
        stack.Push(root);
        while (stack.Count > 0)
        {
            var current = stack.Pop();
            if (!visited.Add(current)) continue;
            foreach (var jointIdx in childJointsByParent[current])
                stack.Push(joints[jointIdx].ChildLink);
        }

        if (visited.Count != linkIndex.Count)
        {
            var unreached = linkIndex.Keys.Where(l => !visited.Contains(l));
            errors.Add($"Unreachable link(s) from root '{root}': {string.Join(", ", unreached)}. The joint graph may contain a cycle or disconnected branch.");
        }

        if (!string.IsNullOrWhiteSpace(tipLink) && !linkIndex.ContainsKey(tipLink))
            errors.Add($"TipLink '{tipLink}' is not a known link.");

        var driverCount = joints.Count(j => j.IsActuated && j.MimicJoint is null);
        if (homeQ is not null && homeQ.Count != driverCount)
            warnings.Add($"HomeQ length ({homeQ.Count}) does not match driver joint count ({driverCount}).");

        if (errors.Count > 0)
        {
            diagnostics = new AssembleDiagnostics(errors, warnings);
            return false;
        }

        diagnostics = new AssembleDiagnostics(errors, warnings);
        description = new RobotDescription(name, links, joints, tipLink, homeQ, root);
        return true;
    }

    /// <summary>Like <see cref="TryAssemble"/> but throws with the collected errors on failure.</summary>
    public static RobotDescription Assemble(
        string name,
        IReadOnlyList<UrdfLink> links,
        IReadOnlyList<UrdfJoint> joints,
        string? tipLink = null,
        IReadOnlyList<double>? homeQ = null)
    {
        if (!TryAssemble(name, links, joints, tipLink, out var description, out var diagnostics, homeQ))
            throw new InvalidOperationException(
                $"RobotDescription.Assemble('{name}') failed: {string.Join("; ", diagnostics.Errors)}");
        return description!;
    }

    /// <summary>
    /// Attach <paramref name="child"/> to this description via a new fixed joint from
    /// <paramref name="parentLink"/> to <paramref name="child"/>'s root link. Names must not clash between
    /// the two descriptions. Mimic references are untouched (they are name-based, not index-based) and
    /// remain valid after merging.
    /// </summary>
    /// <remarks>
    /// <see cref="UrdfJoint"/> origins in this schema are translation-only, so <paramref name="attachFrame"/>
    /// must carry an identity rotation; pre-rotate the child's own links/axes if a rotated mount is required.
    /// </remarks>
    public RobotDescription Attach(RobotDescription child, string parentLink, Frame attachFrame, string? attachJointName = null)
    {
        if (child is null) throw new ArgumentNullException(nameof(child));
        if (string.IsNullOrWhiteSpace(parentLink))
            throw new ArgumentException("Parent link is required.", nameof(parentLink));
        if (!Links.Any(l => string.Equals(l.Name, parentLink, StringComparison.Ordinal)))
            throw new ArgumentException($"Unknown parent link '{parentLink}'.", nameof(parentLink));
        if (!IsIdentityRotation(attachFrame))
            throw new NotSupportedException(
                "Attach: UrdfJoint origins are translation-only in this schema; attachFrame must have an identity rotation. " +
                "Pre-rotate the child's own axes/links before attaching if a rotated mount is required.");

        var ownLinkNames = new HashSet<string>(Links.Select(l => l.Name), StringComparer.Ordinal);
        var ownJointNames = new HashSet<string>(Joints.Select(j => j.Name), StringComparer.Ordinal);

        var clashingLinks = child.Links.Select(l => l.Name).Where(ownLinkNames.Contains).ToList();
        if (clashingLinks.Count > 0)
            throw new InvalidOperationException($"Attach: link name clash with '{Name}': {string.Join(", ", clashingLinks)}.");

        var clashingJoints = child.Joints.Select(j => j.Name).Where(ownJointNames.Contains).ToList();
        if (clashingJoints.Count > 0)
            throw new InvalidOperationException($"Attach: joint name clash with '{Name}': {string.Join(", ", clashingJoints)}.");

        var joinName = attachJointName ?? $"{parentLink}_to_{child.RootLinkName}_fixed";
        if (ownJointNames.Contains(joinName) || child.Joints.Any(j => string.Equals(j.Name, joinName, StringComparison.Ordinal)))
            throw new InvalidOperationException($"Attach: joint name '{joinName}' already exists.");

        var attachJoint = new UrdfJoint(
            joinName, "fixed", parentLink, child.RootLinkName,
            attachFrame.X, attachFrame.Y, attachFrame.Z,
            0, 0, 1,
            0, 0);

        var combinedLinks = new List<UrdfLink>(Links.Count + child.Links.Count);
        combinedLinks.AddRange(Links);
        combinedLinks.AddRange(child.Links);

        var combinedJoints = new List<UrdfJoint>(Joints.Count + child.Joints.Count + 1);
        combinedJoints.AddRange(Joints);
        combinedJoints.AddRange(child.Joints);
        combinedJoints.Add(attachJoint);

        var combinedHomeQ = CombineHomeQ(HomeQ, child.HomeQ);

        return Assemble(Name, combinedLinks, combinedJoints, TipLink, combinedHomeQ);
    }

    /// <summary>Flatten back to raw links/joints (e.g. to feed a URDF writer downstream).</summary>
    public (IReadOnlyList<UrdfLink> Links, IReadOnlyList<UrdfJoint> Joints) Explode() => (Links, Joints);

    /// <summary>
    /// Build a <see cref="KinematicTree"/> for FK/IK/collision. Joint origins carry no rotation in this
    /// schema, so <c>Roll</c>/<c>Pitch</c>/<c>Yaw</c> are always zero; mimic and driver ordering follow the
    /// same rules as <see cref="JointTableTrees"/> / the URDF loader.
    /// </summary>
    public KinematicTree ToKinematicTree()
    {
        var linkIndex = new Dictionary<string, int>(Links.Count, StringComparer.Ordinal);
        for (var i = 0; i < Links.Count; i++)
            linkIndex[Links[i].Name] = i;

        var kLinks = new KinematicLink[Links.Count];
        for (var i = 0; i < Links.Count; i++)
        {
            var link = Links[i];
            var mesh = link.Visuals.FirstOrDefault(v => v.Kind == UrdfGeometryKind.Mesh)
                       ?? link.Collisions.FirstOrDefault(v => v.Kind == UrdfGeometryKind.Mesh);
            var meshPath = mesh?.FilePath;
            var meshName = meshPath is null ? null : Path.GetFileName(meshPath.Replace('\\', '/'));
            kLinks[i] = new KinematicLink(link.Name, meshName, meshPath);
        }

        var jointIndexByName = new Dictionary<string, int>(Joints.Count, StringComparer.Ordinal);
        for (var i = 0; i < Joints.Count; i++)
            jointIndexByName[Joints[i].Name] = i;

        var drivers = new List<int>();
        var kJoints = new KinematicJoint[Joints.Count];
        for (var i = 0; i < Joints.Count; i++)
        {
            var j = Joints[i];
            var type = ToKinematicType(j.Kind);

            KinematicMimic? mimic = null;
            var driverIndex = -1;
            if (type != KinematicJointType.Fixed)
            {
                if (j.MimicJoint is not null)
                    mimic = new KinematicMimic(jointIndexByName[j.MimicJoint], j.MimicMultiplier, j.MimicOffset);
                else
                {
                    driverIndex = drivers.Count;
                    drivers.Add(i);
                }
            }

            kJoints[i] = new KinematicJoint(
                j.Name, type,
                linkIndex[j.ParentLink], linkIndex[j.ChildLink],
                j.OriginX, j.OriginY, j.OriginZ,
                0, 0, 0,
                j.AxisX, j.AxisY, j.AxisZ,
                j.Lower, j.Upper,
                velocity: null,
                driverIndex, mimic);
        }

        return new KinematicTree(Name, kLinks, kJoints, linkIndex[RootLinkName], drivers);
    }

    /// <summary>
    /// Frame of <paramref name="tipLink"/> (default <see cref="TipLink"/>, else <see cref="RootLinkName"/>)
    /// in the root link frame at <see cref="HomeQ"/> (zeros when unset). When HomeQ is unset/zero this is
    /// the cheap translation-only origin sum; otherwise TreeFK at HomeQ.
    /// </summary>
    public Frame TipTcp(string? tipLink = null)
    {
        var target = tipLink ?? TipLink ?? RootLinkName;
        if (!Links.Any(l => string.Equals(l.Name, target, StringComparison.Ordinal)))
            throw new ArgumentException($"Unknown link '{target}'.", nameof(tipLink));
        if (string.Equals(target, RootLinkName, StringComparison.Ordinal))
            return Frame.Identity;

        if (HomeQNeedsFk())
        {
            var tree = ToKinematicTree();
            var tipIdx = tree.IndexOfLink(target);
            var q = ResolveHomeDriverQ(tree.DriverCount);
            var fk = new TreeForwardKinematics(tree);
            var mats = new double[tree.Links.Count][];
            for (var i = 0; i < mats.Length; i++)
                mats[i] = new double[16];
            fk.ComputeLinkTransformsInto(q, mats);
            return Transforms.ToFrame(mats[tipIdx]);
        }

        var byChild = new Dictionary<string, UrdfJoint>(StringComparer.Ordinal);
        foreach (var j in Joints)
            byChild[j.ChildLink] = j;

        var chain = new List<UrdfJoint>();
        var current = target;
        var guard = 0;
        while (!string.Equals(current, RootLinkName, StringComparison.Ordinal))
        {
            if (++guard > Links.Count + 4)
                throw new InvalidOperationException($"Path from root to '{target}' exceeded depth limit.");
            if (!byChild.TryGetValue(current, out var joint))
                throw new InvalidOperationException($"No joint path from root '{RootLinkName}' to link '{current}'.");
            chain.Add(joint);
            current = joint.ParentLink;
        }
        chain.Reverse();

        var m = Transforms.Identity();
        foreach (var j in chain)
        {
            var local = Transforms.FromRpy(j.OriginX, j.OriginY, j.OriginZ, 0, 0, 0);
            m = Transforms.Multiply(m, local);
        }
        return Transforms.ToFrame(m);
    }

    private bool HomeQNeedsFk()
    {
        if (HomeQ is null || HomeQ.Count == 0) return false;
        for (var i = 0; i < HomeQ.Count; i++)
        {
            if (Math.Abs(HomeQ[i]) > 1e-12) return true;
        }
        return false;
    }

    private double[] ResolveHomeDriverQ(int driverCount)
    {
        var q = new double[driverCount];
        if (HomeQ is null) return q;
        var n = Math.Min(driverCount, HomeQ.Count);
        for (var i = 0; i < n; i++)
            q[i] = HomeQ[i];
        return q;
    }

    private static bool IsIdentityRotation(Frame f)
    {
        const double tol = 1e-9;
        return Math.Abs(Math.Abs(f.Qw) - 1) < tol && Math.Abs(f.Qx) < tol && Math.Abs(f.Qy) < tol && Math.Abs(f.Qz) < tol;
    }

    private static IReadOnlyList<double>? CombineHomeQ(IReadOnlyList<double>? parent, IReadOnlyList<double>? child)
    {
        if (parent is null || child is null) return null;
        var combined = new double[parent.Count + child.Count];
        for (var i = 0; i < parent.Count; i++) combined[i] = parent[i];
        for (var i = 0; i < child.Count; i++) combined[parent.Count + i] = child[i];
        return combined;
    }

    private static KinematicJointType ToKinematicType(UrdfJointKind kind) => kind switch
    {
        UrdfJointKind.Revolute => KinematicJointType.Revolute,
        UrdfJointKind.Continuous => KinematicJointType.Continuous,
        UrdfJointKind.Prismatic => KinematicJointType.Prismatic,
        UrdfJointKind.Fixed => KinematicJointType.Fixed,
        _ => throw new InvalidOperationException($"Unsupported joint kind '{kind}'.")
    };

    // ponytail: FNV-1a structural hash; mesh vertex/index buffers intentionally excluded
    private static long ComputeFingerprint(
        string name,
        IReadOnlyList<UrdfLink> links,
        IReadOnlyList<UrdfJoint> joints,
        string? tipLink,
        string rootLinkName)
    {
        unchecked
        {
            long h = unchecked((long)14695981039346656037UL);
            h = Mix(h, name);
            h = Mix(h, rootLinkName);
            h = Mix(h, tipLink ?? "");
            h = Mix(h, links.Count);
            for (var i = 0; i < links.Count; i++)
            {
                var l = links[i];
                h = Mix(h, l.Name);
                h = Mix(h, l.Mass ?? double.NaN);
            }
            h = Mix(h, joints.Count);
            for (var i = 0; i < joints.Count; i++)
            {
                var j = joints[i];
                h = Mix(h, j.Name);
                h = Mix(h, (int)j.Kind);
                h = Mix(h, j.ParentLink);
                h = Mix(h, j.ChildLink);
                h = Mix(h, j.OriginX); h = Mix(h, j.OriginY); h = Mix(h, j.OriginZ);
                h = Mix(h, j.AxisX); h = Mix(h, j.AxisY); h = Mix(h, j.AxisZ);
                h = Mix(h, j.Lower); h = Mix(h, j.Upper);
                if (j.MimicJoint is { } m)
                {
                    h = Mix(h, 1);
                    h = Mix(h, m);
                    h = Mix(h, j.MimicMultiplier);
                    h = Mix(h, j.MimicOffset);
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
