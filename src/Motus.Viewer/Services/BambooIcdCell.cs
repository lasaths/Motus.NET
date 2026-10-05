using System.Globalization;
using System.Xml.Linq;
using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Viewer.Services;

/// <summary>
/// The ICD/LIS bamboo work cell as Motus.NET sees it: the hung arm loaded from <c>bamboo_icd.urdf</c>
/// (link frames == viewer Three.js groups; collision boxes == drawn boxes) plus a <see cref="CollisionScene"/>
/// with the support pole, the strut, the ground slab and the fixed pole mount (base tube + L-grip).
/// Motus is the only collision authority: every answer comes from <see cref="RobotMeshCollisionChecker"/>
/// (exact OBB / triangle narrow phase) via <see cref="ICollisionContactReporter.FindContacts"/>.
/// <para>
/// Frames: viewer is Y-up, Motus is Z-up; <c>Motus = Rx(+90°) · viewer</c>, i.e. viewer (x, y, z) → Motus (x, −z, y).
/// The URDF's fixed <c>pole_hang</c> joint carries that rotation plus the hang at viewer y = 0.40, so Motus FK
/// link poses are the drawn poses — no separate "visual" kinematics for collision.
/// </para>
/// </summary>
public sealed class BambooIcdCell
{
    // Environment geometry (viewer metres). viewer.js draws pole / strut / ground from these same numbers (CellJson).
    public const double PoleRadius = 0.012;
    public const double PoleLength = 0.66;
    public const int PoleSegments = 24;
    public const double StrutRadius = 0.007;
    public const double StrutLength = 0.30;
    public const int StrutSegments = 20;
    public const double GroundSize = 2.2;
    public const double GroundThickness = 0.05;
    /// <summary>Only directly-jointed links skip self checks; every other arm part pair is tested.</summary>
    public const int SelfCollisionMinLinkGap = 2;

    public const string Pole = "pole";
    public const string Strut = "strut";
    public const string Ground = "ground";

    public static readonly string[] RGripPlates = { "R-grip jaw-", "R-grip jaw+" };
    public static readonly string[] LGripPlates = { "L-grip jaw-", "L-grip jaw+" };

    /// <summary>Motus world ← viewer world (Rx(+90°)).</summary>
    public static readonly double[] ViewerToMotus = Transforms.FromAxisAngle(1, 0, 0, Math.PI / 2);

    public UrdfRobot Urdf { get; }
    public RobotModel Model { get; }
    public SerialJointChain Chain { get; }
    public IFkSolver Fk { get; }
    /// <summary>Motus world pose of the fixed mount link (from the URDF <c>pole_hang</c> joint).</summary>
    public double[] MountWorld { get; }
    /// <summary>Box parts per URDF link (mount + link_1..link_5), for drawing and reporting.</summary>
    public IReadOnlyList<CellPart> Parts { get; }
    /// <summary>Joint offsets along viewer −Y (pole→θ1, θ1→θ2, …, θ5→jaw) read from the URDF.</summary>
    public double[] Seg { get; }

    private readonly IReadOnlyList<LinkCollisionGeometry> _mountGeometry;
    private readonly List<double[]> _strutVerts;
    private readonly List<int> _strutIdx;
    private readonly CollisionObject _poleObj;
    private readonly CollisionObject _groundObj;
    private readonly Dictionary<(int gR, bool held, int off), RobotMeshCollisionChecker> _checkers = new();

    public sealed record CellPart(string Name, string Link, double[] Size, double[] Pos);

    /// <summary>Strut placement on the ground in viewer coordinates (centre + pitch about X, degrees).</summary>
    public readonly record struct StrutPlacement(double X, double Y, double Z, double PitchDeg);

    private BambooIcdCell(UrdfRobot urdf, XElement root)
    {
        Urdf = urdf;
        Model = urdf.ToModel();
        Chain = urdf.Chain;
        Fk = KinematicsResolver.CreateFkSolver(urdf.Preset, urdf.Chain);

        var tree = urdf.Tree ?? throw new InvalidOperationException("bamboo URDF: kinematic tree missing.");
        KinematicJoint? hang = null;
        foreach (var j in tree.Joints)
            if (string.Equals(tree.Links[j.ChildLinkIndex].Name, "mount", StringComparison.OrdinalIgnoreCase)) hang = j;
        if (hang is null) throw new InvalidOperationException("bamboo URDF: no joint into 'mount'.");
        MountWorld = Transforms.FromRpy(hang.OriginX, hang.OriginY, hang.OriginZ, hang.Roll, hang.Pitch, hang.Yaw);

        _mountGeometry = UrdfCollisionLoader.Load(root, new[] { "mount" }, ".")?.Links
                         ?? throw new InvalidOperationException("bamboo URDF: mount has no collision.");

        var parts = new List<CellPart>();
        foreach (var g in _mountGeometry) parts.Add(ToPart(g.LocalGeometry, "mount"));
        foreach (var g in Model.CollisionModel?.Links ?? Array.Empty<LinkCollisionGeometry>())
            parts.Add(ToPart(g.LocalGeometry, g.LinkName));
        Parts = parts;

        var seg = new List<double>();
        foreach (var name in new[] { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "tool_joint" })
        {
            var j = tree.Joints.First(x => x.Name == name);
            seg.Add(-j.OriginY);
        }
        Seg = seg.ToArray();

        (_strutVerts, _strutIdx) = CylinderMesh(StrutRadius, StrutLength, StrutSegments, alongZ: true);
        var (pv, pi) = CylinderMesh(PoleRadius, PoleLength, PoleSegments, alongZ: false);
        // viewer: pole.rotation.z = 90°, position (0, POLE_Y, 0)
        var poleViewer = Transforms.Multiply(
            Transforms.FromFrame(new Frame(0, BambooIcdMotion.PoleY, 0)),
            Transforms.FromAxisAngle(0, 0, 1, Math.PI / 2));
        _poleObj = CollisionObject.Mesh(Pole, ViewerPoseToMotus(poleViewer), pv, pi);
        _groundObj = CollisionObject.Box(Ground, new Frame(0, 0, -GroundThickness / 2),
            GroundSize / 2, GroundSize / 2, GroundThickness / 2);
    }

    public static BambooIcdCell Load(string urdfXml)
    {
        var doc = XDocument.Parse(urdfXml);
        var urdf = UrdfRobotLoader.Load(doc, new UrdfLoadOptions { BaseLink = "base_link", TipLink = "tool0" });
        return new BambooIcdCell(urdf, doc.Root!);
    }

    public static BambooIcdCell LoadFile(string path) => Load(File.ReadAllText(path));

    private static CellPart ToPart(CollisionObject g, string link) =>
        new(g.Name, link,
            new[] { g.ExtentX * 2, g.ExtentY * 2, g.ExtentZ * 2 },
            new[] { g.Pose.X, g.Pose.Y, g.Pose.Z });

    public static Frame ViewerPoseToMotus(double[] viewerM) => Transforms.ToFrame(Transforms.Multiply(ViewerToMotus, viewerM));

    /// <summary>Viewer point → Motus point.</summary>
    public static (double X, double Y, double Z) ViewerToMotusPoint(double x, double y, double z) => (x, -z, y);

    /// <summary>Jaw-plate centre offset along jaw X for an opening in mm (same formula as viewer.js setOpening).</summary>
    public static double PlateShift(double openingMm) => openingMm / 1000.0 / 2 + 0.002;

    public static double[] RadFromDeg(IReadOnlyList<double> deg)
    {
        var q = new double[deg.Count];
        for (var i = 0; i < q.Length; i++) q[i] = deg[i] * Math.PI / 180.0;
        return q;
    }

    /// <summary>Three.js CylinderGeometry tessellation (side + fan caps); along local Y, or along Z after rotateX(90°).</summary>
    public static (List<double[]> Verts, List<int> Idx) CylinderMesh(double r, double h, int seg, bool alongZ)
    {
        var verts = new List<double[]>();
        var idx = new List<int>();
        double[] V(double x, double y, double z) => alongZ ? new[] { x, -z, y } : new[] { x, y, z };
        for (var s = 0; s < seg; s++)
        {
            var th = 2 * Math.PI * s / seg;
            verts.Add(V(r * Math.Sin(th), h / 2, r * Math.Cos(th)));
            verts.Add(V(r * Math.Sin(th), -h / 2, r * Math.Cos(th)));
        }
        var top = verts.Count; verts.Add(V(0, h / 2, 0));
        var bot = verts.Count; verts.Add(V(0, -h / 2, 0));
        for (var s = 0; s < seg; s++)
        {
            var a = 2 * s; var b = 2 * ((s + 1) % seg);
            idx.AddRange(new[] { a, a + 1, b + 1, a, b + 1, b });
            idx.AddRange(new[] { top, a, b });
            idx.AddRange(new[] { bot, b + 1, a + 1 });
        }
        return (verts, idx);
    }

    /// <summary>Viewer pose T(p)·Rx(pitch) of a strut placement → Motus world matrix.</summary>
    public static double[] PlacementWorld(StrutPlacement p) =>
        Transforms.Multiply(ViewerToMotus, Transforms.Multiply(
            Transforms.FromFrame(new Frame(p.X, p.Y, p.Z)),
            Transforms.FromAxisAngle(1, 0, 0, p.PitchDeg * Math.PI / 180.0)));

    /// <summary>Strut mesh at a Motus world pose (scene obstacle).</summary>
    public CollisionObject StrutAt(double[] motusWorld, string? name = null) =>
        CollisionObject.Mesh(name ?? Strut, Transforms.ToFrame(motusWorld), _strutVerts, _strutIdx);

    /// <summary>Strut held in the R-grip at its true jaw-local offset (captured when the jaws close).</summary>
    public AttachedBody HeldStrut(Frame tcpLocal) =>
        new(Strut, tcpLocal, CollisionObject.Mesh(Strut, Frame.Identity, _strutVerts, _strutIdx), Strut);

    /// <summary>Motus world TCP (jaw) matrix from Motus FK.</summary>
    public double[] JawWorld(IReadOnlyList<double> qDeg) =>
        Fk.ComputeTcpTransform(RadFromDeg(qDeg), Model.Preset.BaseFrame.Frame, Model.Preset.ToolFrame.Frame);

    /// <summary>
    /// Where the strut physically is through a motion. It rests on the ground at the pick object's location
    /// (the grasp plane's un-nudged origin — nudging a task plane moves the robot's aim, not the object).
    /// When the jaws close it is captured at its real jaw-local offset (no snapping to the jaw origin); when
    /// they open it stays exactly where the jaw left it. If the closed jaws would not enclose it, the grasp
    /// misses and the strut never leaves the ground.
    /// </summary>
    public sealed class StrutTrack
    {
        public required double[] Rest { get; init; }
        public Frame? HeldOffset { get; init; }
        public double[]? Released { get; init; }
        public string? Miss { get; init; }
        public bool Captured => HeldOffset is not null;
    }

    public StrutTrack TrackStrut(IReadOnlyList<BambooIcdMotion.MotionKey> keys, StrutPlacement rest)
    {
        var restM = PlacementWorld(rest);
        var close = keys.FirstOrDefault(k => k.Hold == 1);
        if (close is null) return new StrutTrack { Rest = restM };
        var jaw = JawWorld(close.Q);
        var offsetM = Transforms.Multiply(Transforms.Inverse(jaw), restM);
        // Offset in jaw frame (== viewer jaw group frame): plates span jaw-y [-5, +15] mm, jaw x ±(inner gap).
        var ox = offsetM[3]; var oy = offsetM[7]; var oz = offsetM[11];
        var axisZ = Math.Abs(offsetM[10]); // strut axis (local Z) · jaw Z
        var openInner = PlateShift(BambooIcdMotion.OpenJawMm(keys)) - 0.00225;
        string? miss = null;
        if (oy - StrutRadius > 0.015 || oy + StrutRadius < -0.005)
            miss = $"grasp misses strut (strut axis {oy * 1000:F1} mm from jaw along jaw Y)";
        else if (Math.Abs(ox) + StrutRadius > openInner)
            miss = $"grasp misses strut (strut {ox * 1000:F1} mm off jaw centre)";
        else if (axisZ < Math.Cos(20 * Math.PI / 180) || Math.Abs(oz) > StrutLength / 2)
            miss = "grasp misses strut (jaw not across the strut)";
        if (miss is not null) return new StrutTrack { Rest = restM, Miss = miss };

        var offset = Transforms.ToFrame(offsetM);
        var open = keys.FirstOrDefault(k => k.Hold == 2);
        var released = open is null ? null : Transforms.Multiply(JawWorld(open.Q), offsetM);
        return new StrutTrack { Rest = restM, HeldOffset = offset, Released = released };
    }

    /// <summary>Motus world strut pose for one sample.</summary>
    public double[] StrutWorld(StrutTrack track, int hold, IReadOnlyList<double> qDeg)
    {
        if (hold == 1 && track.HeldOffset is { } off)
            return Transforms.Multiply(JawWorld(qDeg), Transforms.FromFrame(off));
        if (hold == 2 && track.Released is { } rel) return rel;
        return track.Rest;
    }

    /// <summary>Motus world matrix → viewer position + quaternion (x, y, z, qx, qy, qz, qw) for viewer.js.</summary>
    public static double[] ToViewerPose(double[] motusWorld)
    {
        var v = Transforms.Multiply(Transforms.FromAxisAngle(1, 0, 0, -Math.PI / 2), motusWorld);
        var f = Transforms.ToFrame(v);
        return new[] { f.X, f.Y, f.Z, f.Qx, f.Qy, f.Qz, f.Qw };
    }

    /// <summary>Fixed mount parts (base tube, L-grip body/rail/jaws at <paramref name="gLmm"/>) as world scene boxes.</summary>
    public IEnumerable<CollisionObject> MountObjects(double gLmm)
    {
        foreach (var g in _mountGeometry)
        {
            var local = g.LocalGeometry;
            var pose = local.Pose;
            var li = Array.IndexOf(LGripPlates, local.Name);
            if (li >= 0) pose = new Frame((li == 0 ? -1 : 1) * PlateShift(gLmm), pose.Y, pose.Z, pose.Qw, pose.Qx, pose.Qy, pose.Qz);
            var world = Transforms.ToFrame(Transforms.Multiply(MountWorld, Transforms.FromFrame(pose)));
            yield return CollisionObject.Box(local.Name, world, local.ExtentX, local.ExtentY, local.ExtentZ);
        }
    }

    /// <summary>
    /// Scene for one motion sample: pole, ground slab, mount parts, and the strut unless it is held
    /// (then it rides on the checker as an <see cref="AttachedBody"/>). <paramref name="gripping"/> allows only
    /// the two R-grip jaw plates to touch the strut (the clamp contact); neck, rail, carriage and links still collide.
    /// </summary>
    public CollisionScene Scene(StrutTrack track, int hold, bool gripping, double gLmm, IReadOnlyList<double> qDeg,
        IReadOnlyList<(string Name, StrutPlacement Pose)>? extras = null)
    {
        // Scenes only depend on (track, hold, gripping, gL, extras): the strut is static unless held, and held = attached.
        var extraKey = extras is null || extras.Count == 0
            ? 0
            : extras.Aggregate(0, (acc, e) => HashCode.Combine(acc, e.Name,
                Math.Round(e.Pose.X, 5), Math.Round(e.Pose.Y, 5), Math.Round(e.Pose.Z, 5), Math.Round(e.Pose.PitchDeg, 3)));
        var key = (track, hold, gripping, (int)Math.Round(gLmm * 10), extraKey);
        if (_scenes.TryGetValue(key, out var cached)) return cached;
        if (_scenes.Count > 64) _scenes.Clear();
        return _scenes[key] = BuildScene(track, hold, gripping, gLmm, qDeg, extras);
    }

    private readonly Dictionary<(StrutTrack, int, bool, int, int), CollisionScene> _scenes = new();

    private CollisionScene BuildScene(StrutTrack track, int hold, bool gripping, double gLmm, IReadOnlyList<double> qDeg,
        IReadOnlyList<(string Name, StrutPlacement Pose)>? extras)
    {
        var objects = new List<CollisionObject> { _poleObj, _groundObj };
        objects.AddRange(MountObjects(gLmm));
        var held = hold == 1 && track.Captured;
        if (!held) objects.Add(StrutAt(StrutWorld(track, hold, qDeg)));
        if (extras is not null)
            foreach (var (name, pose) in extras)
                objects.Add(StrutAt(PlacementWorld(pose), name));

        var allowed = new List<(string, string)>
        {
            // θ1 housing is journaled in the base tube (the θ1 joint itself).
            ("θ1", "base tube"),
        };
        if (gripping)
            foreach (var plate in RGripPlates) allowed.Add((plate, Strut));
        return new CollisionScene(objects, allowed);
    }

    /// <summary>Checker for an R-grip opening (jaw plates moved to match the drawing) and optional held strut.</summary>
    public RobotMeshCollisionChecker Checker(double gRmm, Frame? heldOffset)
    {
        var offKey = heldOffset is { } o
            ? HashCode.Combine(Math.Round(o.X, 6), Math.Round(o.Y, 6), Math.Round(o.Z, 6), Math.Round(o.Qw, 6), Math.Round(o.Qx, 6), Math.Round(o.Qy, 6), Math.Round(o.Qz, 6))
            : 0;
        var key = ((int)Math.Round(gRmm * 10), heldOffset is not null, offKey);
        if (_checkers.TryGetValue(key, out var c)) return c;
        var baseModel = Model.CollisionModel!;
        var links = baseModel.Links.Select(l =>
        {
            var pi = Array.IndexOf(RGripPlates, l.LocalGeometry.Name);
            if (pi < 0) return l;
            var g = l.LocalGeometry;
            var p = g.Pose;
            var moved = CollisionObject.Box(g.Name,
                new Frame((pi == 0 ? -1 : 1) * PlateShift(key.Item1 / 10.0), p.Y, p.Z, p.Qw, p.Qx, p.Qy, p.Qz),
                g.ExtentX, g.ExtentY, g.ExtentZ);
            return new LinkCollisionGeometry(l.LinkIndex, l.LinkName, moved);
        }).ToList();
        var model = new RobotModel(Model.Preset, new RobotCollisionModel(links), Urdf.JointNames);
        var attached = heldOffset is { } off ? new[] { HeldStrut(off) } : Array.Empty<AttachedBody>();
        c = new RobotMeshCollisionChecker(model, Chain, attached, SelfCollisionMinLinkGap);
        if (_checkers.Count > 512) _checkers.Clear();
        _checkers[key] = c;
        return c;
    }

    /// <summary>Gripping = R-grip closed on a captured strut, or closing/opening around it.</summary>
    public static bool IsGripping(StrutTrack track, int hold, string label) =>
        track.Captured && (hold == 1 || label is "Close" or "Open");

    /// <summary>All Motus contacts for one pose. Empty = clear.</summary>
    public IReadOnlyList<CollisionContact> Contacts(
        IReadOnlyList<double> qDeg, double gRmm, double gLmm, int hold, string label, StrutTrack track,
        IReadOnlyList<(string Name, StrutPlacement Pose)>? extras = null, int maxContacts = 32)
    {
        var scene = Scene(track, hold, IsGripping(track, hold, label), gLmm, qDeg, extras);
        var checker = Checker(gRmm, hold == 1 ? track.HeldOffset : null);
        return checker.FindContacts(new JointState(RadFromDeg(qDeg)), scene, maxContacts);
    }

    public bool IsClear(IReadOnlyList<double> qDeg, double gRmm, double gLmm, int hold, string label, StrutTrack track,
        IReadOnlyList<(string Name, StrutPlacement Pose)>? extras = null)
    {
        var scene = Scene(track, hold, IsGripping(track, hold, label), gLmm, qDeg, extras);
        return Checker(gRmm, hold == 1 ? track.HeldOffset : null).IsCollisionFree(new JointState(RadFromDeg(qDeg)), scene);
    }

    /// <summary>Motus FK: jaw (TCP) origin in viewer coordinates.</summary>
    public (double X, double Y, double Z) JawViewer(IReadOnlyList<double> qDeg)
    {
        var tcp = Fk.ComputeTcp(new JointState(RadFromDeg(qDeg)), Model.Preset.BaseFrame, Model.Preset.ToolFrame).Tcp;
        return (tcp.X, tcp.Z, -tcp.Y);
    }

    /// <summary>Where the pick object lies: the first pick task's grasp plane at its un-nudged origin.</summary>
    public static StrutPlacement StrutRest(IReadOnlyList<BambooIcdMotion.TaskEdit> tasks)
    {
        var p = tasks.FirstOrDefault(t => t.Identity == "pick")?.Planes.FirstOrDefault(x => x.Name == "grasp");
        return p is null
            ? new StrutPlacement(BambooIcdMotion.Pick.X, BambooIcdMotion.Pick.Y, BambooIcdMotion.Pick.Z, 0)
            : new StrutPlacement(p.X, p.Y0, p.Z, p.PitchDeg);
    }

    /// <summary>JSON-friendly cell description for viewer.js (parts + environment), so drawing == collision.</summary>
    public object CellDescription() => new
    {
        seg = Seg,
        poleY = BambooIcdMotion.PoleY,
        pole = new { radius = PoleRadius, length = PoleLength, segments = PoleSegments },
        strut = new { radius = StrutRadius, length = StrutLength, segments = StrutSegments },
        ground = new { size = GroundSize },
        parts = Parts.Select(p => new { name = p.Name, link = p.Link, size = p.Size, pos = p.Pos })
    };

    public static string Describe(IEnumerable<CollisionContact> contacts) =>
        string.Join(", ", contacts.Select(c => $"{c.BodyA} ↔ {c.BodyB}").Distinct());
}
