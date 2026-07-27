using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// One leg recipe: hip mount in body, serial chain / 3R lengths, foot tip, IK solver.
/// <see cref="HipInBody"/> null = unset (do not treat <see cref="Frame.Identity"/> as unset).
/// </summary>
public sealed class LegDefinition
{
    public LegDefinition(
        string name,
        Frame? hipInBody,
        ILegIkSolver ik,
        string footLink,
        IReadOnlyList<double>? lengths3R = null,
        KinematicTree? chain = null)
    {
        Name = name;
        HipInBody = hipInBody;
        Ik = ik;
        FootLink = footLink;
        Lengths3R = lengths3R;
        Chain = chain;
    }

    public string Name { get; }
    public Frame? HipInBody { get; }
    public ILegIkSolver Ik { get; }
    public string FootLink { get; }
    /// <summary>Optional coxa/femur/tibia (m) when the leg is homogeneous 3R.</summary>
    public IReadOnlyList<double>? Lengths3R { get; }
    public KinematicTree? Chain { get; }

    public int DriverDof =>
        Chain?.DriverCount
        ?? (Ik is LegIk3RSolver ? 3 : throw new InvalidOperationException(
            $"Leg '{Name}' needs Chain or LegIk3RSolver to know driver DOF."));

    public string? Validate()
    {
        if (string.IsNullOrWhiteSpace(Name))
            return "LegDefinition.Name is required.";
        if (string.IsNullOrWhiteSpace(FootLink))
            return $"Leg '{Name}': FootLink is required.";
        if (Ik is null)
            return $"Leg '{Name}': Ik solver is required.";
        if (HipInBody is { } hip &&
            (!double.IsFinite(hip.X) || !double.IsFinite(hip.Y) || !double.IsFinite(hip.Z)))
            return $"Leg '{Name}': HipInBody is not finite.";
        if (Lengths3R is not null)
        {
            if (Lengths3R.Count != 3)
                return $"Leg '{Name}': Lengths3R must be [coxa,femur,tibia].";
            for (var i = 0; i < 3; i++)
                if (!(Lengths3R[i] > 0) || !double.IsFinite(Lengths3R[i]))
                    return $"Leg '{Name}': Lengths3R[{i}] must be finite and > 0 (m).";
        }

        return null;
    }
}

/// <summary>
/// Assembled N-leg walker: legs + gait + tip + clearance. Motus.NET owns math; GH is thin wiring.
/// </summary>
public sealed class LeggedMechanism
{
    public LeggedMechanism(
        IReadOnlyList<LegDefinition> legs,
        GaitSchedule gait,
        string tipLegName,
        double nominalBodyClearance,
        bool allowDynamicGait = false,
        string bodyLinkName = "body",
        string modelName = "legged")
    {
        Legs = legs;
        Gait = gait;
        TipLegName = tipLegName;
        NominalBodyClearance = nominalBodyClearance;
        AllowDynamicGait = allowDynamicGait;
        BodyLinkName = bodyLinkName;
        ModelName = modelName;

        var offsets = new int[legs.Count];
        var total = 0;
        for (var i = 0; i < legs.Count; i++)
        {
            offsets[i] = total;
            total += legs[i].DriverDof;
        }

        DriverOffsets = offsets;
        DriverCount = total;
    }

    public IReadOnlyList<LegDefinition> Legs { get; }
    public GaitSchedule Gait { get; }
    public string TipLegName { get; }
    /// <summary>Design body/hip clearance above support (m). Hip Z lives in <see cref="LegDefinition.HipInBody"/>.</summary>
    public double NominalBodyClearance { get; }
    public bool AllowDynamicGait { get; }
    public string BodyLinkName { get; }
    public string ModelName { get; }
    public IReadOnlyList<int> DriverOffsets { get; }
    public int DriverCount { get; }
    public int LegCount => Legs.Count;

    public string TipLinkName
    {
        get
        {
            foreach (var leg in Legs)
            {
                if (string.Equals(leg.Name, TipLegName, StringComparison.Ordinal))
                    return NamespacedLink(leg.Name, leg.FootLink);
            }

            return NamespacedLink(TipLegName, "tibia");
        }
    }

    public string? Validate()
    {
        if (Legs.Count < 2)
            return "LeggedMechanism needs ≥ 2 legs.";
        if (!double.IsFinite(NominalBodyClearance) || NominalBodyClearance < 0)
            return "NominalBodyClearance must be finite and ≥ 0 (m).";
        if (string.IsNullOrWhiteSpace(TipLegName))
            return "TipLegName is required.";
        if (string.IsNullOrWhiteSpace(BodyLinkName))
            return "BodyLinkName is required.";

        var tipOk = false;
        var names = new HashSet<string>(StringComparer.Ordinal);
        for (var i = 0; i < Legs.Count; i++)
        {
            var err = Legs[i].Validate();
            if (err is not null) return err;
            if (!names.Add(Legs[i].Name))
                return $"Duplicate leg name '{Legs[i].Name}'.";
            if (string.Equals(Legs[i].Name, TipLegName, StringComparison.Ordinal))
                tipOk = true;
            if (Legs[i].HipInBody is null)
                return $"Leg '{Legs[i].Name}': HipInBody is required (null = unset).";
        }

        if (!tipOk)
            return $"TipLegName '{TipLegName}' not found in Legs.";

        var gaitErr = Gait.Validate(Legs.Count, AllowDynamicGait);
        if (gaitErr is not null) return gaitErr;

        return null;
    }

    /// <summary>
    /// Assemble tree + per-leg FK→IK→FK residual probe at nominal stance (NASA gate before Walk).
    /// </summary>
    public string? ValidateAndCalibrate(
        double hipStanceRad = 7.5 * Math.PI / 180.0,
        double femurStanceRad = 30.0 * Math.PI / 180.0,
        double tibiaStanceRad = -30.0 * Math.PI / 180.0,
        double positionTolMeters = 1e-3)
    {
        var err = Validate();
        if (err is not null) return err;

        try
        {
            _ = Assemble();
        }
        catch (Exception ex)
        {
            return $"Assemble failed: {ex.Message}";
        }

        for (var leg = 0; leg < Legs.Count; leg++)
        {
            var def = Legs[leg];
            var hip = HipBody(leg);
            var yaw = HipYawRad(leg);
            var side = LegIsLeft(leg) ? 1.0 : -1.0;
            if (!def.Ik.TryNominalStance(
                    hip, yaw, side, hipStanceRad, femurStanceRad, tibiaStanceRad,
                    NominalBodyClearance, out var q, out var foot, out var code))
                return $"Leg '{def.Name}' nominal stance failed: {code}.";

            if (!def.Ik.TrySolve(hip, foot, FootTargetKind.Position, out var q2, out code))
                return $"Leg '{def.Name}' FK→IK calibrate failed: {code}.";

            if (def.Ik is LegIk3RSolver s3)
            {
                var back = s3.FootPosition(hip, q2[0], q2[1], q2[2]);
                var dx = back.X - foot.X;
                var dy = back.Y - foot.Y;
                var dz = back.Z - foot.Z;
                var res = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                if (res > positionTolMeters)
                    return $"Leg '{def.Name}' FK↔IK residual {res:F4} m > {positionTolMeters:F4} m.";
            }

            _ = q;
        }

        return null;
    }

    public RobotPreset ToPreset(string? modelName = null, IReadOnlyList<JointLimit>? limits = null)
    {
        var lim = limits ?? Enumerable.Range(0, DriverCount)
            .Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        return new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = modelName ?? ModelName,
            Family = Units.LeggedFamily,
            AxisCount = DriverCount,
            JointLimits = lim.ToList(),
            BaseFrame = BaseFrame.Identity,
            ToolFrame = ToolFrame.Identity,
        };
    }

    /// <summary>
    /// Body link + namespaced <see cref="KinematicTree.Attach"/> of each leg (<c>legName/</c> prefix).
    /// </summary>
    public KinematicTree Assemble(string? treeName = null)
    {
        var err = Validate();
        if (err is not null)
            throw new InvalidOperationException(err);

        var body = new KinematicTree(
            treeName ?? ModelName,
            [new KinematicLink(BodyLinkName)],
            Array.Empty<KinematicJoint>(),
            rootLinkIndex: 0,
            driverJointIndices: Array.Empty<int>());

        var tree = body;
        foreach (var leg in Legs)
        {
            var hip = leg.HipInBody ?? Frame.Identity;
            var legTree = leg.Chain is not null
                ? WithNamespace(leg.Chain, leg.Name + "/")
                : BuildNamespaced3R(leg);
            tree = tree.Attach(BodyLinkName, legTree, legTree.Links[legTree.RootLinkIndex].Name, hip);
        }

        return tree;
    }

    /// <summary>Homogeneous N×3R radial insectoid.</summary>
    public static LeggedMechanism FromHomogeneous3RRadial(
        int legCount,
        double bodyR,
        double coxa,
        double femur,
        double tibia,
        double bodyZ,
        IReadOnlyList<string>? names = null,
        IReadOnlyList<double>? hipYawsRad = null,
        GaitSchedule? gait = null,
        string? tipLegName = null,
        bool allowDynamicGait = false,
        string modelName = "legged")
    {
        if (legCount < 2)
            throw new ArgumentException("Need ≥ 2 legs.", nameof(legCount));
        if (!(bodyR > 0) || !(coxa > 0) || !(femur > 0) || !(tibia > 0) || !(bodyZ > 0))
            throw new ArgumentException("BodyR/Coxa/Femur/Tibia/BodyZ must be > 0 (m).");

        var legNames = names ?? Enumerable.Range(0, legCount).Select(i => $"leg{i}").ToArray();
        if (legNames.Count != legCount)
            throw new ArgumentException("names length must equal legCount.", nameof(names));

        var yaws = new double[legCount];
        if (hipYawsRad is not null)
        {
            if (hipYawsRad.Count != legCount)
                throw new ArgumentException("hipYawsRad length must equal legCount.", nameof(hipYawsRad));
            for (var i = 0; i < legCount; i++)
                yaws[i] = hipYawsRad[i];
        }
        else
        {
            for (var i = 0; i < legCount; i++)
                yaws[i] = i * (2.0 * Math.PI / legCount);
        }

        gait ??= GaitSchedule.Auto(legCount, yaws);
        var tip = tipLegName ?? legNames[0];
        var ik = new LegIk3RSolver(coxa, femur, tibia);
        var legs = new LegDefinition[legCount];
        for (var i = 0; i < legCount; i++)
        {
            var hip = new Frame(bodyR * Math.Cos(yaws[i]), bodyR * Math.Sin(yaws[i]), bodyZ);
            legs[i] = new LegDefinition(
                legNames[i], hip, ik, footLink: "tibia",
                lengths3R: [coxa, femur, tibia]);
        }

        return new LeggedMechanism(
            legs, gait, tip, nominalBodyClearance: bodyZ,
            allowDynamicGait, modelName: modelName);
    }

    public static LeggedMechanism HexMithi(
        double bodyR, double coxa, double femur, double tibia, double bodyZ) =>
        FromHomogeneous3RRadial(
            6, bodyR, coxa, femur, tibia, bodyZ,
            names:
            [
                "right-middle", "right-front", "left-front",
                "left-middle", "left-back", "right-back",
            ],
            hipYawsRad: Enumerable.Range(0, 6).Select(i => i * (Math.PI / 3.0)).ToArray(),
            gait: GaitSchedule.AlternatingTripod(),
            tipLegName: "right-middle",
            modelName: "hex_mithi");

    /// <summary>Quad trot fixture — requires <see cref="AllowDynamicGait"/> (MinStanceCount=2).</summary>
    public static LeggedMechanism QuadSmoke(
        double bodyR, double coxa, double femur, double tibia, double bodyZ) =>
        FromHomogeneous3RRadial(
            4, bodyR, coxa, femur, tibia, bodyZ,
            names: ["front-right", "front-left", "rear-left", "rear-right"],
            hipYawsRad: Enumerable.Range(0, 4)
                .Select(i => i * (Math.PI / 2.0) + Math.PI / 4.0).ToArray(),
            gait: GaitSchedule.FromGroups([[0, 2], [1, 3]], "QuadSmokeTrot"),
            tipLegName: "front-right",
            allowDynamicGait: true,
            modelName: "quad_smoke");

    public Vec3 HipBody(int legIndex)
    {
        var hip = Legs[legIndex].HipInBody
                  ?? throw new InvalidOperationException($"Leg {legIndex} HipInBody unset.");
        return new Vec3(hip.X, hip.Y, hip.Z);
    }

    public double HipYawRad(int legIndex)
    {
        var h = HipBody(legIndex);
        return Math.Atan2(h.Y, h.X);
    }

    public bool LegIsLeft(int legIndex) =>
        Legs[legIndex].Name.Contains("left", StringComparison.OrdinalIgnoreCase);

    public static string NamespacedLink(string legName, string localLink) =>
        localLink.StartsWith(legName + "/", StringComparison.Ordinal)
            ? localLink
            : legName + "/" + localLink;

    private static KinematicTree BuildNamespaced3R(LegDefinition leg)
    {
        if (leg.Lengths3R is null || leg.Lengths3R.Count != 3)
            throw new InvalidOperationException($"Leg '{leg.Name}' needs Lengths3R or Chain for Assemble.");

        var coxa = leg.Lengths3R[0];
        var femur = leg.Lengths3R[1];
        var prefix = leg.Name + "/";
        // mount →(hip Z)→ coxa →(femur Y)→ femur →(tibia Y)→ tibia; Attach(fixed) grafts mount onto body.
        var links = new KinematicLink[]
        {
            new(prefix + "mount"),
            new(prefix + "coxa"),
            new(prefix + "femur"),
            new(prefix + "tibia"),
        };
        var joints = new KinematicJoint[]
        {
            new(prefix + "hip", KinematicJointType.Revolute, 0, 1,
                0, 0, 0, 0, 0, 0, 0, 0, 1, -Math.PI, Math.PI, Math.PI, 0, null),
            new(prefix + "femur", KinematicJointType.Revolute, 1, 2,
                coxa, 0, 0, 0, 0, 0, 0, 1, 0, -Math.PI, Math.PI, Math.PI, 1, null),
            new(prefix + "tibia", KinematicJointType.Revolute, 2, 3,
                femur, 0, 0, 0, 0, 0, 0, 1, 0, -Math.PI, Math.PI, Math.PI, 2, null),
        };
        return new KinematicTree(leg.Name, links, joints, rootLinkIndex: 0, driverJointIndices: [0, 1, 2]);
    }

    /// <summary>Prefix all link/joint names (for Attach without clashes).</summary>
    public static KinematicTree WithNamespace(KinematicTree tree, string prefix)
    {
        if (string.IsNullOrEmpty(prefix))
            return tree;
        var links = tree.Links.Select(l => new KinematicLink(prefix + l.Name, l.MeshName, l.MeshPath)).ToArray();
        var joints = new KinematicJoint[tree.Joints.Count];
        for (var i = 0; i < tree.Joints.Count; i++)
        {
            var j = tree.Joints[i];
            joints[i] = new KinematicJoint(
                prefix + j.Name, j.Type, j.ParentLinkIndex, j.ChildLinkIndex,
                j.OriginX, j.OriginY, j.OriginZ, j.Roll, j.Pitch, j.Yaw,
                j.AxisX, j.AxisY, j.AxisZ, j.Lower, j.Upper, j.Velocity,
                j.DriverIndex, j.Mimic);
        }

        return new KinematicTree(
            prefix + tree.Name, links, joints, tree.RootLinkIndex, tree.DriverJointIndices.ToArray());
    }
}
