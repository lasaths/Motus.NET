using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// N×3R insectoid layout: hip mount yaws, swing-group partition, tip-path leg.
/// Flat-ground preview gait — Motus.NET owns math; GH is thin wiring.
/// </summary>
public sealed class LeggedLayout
{
    public IReadOnlyList<string> LegNames { get; }
    public IReadOnlyList<double> HipYawsRad { get; }
    public IReadOnlyList<int[]> SwingGroups { get; }
    public double BodyR { get; }
    public double Coxa { get; }
    public double Femur { get; }
    public double Tibia { get; }
    public double BodyZ { get; }
    public string TipLegName { get; }

    public LeggedLayout(
        IReadOnlyList<string> legNames,
        IReadOnlyList<double> hipYawsRad,
        IReadOnlyList<int[]> swingGroups,
        double bodyR,
        double coxa,
        double femur,
        double tibia,
        double bodyZ,
        string tipLegName)
    {
        LegNames = legNames;
        HipYawsRad = hipYawsRad;
        SwingGroups = swingGroups;
        BodyR = bodyR;
        Coxa = coxa;
        Femur = femur;
        Tibia = tibia;
        BodyZ = bodyZ;
        TipLegName = tipLegName;
    }

    public int LegCount => LegNames.Count;
    public int DriverCount => LegCount * 3;
    public string TipLinkName => $"{TipLegName}_tibia";

    /// <summary>Mithi-style hex: 6 legs @ π/3, tripod groups RF/LM/LB then RM/LF/RB.</summary>
    public static LeggedLayout HexMithi(
        double bodyR, double coxa, double femur, double tibia, double bodyZ)
    {
        string[] names =
        [
            "right-middle", "right-front", "left-front",
            "left-middle", "left-back", "right-back",
        ];
        var yaws = new double[6];
        for (var i = 0; i < 6; i++)
            yaws[i] = i * (Math.PI / 3.0);

        return new LeggedLayout(
            names, yaws, [[1, 3, 4], [0, 2, 5]],
            bodyR, coxa, femur, tibia, bodyZ, tipLegName: "right-middle");
    }

    /// <summary>Smoke / reuse fixture: 4 legs @ π/2, alternating biped groups.</summary>
    public static LeggedLayout QuadSmoke(
        double bodyR, double coxa, double femur, double tibia, double bodyZ)
    {
        string[] names = ["front-right", "front-left", "rear-left", "rear-right"];
        var yaws = new double[4];
        for (var i = 0; i < 4; i++)
            yaws[i] = i * (Math.PI / 2.0) + Math.PI / 4.0;

        return new LeggedLayout(
            names, yaws, [[0, 2], [1, 3]],
            bodyR, coxa, femur, tibia, bodyZ, tipLegName: "front-right");
    }

    public string? Validate()
    {
        if (LegCount < 2)
            return "LeggedLayout needs ≥ 2 legs.";
        if (HipYawsRad.Count != LegCount)
            return $"HipYawsRad length ({HipYawsRad.Count}) must equal LegCount ({LegCount}).";
        if (SwingGroups is null || SwingGroups.Count < 1)
            return "SwingGroups must contain ≥ 1 group.";
        if (BodyR <= 0 || Coxa <= 0 || Femur <= 0 || Tibia <= 0 || BodyZ <= 0)
            return "BodyR / Coxa / Femur / Tibia / BodyZ must be > 0 (m).";
        if (string.IsNullOrWhiteSpace(TipLegName))
            return "TipLegName is required.";

        var seen = new bool[LegCount];
        var covered = 0;
        for (var g = 0; g < SwingGroups.Count; g++)
        {
            var group = SwingGroups[g];
            if (group is null || group.Length == 0)
                return $"SwingGroups[{g}] is empty.";
            foreach (var leg in group)
            {
                if (leg < 0 || leg >= LegCount)
                    return $"SwingGroups[{g}] contains out-of-range leg index {leg}.";
                if (seen[leg])
                    return $"Leg {leg} appears in more than one swing group.";
                seen[leg] = true;
                covered++;
            }
        }

        if (covered != LegCount)
            return $"SwingGroups must partition all {LegCount} legs (covered {covered}).";

        var tipOk = false;
        for (var i = 0; i < LegCount; i++)
        {
            if (string.IsNullOrWhiteSpace(LegNames[i]))
                return $"LegNames[{i}] is empty.";
            if (string.Equals(LegNames[i], TipLegName, StringComparison.Ordinal))
                tipOk = true;
            if (!double.IsFinite(HipYawsRad[i]))
                return $"HipYawsRad[{i}] is not finite.";
        }

        return tipOk ? null : $"TipLegName '{TipLegName}' not found in LegNames.";
    }

    public bool LegIsLeft(int legIndex) =>
        LegNames[legIndex].Contains("left", StringComparison.OrdinalIgnoreCase);

    public RobotPreset ToPreset(string modelName, int axisCount, IReadOnlyList<JointLimit> limits) =>
        new()
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = modelName,
            Family = Units.LeggedFamily,
            AxisCount = axisCount,
            JointLimits = limits.ToList(),
            BaseFrame = BaseFrame.Identity,
            ToolFrame = ToolFrame.Identity,
        };
}
