using Motus.Core;

namespace Motus.Geometry;

public static class KinematicsProfiles
{
    private static readonly Dictionary<string, KinematicsChain> ByModel = new(StringComparer.OrdinalIgnoreCase)
    {
        ["UR3e"] = Ur(0.15185, 0.24355, 0.2132, 0.13105, 0.08535, 0.0921),
        ["UR5e"] = Ur(0.089159, 0.425, 0.39225, 0.10915, 0.09465, 0.0823),
        ["UR10e"] = Ur(0.1807, 0.6127, 0.57155, 0.17415, 0.11985, 0.11655),
        ["UR16e"] = Ur(0.1807, 0.7782, 0.6405, 0.17415, 0.11985, 0.11655),
        ["UR20"] = Ur(0.2363, 0.8618, 0.727, 0.201, 0.1593, 0.1548),
        ["UR30"] = Ur(0.2363, 1.116, 0.940, 0.201, 0.1593, 0.1548),
        ["KR 6 R900"] = Kuka6(0.25, 0.455, 0.42, 0.12, 0.12),
        ["KR 10 R1100"] = Kuka6(0.28, 0.56, 0.515, 0.14, 0.14),
        ["KR 16 R2010"] = Kuka6(0.32, 0.675, 0.62, 0.16, 0.16),
        ["KR 22 R1610"] = Kuka6(0.30, 0.59, 0.54, 0.15, 0.15),
        ["KR 60 R2100"] = Kuka6(0.35, 0.78, 0.72, 0.18, 0.18),
        ["KR 120 R2700"] = Kuka6(0.42, 1.03, 0.95, 0.22, 0.22),
        ["LBR iiwa 7 R800"] = Kuka7(0.34, 0.36, 0.42, 0.4, 0.39, 0.126),
        ["LBR iiwa 14 R820"] = Kuka7(0.36, 0.42, 0.48, 0.42, 0.41, 0.13),
    };

    public static bool TryGet(RobotPreset preset, out KinematicsChain chain) =>
        ByModel.TryGetValue(preset.ModelName, out chain!);

    public static KinematicsChain GetRequired(RobotPreset preset) =>
        TryGet(preset, out var chain)
            ? chain
            : throw new InvalidOperationException($"No kinematics profile for model '{preset.ModelName}'.");

    // ponytail: one UR DH template, link lengths scaled per model row above
    private static KinematicsChain Ur(double d1, double a2, double a3, double d4, double d5, double d6)
    {
        var halfPi = Math.PI / 2;
        var links = new[]
        {
            new DhLink(0, d1, halfPi),
            new DhLink(-a2, 0, 0),
            new DhLink(-a3, 0, 0),
            new DhLink(0, d4, halfPi),
            new DhLink(0, d5, -halfPi),
            new DhLink(0, d6, 0),
        };
        var radii = new[] { 0.08, 0.09, 0.08, 0.07, 0.06, 0.05 };
        return new KinematicsChain(links, radii);
    }

    private static KinematicsChain Kuka6(double d1, double a2, double a3, double d4, double d6)
    {
        var halfPi = Math.PI / 2;
        var links = new[]
        {
            new DhLink(0, d1, halfPi),
            new DhLink(a2, 0, 0),
            new DhLink(a3, 0, halfPi),
            new DhLink(0, d4, -halfPi),
            new DhLink(0, 0, halfPi),
            new DhLink(0, d6, 0),
        };
        return new KinematicsChain(links, [0.09, 0.10, 0.09, 0.08, 0.07, 0.06]);
    }

    private static KinematicsChain Kuka7(double d1, double a2, double a3, double a4, double d5, double d6)
    {
        var halfPi = Math.PI / 2;
        var links = new[]
        {
            new DhLink(0, d1, halfPi),
            new DhLink(a2, 0, 0),
            new DhLink(a3, 0, halfPi),
            new DhLink(-a4, 0, -halfPi),
            new DhLink(0, d5, halfPi),
            new DhLink(0, 0, -halfPi),
            new DhLink(0, d6, 0),
        };
        return new KinematicsChain(links, [0.08, 0.08, 0.08, 0.07, 0.07, 0.06, 0.05]);
    }
}
