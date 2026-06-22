using Motus.Core;

namespace Motus.Geometry;

public static class KinematicsResolver
{
    public static IForwardKinematics CreateForwardKinematics(RobotPreset preset) =>
        new DhForwardKinematics(preset);

    public static IInverseKinematics CreateInverseKinematics(RobotPreset preset) =>
        preset.Manufacturer == RobotManufacturer.UniversalRobots
            ? new UrInverseKinematics(preset)
            : new NumericalInverseKinematics(preset);

    public static bool SupportsModel(RobotPreset preset) =>
        KinematicsProfiles.TryGet(preset, out _);
}
