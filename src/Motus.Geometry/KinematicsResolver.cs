using Motus.Core;

namespace Motus.Geometry;

public static class KinematicsResolver
{
    public static IFkSolver CreateFkSolver(RobotPreset preset, SerialJointChain? serialChain = null)
    {
        if (serialChain is not null)
            return new SerialForwardKinematics(serialChain);
        if (KinematicsProfiles.TryGet(preset, out var dh))
            return new DhForwardKinematics(dh);
        throw new InvalidOperationException($"No kinematics for model '{preset.ModelName}'. Load URDF or add a DH profile.");
    }

    public static IForwardKinematics CreateForwardKinematics(RobotPreset preset, SerialJointChain? serialChain = null) =>
        CreateFkSolver(preset, serialChain);

    public static IInverseKinematics CreateInverseKinematics(RobotPreset preset, SerialJointChain? serialChain = null)
    {
        if (serialChain is not null && KinematicsProfiles.IsUniversalRobots(preset))
            return new UrdfUniversalRobotsKinematics(preset, serialChain);
        if (serialChain is not null)
            return new NumericalInverseKinematics(CreateFkSolver(preset, serialChain), preset);
        if (KinematicsProfiles.IsUniversalRobots(preset))
            return new UrInverseKinematics(preset);
        return new NumericalInverseKinematics(CreateFkSolver(preset), preset);
    }

    public static bool SupportsModel(RobotPreset preset, SerialJointChain? serialChain = null) =>
        serialChain is not null || KinematicsProfiles.TryGet(preset, out _);
}
