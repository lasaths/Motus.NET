using Motus.Core;

namespace Motus.Geometry;

public static class KinematicsResolver
{
    public static IFkSolver CreateFkSolver(RobotPreset preset, SerialJointChain? serialChain = null)
    {
        if (Units.IsStewart(preset))
            throw new InvalidOperationException(
                "Stewart platforms use StewartForwardKinematics, not serial IFkSolver. Pass StewartPlatform.");
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
        if (Units.IsStewart(preset))
            throw new InvalidOperationException(
                "Stewart platforms use StewartInverseKinematics(StewartPlatform), not serial IK factories.");
        // Analytic UR IK is 6R only — N-DOF / rail / serial trees use numerical IK.
        if (serialChain is not null
            && KinematicsProfiles.IsUniversalRobots(preset)
            && preset.AxisCount == 6
            && serialChain.Joints.Length == 6)
            return new UrdfUniversalRobotsKinematics(preset, serialChain);
        if (serialChain is not null)
            return new NumericalInverseKinematics(
                CreateFkSolver(preset, serialChain), preset, serialChain);
        if (KinematicsProfiles.IsUniversalRobots(preset) && preset.AxisCount == 6)
            return new UrInverseKinematics(preset);
        return new NumericalInverseKinematics(CreateFkSolver(preset), preset);
    }

    public static bool SupportsModel(RobotPreset preset, SerialJointChain? serialChain = null) =>
        Units.IsStewart(preset) || serialChain is not null || KinematicsProfiles.TryGet(preset, out _);
}
