namespace Motus.Core;

/// <summary>
/// Structured kinematics outcome for Stewart (and other mechanism families that need
/// Status richer than <see cref="IInverseKinematics.TrySolve"/>'s bool).
/// </summary>
public enum KinematicsReason
{
    Ok = 0,
    InvalidInput = 1,
    StrokeLimit = 2,
    Singular = 3,
    FkDiverge = 4,
    DeltaLengthJump = 5,
    Workspace = 6
}

public readonly struct KinematicsSolveResult
{
    public bool Success => Reason == KinematicsReason.Ok;
    public KinematicsReason Reason { get; }
    public string Detail { get; }
    public JointState? JointState { get; }
    public CartesianPose? Pose { get; }

    private KinematicsSolveResult(
        KinematicsReason reason,
        string detail,
        JointState? jointState,
        CartesianPose? pose)
    {
        Reason = reason;
        Detail = detail;
        JointState = jointState;
        Pose = pose;
    }

    public static KinematicsSolveResult OkJoints(JointState joints, string detail = "") =>
        new(KinematicsReason.Ok, detail, joints, null);

    public static KinematicsSolveResult OkPose(CartesianPose pose, string detail = "") =>
        new(KinematicsReason.Ok, detail, null, pose);

    public static KinematicsSolveResult Fail(KinematicsReason reason, string detail) =>
        new(reason, detail, null, null);

    public override string ToString() =>
        Success ? (string.IsNullOrEmpty(Detail) ? "Ok" : $"Ok: {Detail}") : $"{Reason}: {Detail}";
}
