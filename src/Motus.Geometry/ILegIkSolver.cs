namespace Motus.Geometry;

/// <summary>What the foot target specifies for <see cref="ILegIkSolver.TrySolve"/>.</summary>
public enum FootTargetKind
{
    /// <summary>Position only (meters); orientation ignored.</summary>
    Position = 0,
    /// <summary>Full pose — solvers that cannot meet orientation may fail or degrade to position.</summary>
    Pose = 1,
}

/// <summary>Typed IK failure — no silent NaN poses.</summary>
public enum LegIkFailureCode
{
    None = 0,
    NonFiniteInput,
    InvalidGeometry,
    Unreachable,
    Degenerate,
    UnsupportedTargetKind,
}

/// <summary>Reach annulus / limits for a leg IK solver (meters / radians as documented per solver).</summary>
public readonly record struct LegIkWorkspace(
    double MinReachMeters,
    double MaxReachMeters,
    string Description);

/// <summary>
/// Per-leg IK: solve + nominal stance + workspace. Gait must not leak coxa/femur lengths past this boundary.
/// </summary>
public interface ILegIkSolver
{
    LegIkWorkspace Workspace { get; }

    bool TrySolve(
        Vec3 hipBody,
        Vec3 footTargetBody,
        FootTargetKind kind,
        out double[] q,
        out LegIkFailureCode code);

    /// <summary>
    /// Preferred plant under the hip at body-floor Z=0 (or solver-specific nominal).
    /// </summary>
    bool TryNominalStance(
        Vec3 hipBody,
        double hipYawRad,
        double sideSign,
        double hipStanceRad,
        double femurStanceFallbackRad,
        double tibiaStanceFallbackRad,
        double bodyClearanceMeters,
        out double[] q,
        out Vec3 footBody,
        out LegIkFailureCode code);
}
