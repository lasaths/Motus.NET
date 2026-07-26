using Motus.Core;

namespace Motus.Geometry;

/// <summary>Configurable tolerances for <see cref="NumericalInverseKinematics"/> (Lynch &amp; Park §6.2.2 / §6.6).</summary>
public sealed class NumericalIkOptions
{
    /// <summary>Default matches historical Motus NR behavior (compat).</summary>
    public static NumericalIkOptions Default { get; } = new();

    /// <summary>MR-style aggressive cap (reference IKinBody often uses 20).</summary>
    public static NumericalIkOptions ModernRoboticsAggressive { get; } = new()
    {
        MaxIterations = 20,
        PositionToleranceMeters = 1e-4,
        OmegaToleranceRadians = 1e-3,
        FinalPositionToleranceMeters = 1e-3,
        FinalOmegaToleranceRadians = 0.01
    };

    public int MaxIterations { get; init; } = 400;
    public double PositionToleranceMeters { get; init; } = 1e-3;
    public double OmegaToleranceRadians { get; init; } = 1e-2;
    public double FinalPositionToleranceMeters { get; init; } = 5e-3;
    public double FinalOmegaToleranceRadians { get; init; } = 0.05;
}

/// <summary>Named NR IK failure reasons (NASA bar — no silent fail).</summary>
public static class NumericalIkFailureReasons
{
    public const string NoConvergence = "NoConvergence";
    public const string SingularJacobian = "SingularJacobian";
    public const string InvalidInput = "InvalidInput";
}

public readonly record struct NumericalIkResult(bool Success, JointState Solution, string? FailureReason, int Iterations);
