namespace Motus.Core;

/// <summary>Unit of a generalized joint coordinate in <see cref="JointState"/> / <see cref="JointLimit"/>.</summary>
public enum JointCoordinateUnit
{
    /// <summary>Revolute joints (serial arms). Values are radians.</summary>
    Radians = 0,
    /// <summary>Prismatic joints (e.g. Stewart leg lengths). Values are meters.</summary>
    Meters = 1
}
