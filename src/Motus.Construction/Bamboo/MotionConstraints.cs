namespace Motus.Construction;

/// <summary>
/// The motion rules of a grounded, base-swapping robot, in one place. The planner, the collision scan and the tests
/// all read this profile instead of their own flags/constants, so they cannot drift apart.
/// </summary>
/// <remarks>
/// Rules that are structural in the planner (one clamped base, swaps at rest) cannot be switched off; a profile that
/// asks for that is rejected by <see cref="Validate"/>. <see cref="ClearanceMarginM"/> is the extra distance a pose
/// must keep from every obstacle at every sample: obstacles are grown by it before the exact Motus contact test, so any
/// margin in [0, 20 mm] is supported (see <see cref="Validate"/>). Between samples a segment is only sampled unless
/// <see cref="CertifiedSegments"/> is on.
/// </remarks>
public sealed record MotionConstraints
{
    /// <summary>Every move is a joint-space PTP segment between IK waypoints (no Cartesian straight lines).</summary>
    public bool PointToPointOnly { get; init; } = true;
    /// <summary>At every instant one gripper is clamped on a fixed object and is the IK base.</summary>
    public bool OneClampedBase { get; init; } = true;
    /// <summary>A base swap is its own key with the joints at rest (both grippers clamped).</summary>
    public bool SwapAtRest { get; init; } = true;
    /// <summary>Grippers clamp only anchors or members of the rigid, anchored subframe ("fixed" members).</summary>
    public bool FixedMemberRule { get; init; } = true;
    /// <summary>After every placement the free gripper parks before anything else happens.</summary>
    public bool ParkAfterPlace { get; init; } = true;
    /// <summary>Clearance to obstacles (metres), enforced by the shared check (obstacles grown by it; clamped objects and
    /// the held cane on its stock pegs exempt). Lassie: 2 mm.</summary>
    public double ClearanceMarginM { get; init; } = 0.002;
    /// <summary>Joint step of the coarse (search) segment check, degrees.</summary>
    public double SearchStepDeg { get; init; } = 2.0;
    /// <summary>Joint step every accepted segment is verified at, degrees.</summary>
    public double VerifyStepDeg { get; init; } = 0.4;
    /// <summary>Joint step of the canonical scan samples (playback / tests), degrees.</summary>
    public double ScanStepDeg { get; init; } = 0.5;
    /// <summary>Canonical samples per <see cref="ScanStepDeg"/> (n = ⌈oversample · Δ / step⌉).</summary>
    public double ScanOversample { get; init; } = 1.5;
    /// <summary>Jaw stroke per canonical sample, mm.</summary>
    public double JawStepMm { get; init; } = 0.25;
    /// <summary>Jaw stroke per coarse (search) sample, mm.</summary>
    public double SearchJawStepMm { get; init; } = 1.0;

    /// <summary>Also prove every verified segment free of robot ↔ environment contact between its samples
    /// (<see cref="BambooSegmentCertifier"/>); a segment that cannot be proven is rejected. Off: sampled only.</summary>
    public bool CertifiedSegments { get; init; }
    /// <summary>Bisection depth limit of a segment proof (deeper = uncertified).</summary>
    public int CertifyMaxDepth { get; init; } = 18;
    /// <summary>Pose-check budget of one segment proof (exceeded = uncertified).</summary>
    public int CertifyMaxChecks { get; init; } = 20000;

    /// <summary>The bamboo-truss profile (Lassie's rules).</summary>
    public static MotionConstraints Bamboo { get; } = new();

    /// <summary>Profile new planners and scans use by default.</summary>
    public static MotionConstraints Current { get; set; } = Bamboo;

    public MotionConstraints Validate()
    {
        if (!OneClampedBase) throw new NotSupportedException("The planner always keeps one gripper clamped as the IK base.");
        if (!SwapAtRest) throw new NotSupportedException("The planner only swaps the base at rest.");
        if (ClearanceMarginM < 0 || ClearanceMarginM > 0.02) throw new ArgumentException("Clearance margin must be in [0, 20 mm].");
        if (SearchStepDeg <= 0 || VerifyStepDeg <= 0 || ScanStepDeg <= 0 || ScanOversample < 1 || JawStepMm <= 0 || SearchJawStepMm <= 0)
            throw new ArgumentException("Sample resolutions must be positive (oversample ≥ 1).");
        if (CertifyMaxDepth < 1 || CertifyMaxDepth > 40 || CertifyMaxChecks < 1)
            throw new ArgumentException("Certification limits: depth in [1, 40], at least one check.");
        return this;
    }
}
