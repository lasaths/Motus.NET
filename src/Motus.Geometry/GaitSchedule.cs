namespace Motus.Geometry;

/// <summary>
/// Song &amp; Waldron duty-factor gait: stance fraction <c>β</c> + per-leg phase offsets in [0,1).
/// </summary>
/// <remarks>
/// DOI <see cref="LeggedMethodRefs.SongWaldron1987Doi"/>. <see cref="FromGroups"/> is sugar for
/// equal-slot swing partitions. <see cref="Auto"/> uses <c>G = max(2, ⌈N/(N−3)⌉)</c> with
/// round-robin on yaw-sorted indices (hex → tripod <c>[[0,2,4],[1,3,5]]</c>; N=4 → crawl).
/// </remarks>
public sealed class GaitSchedule
{
    public GaitSchedule(
        double dutyFactor,
        IReadOnlyList<double> phaseOffsets,
        string methodId,
        IReadOnlyList<int[]>? swingGroups = null)
    {
        DutyFactor = dutyFactor;
        PhaseOffsets = phaseOffsets;
        MethodId = methodId;
        SwingGroups = swingGroups;
    }

    /// <summary>Stance duty factor β ∈ (0,1] — fraction of cycle each leg is planted.</summary>
    public double DutyFactor { get; }

    /// <summary>Per-leg swing-start phase in [0,1).</summary>
    public IReadOnlyList<double> PhaseOffsets { get; }

    public string MethodId { get; }

    /// <summary>Optional group partition (when built via <see cref="FromGroups"/> / factories).</summary>
    public IReadOnlyList<int[]>? SwingGroups { get; }

    public int LegCount => PhaseOffsets.Count;

    /// <summary>Minimum simultaneous stance count under equal-slot groups; else floor(β·N).</summary>
    public int MinStanceCount
    {
        get
        {
            if (SwingGroups is { Count: > 0 } groups)
            {
                var n = LegCount;
                var maxSwing = 0;
                foreach (var g in groups)
                    if (g is not null && g.Length > maxSwing)
                        maxSwing = g.Length;
                return Math.Max(0, n - maxSwing);
            }

            return Math.Max(0, (int)Math.Floor(DutyFactor * LegCount + 1e-12));
        }
    }

    /// <summary>
    /// Whether <paramref name="leg"/> is in its swing window at <paramref name="cyclePhase01"/> ∈ [0,1).
    /// </summary>
    public bool IsSwinging(int leg, double cyclePhase01, out double localPhase01)
    {
        localPhase01 = 0;
        if (leg < 0 || leg >= PhaseOffsets.Count)
            return false;
        if (!double.IsFinite(cyclePhase01) || !double.IsFinite(DutyFactor))
            return false;

        var beta = Math.Clamp(DutyFactor, 0, 1);
        var swingDur = 1.0 - beta;
        if (swingDur <= 1e-12)
            return false;

        var phase = cyclePhase01 - Math.Floor(cyclePhase01);
        if (phase < 0) phase += 1.0;
        var phi = PhaseOffsets[leg];
        if (!double.IsFinite(phi))
            return false;
        phi -= Math.Floor(phi);
        if (phi < 0) phi += 1.0;

        var legPhase = phase - phi;
        if (legPhase < 0) legPhase += 1.0;
        if (legPhase >= swingDur)
            return false;

        localPhase01 = Math.Clamp(legPhase / swingDur, 0, 1);
        return true;
    }

    /// <summary>
    /// Validate against leg count. Static gaits need N≥4 and MinStanceCount≥3 unless
    /// <paramref name="allowDynamicGait"/>.
    /// </summary>
    public string? Validate(int legCount, bool allowDynamicGait = false)
    {
        if (legCount < 2)
            return "GaitSchedule needs ≥ 2 legs.";
        if (PhaseOffsets.Count != legCount)
            return $"PhaseOffsets length ({PhaseOffsets.Count}) must equal leg count ({legCount}).";
        if (!double.IsFinite(DutyFactor) || DutyFactor <= 0 || DutyFactor > 1)
            return $"DutyFactor β must be in (0,1], got {DutyFactor}.";
        if (string.IsNullOrWhiteSpace(MethodId))
            return "MethodId is required.";

        for (var i = 0; i < PhaseOffsets.Count; i++)
        {
            if (!double.IsFinite(PhaseOffsets[i]))
                return $"PhaseOffsets[{i}] is not finite.";
        }

        if (SwingGroups is not null)
        {
            var err = ValidateGroups(SwingGroups, legCount);
            if (err is not null) return err;
        }

        if (!allowDynamicGait)
        {
            if (legCount <= 3)
                return $"Static gait rejected for N={legCount} (need N≥4 or AllowDynamicGait).";
            if (MinStanceCount < 3)
                return $"Static gait needs MinStanceCount≥3 (got {MinStanceCount}, β={DutyFactor:F3}); set AllowDynamicGait for dynamic gaits.";
        }

        return null;
    }

    /// <summary>Equal-slot swing groups → β = 1 − 1/G, phase offset = g/G.</summary>
    public static GaitSchedule FromGroups(IReadOnlyList<int[]> swingGroups, string? methodId = null)
    {
        if (swingGroups is null || swingGroups.Count < 1)
            throw new ArgumentException("SwingGroups must contain ≥ 1 group.", nameof(swingGroups));

        var maxLeg = -1;
        foreach (var g in swingGroups)
        {
            if (g is null) continue;
            foreach (var leg in g)
                if (leg > maxLeg) maxLeg = leg;
        }

        if (maxLeg < 0)
            throw new ArgumentException("SwingGroups cover no legs.", nameof(swingGroups));

        var n = maxLeg + 1;
        var err = ValidateGroups(swingGroups, n);
        if (err is not null)
            throw new ArgumentException(err, nameof(swingGroups));

        var gCount = swingGroups.Count;
        var beta = 1.0 - 1.0 / gCount;
        var offsets = new double[n];
        var groupsCopy = new int[gCount][];
        for (var g = 0; g < gCount; g++)
        {
            groupsCopy[g] = (int[])swingGroups[g].Clone();
            var phi = g / (double)gCount;
            foreach (var leg in groupsCopy[g])
                offsets[leg] = phi;
        }

        return new GaitSchedule(
            beta,
            offsets,
            methodId ?? $"FromGroups(G={gCount},β={beta:F3})",
            groupsCopy);
    }

    /// <summary>Wave / crawl: one swing group per leg (G=N, β=1−1/N).</summary>
    public static GaitSchedule Wave(int legCount) =>
        FromGroups(SingletonGroups(legCount), $"Wave(N={legCount})");

    /// <summary>N=4 crawl alias of <see cref="Wave"/>.</summary>
    public static GaitSchedule Crawl(int legCount = 4) =>
        FromGroups(SingletonGroups(legCount), $"Crawl(N={legCount})");

    /// <summary>Hex alternating tripod [[0,2,4],[1,3,5]].</summary>
    public static GaitSchedule AlternatingTripod(int legCount = 6)
    {
        if (legCount != 6)
            throw new ArgumentException("AlternatingTripod requires N=6.", nameof(legCount));
        return FromGroups([[0, 2, 4], [1, 3, 5]], "AlternatingTripod");
    }

    /// <summary>
    /// Song–Waldron Auto: <c>G = max(2, ⌈N/(N−3)⌉)</c>, round-robin on ascending hip yaw.
    /// N≤3 builds a schedule that fails <see cref="Validate"/> unless AllowDynamicGait.
    /// </summary>
    public static GaitSchedule Auto(int legCount, IReadOnlyList<double>? hipYawsRad = null)
    {
        if (legCount < 2)
            throw new ArgumentException("Auto needs ≥ 2 legs.", nameof(legCount));

        if (legCount <= 3)
        {
            // ponytail: still return a schedule so Validate can name the static rejection.
            var dyn = FromGroups(SingletonGroups(legCount), $"Auto(N={legCount},dynamic)");
            return dyn;
        }

        var g = Math.Max(2, (int)Math.Ceiling(legCount / (double)(legCount - 3)));
        var order = SortedYawOrder(legCount, hipYawsRad);
        var groups = new List<int>[g];
        for (var i = 0; i < g; i++)
            groups[i] = new List<int>();
        for (var i = 0; i < order.Length; i++)
            groups[i % g].Add(order[i]);

        var arrays = new int[g][];
        for (var i = 0; i < g; i++)
            arrays[i] = groups[i].ToArray();

        return FromGroups(arrays, $"Auto(N={legCount},G={g})");
    }

    private static int[][] SingletonGroups(int n)
    {
        var groups = new int[n][];
        for (var i = 0; i < n; i++)
            groups[i] = [i];
        return groups;
    }

    private static int[] SortedYawOrder(int n, IReadOnlyList<double>? yaws)
    {
        var order = new int[n];
        for (var i = 0; i < n; i++) order[i] = i;
        if (yaws is null || yaws.Count != n)
            return order;

        Array.Sort(order, (a, b) =>
        {
            var c = yaws[a].CompareTo(yaws[b]);
            return c != 0 ? c : a.CompareTo(b);
        });
        return order;
    }

    private static string? ValidateGroups(IReadOnlyList<int[]> swingGroups, int legCount)
    {
        var seen = new bool[legCount];
        var covered = 0;
        for (var g = 0; g < swingGroups.Count; g++)
        {
            var group = swingGroups[g];
            if (group is null || group.Length == 0)
                return $"SwingGroups[{g}] is empty.";
            foreach (var leg in group)
            {
                if (leg < 0 || leg >= legCount)
                    return $"SwingGroups[{g}] contains out-of-range leg index {leg}.";
                if (seen[leg])
                    return $"Leg {leg} appears in more than one swing group.";
                seen[leg] = true;
                covered++;
            }
        }

        return covered == legCount
            ? null
            : $"SwingGroups must partition all {legCount} legs (covered {covered}).";
    }
}
