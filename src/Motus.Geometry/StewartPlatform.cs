using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Stewart/Gough platform: fixed base anchors B[6], platform anchors P[6] in the platform body frame,
/// six prismatic leg stroke limits. Units: meters.
/// </summary>
public sealed class StewartPlatform
{
    public const int LegCount = 6;

    public string ModelName { get; }
    public IReadOnlyList<Vec3> BaseAnchors { get; }
    public IReadOnlyList<Vec3> PlatformAnchors { get; }
    public IReadOnlyList<JointLimit> StrokeLimits { get; }
    public Frame ToolOffset { get; }
    public StewartSolverOptions SolverOptions { get; }

    public StewartPlatform(
        string modelName,
        IReadOnlyList<Vec3> baseAnchors,
        IReadOnlyList<Vec3> platformAnchors,
        IReadOnlyList<JointLimit> strokeLimits,
        Frame? toolOffset = null,
        StewartSolverOptions? solverOptions = null)
    {
        if (string.IsNullOrWhiteSpace(modelName))
            throw new ArgumentException("ModelName is required.", nameof(modelName));
        ValidateAnchors(baseAnchors, nameof(baseAnchors));
        ValidateAnchors(platformAnchors, nameof(platformAnchors));
        if (strokeLimits is null || strokeLimits.Count != LegCount)
            throw new ArgumentException($"Exactly {LegCount} stroke limits are required.", nameof(strokeLimits));
        for (var i = 0; i < LegCount; i++)
        {
            if (strokeLimits[i].Unit != JointCoordinateUnit.Meters)
                throw new ArgumentException($"Stroke limit {i} must use JointCoordinateUnit.Meters.", nameof(strokeLimits));
            if (strokeLimits[i].Min <= 0 || strokeLimits[i].Max <= strokeLimits[i].Min)
                throw new ArgumentException($"Stroke limit {i} must satisfy 0 < min < max.", nameof(strokeLimits));
        }

        ModelName = modelName.Trim();
        BaseAnchors = baseAnchors.ToArray();
        PlatformAnchors = platformAnchors.ToArray();
        StrokeLimits = strokeLimits.ToArray();
        ToolOffset = toolOffset ?? Frame.Identity;
        SolverOptions = solverOptions ?? StewartSolverOptions.Default;
        EnsureNonDegenerate();
    }

    /// <summary>
    /// Classic hex layout: base/platform circles with staggered angular pairs (meters).
    /// </summary>
    public static StewartPlatform CreateClassic(
        string modelName,
        double baseRadiusMeters,
        double platformRadiusMeters,
        double minStrokeMeters,
        double maxStrokeMeters,
        double basePairSeparationRadians = 0.15,
        double platformPairSeparationRadians = 0.15,
        Frame? toolOffset = null,
        StewartSolverOptions? solverOptions = null)
    {
        if (!double.IsFinite(baseRadiusMeters) || baseRadiusMeters <= 0)
            throw new ArgumentOutOfRangeException(nameof(baseRadiusMeters));
        if (!double.IsFinite(platformRadiusMeters) || platformRadiusMeters <= 0)
            throw new ArgumentOutOfRangeException(nameof(platformRadiusMeters));

        var baseAnchors = new Vec3[LegCount];
        var platformAnchors = new Vec3[LegCount];
        for (var i = 0; i < 3; i++)
        {
            var center = i * (2.0 * Math.PI / 3.0);
            var b0 = center - basePairSeparationRadians * 0.5;
            var b1 = center + basePairSeparationRadians * 0.5;
            // Platform pairs rotated by 60°; cross within each sector (b0→p1, b1→p0)
            // so legs are not nearly parallel — non-crossed pairing is singular at home.
            var pCenter = center + Math.PI / 3.0;
            var p0 = pCenter - platformPairSeparationRadians * 0.5;
            var p1 = pCenter + platformPairSeparationRadians * 0.5;
            baseAnchors[2 * i] = OnCircle(baseRadiusMeters, b0);
            baseAnchors[2 * i + 1] = OnCircle(baseRadiusMeters, b1);
            platformAnchors[2 * i] = OnCircle(platformRadiusMeters, p1);
            platformAnchors[2 * i + 1] = OnCircle(platformRadiusMeters, p0);
        }

        var limits = Enumerable.Range(0, LegCount)
            .Select(_ => JointLimit.Meters(minStrokeMeters, maxStrokeMeters))
            .ToArray();
        return new StewartPlatform(modelName, baseAnchors, platformAnchors, limits, toolOffset, solverOptions);
    }

    public RobotPreset ToPreset(string? notes = null)
    {
        var names = Enumerable.Range(1, LegCount).Select(i => $"leg_{i}").ToArray();
        return new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = ModelName,
            Family = Units.StewartFamily,
            AxisCount = LegCount,
            JointLimits = StrokeLimits.ToArray(),
            BaseFrame = BaseFrame.Identity,
            ToolFrame = new ToolFrame(ToolOffset, "platform_tcp"),
            Notes = notes ?? "Stewart/Gough platform; JointState positions are leg lengths in meters.",
            SourceNote = "Motus.Geometry.StewartPlatform"
        };
    }

    public RobotModel ToModel(IReadOnlyList<string>? jointNames = null) =>
        new(ToPreset(), jointNames: jointNames ?? Enumerable.Range(1, LegCount).Select(i => $"leg_{i}").ToArray());

    public void LegLengthAtPose(Frame platformPose, Span<double> lengths)
    {
        if (lengths.Length < LegCount)
            throw new ArgumentException($"Need at least {LegCount} slots.", nameof(lengths));
        var m = Transforms.FromFrame(platformPose);
        for (var i = 0; i < LegCount; i++)
        {
            var p = PlatformAnchors[i];
            Transforms.TransformPointInto(m, p.X, p.Y, p.Z, out var wx, out var wy, out var wz);
            var b = BaseAnchors[i];
            var dx = wx - b.X;
            var dy = wy - b.Y;
            var dz = wz - b.Z;
            lengths[i] = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }
    }

    public JointState HomeLengths()
    {
        // Mid-stroke home at a neutral raised pose (identity orientation, z = mid of first stroke).
        var mid = 0.5 * (StrokeLimits[0].Min + StrokeLimits[0].Max);
        var homePose = new Frame(0, 0, mid);
        Span<double> L = stackalloc double[LegCount];
        LegLengthAtPose(homePose, L);
        // If mid-height is outside stroke for this geometry, clamp each to mid of its stroke.
        var q = new double[LegCount];
        for (var i = 0; i < LegCount; i++)
        {
            var li = L[i];
            if (!StrokeLimits[i].Contains(li))
                li = 0.5 * (StrokeLimits[i].Min + StrokeLimits[i].Max);
            q[i] = li;
        }
        return new JointState(q);
    }

    private static Vec3 OnCircle(double radius, double angle) =>
        new(radius * Math.Cos(angle), radius * Math.Sin(angle), 0);

    private static void ValidateAnchors(IReadOnlyList<Vec3> anchors, string name)
    {
        if (anchors is null || anchors.Count != LegCount)
            throw new ArgumentException($"Exactly {LegCount} anchors are required.", name);
        for (var i = 0; i < LegCount; i++)
        {
            if (!anchors[i].IsFinite)
                throw new ArgumentException($"Anchor {i} must be finite.", name);
        }
    }

    private void EnsureNonDegenerate()
    {
        // Reject if all base (or platform) points are nearly coincident.
        var baseSpan = PointCloudSpan(BaseAnchors);
        var platSpan = PointCloudSpan(PlatformAnchors);
        if (baseSpan < 1e-6 || platSpan < 1e-6)
            throw new ArgumentException("Stewart anchors are degenerate (near-zero span).");
    }

    private static double PointCloudSpan(IReadOnlyList<Vec3> pts)
    {
        var minX = pts[0].X; var maxX = pts[0].X;
        var minY = pts[0].Y; var maxY = pts[0].Y;
        var minZ = pts[0].Z; var maxZ = pts[0].Z;
        for (var i = 1; i < pts.Count; i++)
        {
            minX = Math.Min(minX, pts[i].X); maxX = Math.Max(maxX, pts[i].X);
            minY = Math.Min(minY, pts[i].Y); maxY = Math.Max(maxY, pts[i].Y);
            minZ = Math.Min(minZ, pts[i].Z); maxZ = Math.Max(maxZ, pts[i].Z);
        }
        var dx = maxX - minX; var dy = maxY - minY; var dz = maxZ - minZ;
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }
}

public readonly struct Vec3
{
    public double X { get; }
    public double Y { get; }
    public double Z { get; }

    public Vec3(double x, double y, double z)
    {
        X = x; Y = y; Z = z;
    }

    public bool IsFinite => double.IsFinite(X) && double.IsFinite(Y) && double.IsFinite(Z);

    public double Length => Math.Sqrt(X * X + Y * Y + Z * Z);
}

/// <summary>Documented Stewart FK/path defaults (ADR 0003).</summary>
public sealed class StewartSolverOptions
{
    public double FkPositionTolMeters { get; init; } = 1e-6;
    public double FkOrientationTolRadians { get; init; } = 1e-6;
    public int FkMaxIterations { get; init; } = 40;
    /// <summary>Reserved; FK no longer gates on ‖J‖∞·‖J⁻¹‖∞ (mixed m/rad FD inflated it).</summary>
    public double JacobianConditionLimit { get; init; } = 1e12;
    public double MaxLegDeltaPerStepMeters { get; init; } = 0.05;
    public double FiniteDiffStepMeters { get; init; } = 1e-7;
    public double FiniteDiffStepRadians { get; init; } = 1e-7;

    public static StewartSolverOptions Default { get; } = new();
}
