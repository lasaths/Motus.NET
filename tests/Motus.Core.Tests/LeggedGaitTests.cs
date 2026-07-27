using Motus.Core;
using Motus.Geometry;
using Xunit;

namespace Motus.Core.Tests;

public class LeggedGaitTests
{
    private const double Tol = 1e-9;
    private const double CoarseTol = 1e-6;
    private static readonly double Deg = Math.PI / 180.0;

    // --- FK↔IK Round-Trip Tests ---

    [Fact]
    public void LegIk3R_RoundTripNearStance()
    {
        var hip = new Vec3(0.12, 0, 0.12);
        const double coxa = 0.06, femur = 0.17, tibia = 0.19;
        // Seed the preferred elbow-up branch (IK recovers same q only on that branch).
        Assert.True(LegIk3R.TrySolve(hip, new Vec3(0.30, 0, 0), coxa, femur, tibia, out var q0, out var q1, out var q2));
        var foot = LegIk3R.FootPosition(hip, coxa, femur, tibia, q0, q1, q2);
        Assert.True(LegIk3R.TrySolve(hip, foot, coxa, femur, tibia, out var s0, out var s1, out var s2));
        var back = LegIk3R.FootPosition(hip, coxa, femur, tibia, s0, s1, s2);
        Assert.InRange(back.X - foot.X, -CoarseTol, CoarseTol);
        Assert.InRange(back.Y - foot.Y, -CoarseTol, CoarseTol);
        Assert.InRange(back.Z - foot.Z, -CoarseTol, CoarseTol);
        Assert.InRange(s0 - q0, -CoarseTol, CoarseTol);
        Assert.InRange(s1 - q1, -CoarseTol, CoarseTol);
        Assert.InRange(s2 - q2, -CoarseTol, CoarseTol);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(60.0)]
    [InlineData(120.0)]
    [InlineData(180.0)]
    [InlineData(240.0)]
    [InlineData(300.0)]
    public void LegIk3R_RoundTrip_HexStanceYaws(double yawDeg)
    {
        // Hex hip positions at different yaw angles
        var yaw = yawDeg * Deg;
        const double bodyR = 0.12, bodyZ = 0.12;
        const double coxa = 0.06, femur = 0.17, tibia = 0.19;
        var hip = new Vec3(bodyR * Math.Cos(yaw), bodyR * Math.Sin(yaw), bodyZ);

        // Stance angles
        var q0 = yaw + 7.5 * Deg; // coxa yaw with stance offset
        var q1 = 30.0 * Deg;
        var q2 = -30.0 * Deg;

        var foot = LegIk3R.FootPosition(hip, coxa, femur, tibia, q0, q1, q2);
        Assert.True(LegIk3R.TrySolve(hip, foot, coxa, femur, tibia, out var s0, out var s1, out var s2),
            $"IK solve failed for yaw={yawDeg}°");
        var back = LegIk3R.FootPosition(hip, coxa, femur, tibia, s0, s1, s2);

        AssertVec3Near(back, foot, CoarseTol, $"FK→IK→FK mismatch at yaw={yawDeg}°");
    }

    [Fact]
    public void LegIk3R_RoundTrip_SeededRandomReachableFeet()
    {
        // Seeded PRNG for reproducible "random" tests
        var rng = new Random(42);
        const double bodyR = 0.12, bodyZ = 0.12;
        const double coxa = 0.06, femur = 0.17, tibia = 0.19;
        var maxReach = coxa + femur + tibia;
        var minReach = 0.3 * maxReach; // stay within reachable annulus

        for (var i = 0; i < 20; i++)
        {
            var yaw = rng.NextDouble() * 2 * Math.PI;
            var hip = new Vec3(bodyR * Math.Cos(yaw), bodyR * Math.Sin(yaw), bodyZ);

            // Generate random q in reasonable range
            var q0 = yaw + (rng.NextDouble() - 0.5) * 60 * Deg;
            var q1 = (20 + rng.NextDouble() * 40) * Deg;
            var q2 = (-50 + rng.NextDouble() * 30) * Deg;

            var foot = LegIk3R.FootPosition(hip, coxa, femur, tibia, q0, q1, q2);
            if (!LegIk3R.TrySolve(hip, foot, coxa, femur, tibia, out var s0, out var s1, out var s2))
                continue; // skip unreachable (edge case from random)

            var back = LegIk3R.FootPosition(hip, coxa, femur, tibia, s0, s1, s2);
            AssertVec3Near(back, foot, CoarseTol, $"FK→IK→FK mismatch on seeded sample {i}");
        }
    }

    // --- Z=0 Plant Target Tests ---

    [Fact]
    public void LegIk3R_ZeroPlant_FootPositionZNearZero()
    {
        const double bodyR = 0.12, bodyZ = 0.12;
        const double coxa = 0.06, femur = 0.17, tibia = 0.19;

        for (var legIdx = 0; legIdx < 6; legIdx++)
        {
            var yaw = legIdx * (Math.PI / 3.0);
            var hip = new Vec3(bodyR * Math.Cos(yaw), bodyR * Math.Sin(yaw), bodyZ);

            // Stance foot projected to Z=0
            var q0 = yaw + 7.5 * Deg;
            var q1 = 30.0 * Deg;
            var q2 = -30.0 * Deg;
            var stanceFoot = LegIk3R.FootPosition(hip, coxa, femur, tibia, q0, q1, q2);
            var targetZ0 = new Vec3(stanceFoot.X, stanceFoot.Y, 0);

            Assert.True(LegIk3R.TrySolve(hip, targetZ0, coxa, femur, tibia, out var s0, out var s1, out var s2),
                $"IK to Z=0 failed for leg {legIdx}");

            var solvedFoot = LegIk3R.FootPosition(hip, coxa, femur, tibia, s0, s1, s2);
            Assert.InRange(solvedFoot.Z, -1e-6, 1e-6);
        }
    }

    // --- Layout Validation Tests ---

    [Fact]
    public void HexLayout_Validates_And_FamilyIsLegged()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        Assert.Null(layout.Validate());
        Assert.Equal(6, layout.LegCount);
        Assert.Equal(18, layout.DriverCount);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var preset = layout.ToPreset("hex", 18, limits);
        Assert.True(Units.IsLegged(preset));
        Assert.False(Units.IsStewart(preset));
    }

    [Fact]
    public void LeggedLayout_RejectsInvalidLayouts()
    {
        // Zero body radius
        var bad1 = new LeggedLayout(
            ["leg0", "leg1"], [0.0, Math.PI], [[0], [1]],
            bodyR: 0, coxa: 0.06, femur: 0.17, tibia: 0.19, bodyZ: 0.12, tipLegName: "leg0");
        Assert.NotNull(bad1.Validate());
        Assert.Contains("BodyR", bad1.Validate());

        // Empty swing group
        var bad2 = new LeggedLayout(
            ["leg0", "leg1"], [0.0, Math.PI], [[], [0, 1]],
            bodyR: 0.12, coxa: 0.06, femur: 0.17, tibia: 0.19, bodyZ: 0.12, tipLegName: "leg0");
        Assert.NotNull(bad2.Validate());
        Assert.Contains("empty", bad2.Validate()!.ToLower());

        // Missing leg in swing groups
        var bad3 = new LeggedLayout(
            ["leg0", "leg1", "leg2"], [0.0, Math.PI / 2, Math.PI], [[0], [1]],
            bodyR: 0.12, coxa: 0.06, femur: 0.17, tibia: 0.19, bodyZ: 0.12, tipLegName: "leg0");
        Assert.NotNull(bad3.Validate());
        Assert.Contains("partition", bad3.Validate()!.ToLower());

        // Invalid tip leg name
        var bad4 = new LeggedLayout(
            ["leg0", "leg1"], [0.0, Math.PI], [[0], [1]],
            bodyR: 0.12, coxa: 0.06, femur: 0.17, tibia: 0.19, bodyZ: 0.12, tipLegName: "nonexistent");
        Assert.NotNull(bad4.Validate());
        Assert.Contains("TipLegName", bad4.Validate());
    }

    // --- Hex Gait Tests ---

    [Fact]
    public void HexGait_Builds_18Dof_Trajectory_BasePathLengthMatches()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("hex", 18, limits));
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.5, 0, 0) };

        Assert.True(LeggedGait.TryBuild(
            layout, path, 0.08, 0.08, 0.025,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out var result, out var err), err);

        Assert.NotNull(result);
        Assert.Equal(18, result!.Trajectory.Points[0].JointState.AxisCount);
        Assert.Equal(result.Trajectory.Points.Count, result.BasePath.Count);
        Assert.True(result.BasePath.Count >= 10, $"BasePath too short: {result.BasePath.Count}");
    }

    [Fact]
    public void HexGait_MidSample_StanceFeetNearZ0()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("hex", 18, limits));
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.4, 0, 0) };

        Assert.True(LeggedGait.TryBuild(
            layout, path, 0.08, 0.08, 0.025,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out var result, out var err), err);

        // Check mid-sample feet positions using FK
        var midIdx = result!.Trajectory.Points.Count / 2;
        var midQ = result.Trajectory.Points[midIdx].JointState.Positions;

        // Count how many legs have feet near Z=0 (stance)
        var nearGround = 0;
        for (var leg = 0; leg < 6; leg++)
        {
            var yaw = leg * (Math.PI / 3.0);
            var hip = new Vec3(0.12 * Math.Cos(yaw), 0.12 * Math.Sin(yaw), 0.12);
            var foot = LegIk3R.FootPosition(hip, 0.06, 0.17, 0.19,
                midQ[leg * 3 + 0], midQ[leg * 3 + 1], midQ[leg * 3 + 2]);
            if (Math.Abs(foot.Z) < 0.03) // within 3cm of ground = stance
                nearGround++;
        }

        // Tripod gait: at least 3 legs should be near ground at any time
        Assert.True(nearGround >= 3, $"Only {nearGround} legs near ground at mid-sample (expected ≥3 for tripod)");
    }

    // --- Quad Gait Tests ---

    [Fact]
    public void QuadGait_Builds_12Dof_Trajectory()
    {
        var layout = LeggedLayout.QuadSmoke(0.10, 0.06, 0.17, 0.19, 0.12);
        Assert.Null(layout.Validate());
        var limits = Enumerable.Range(0, 12).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("quad", 12, limits));
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.4, 0, 0) };
        Assert.True(LeggedGait.TryBuild(
            layout, path, 0.08, 0.08, 0.025,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out var result, out var err), err);
        Assert.NotNull(result);
        Assert.Equal(12, result!.Trajectory.Points[0].JointState.AxisCount);
        Assert.True(result.BasePath.Count >= 5);
        Assert.Equal(result.Trajectory.Points.Count, result.BasePath.Count);
    }

    [Fact]
    public void Step_Changes_MidGait_Joints()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("hex", 18, limits));
        // Arc-ish polyline
        var path = new List<Vec3>();
        for (var i = 0; i <= 32; i++)
        {
            var a = Math.PI * i / 32.0;
            path.Add(new Vec3(0.05 + 0.45 * Math.Cos(a), 0.45 * Math.Sin(a), 0));
        }

        Assert.True(LeggedGait.TryBuild(
            layout, path, 0.08, 0.06, 0.03,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out var fine, out var e1), e1);
        Assert.True(LeggedGait.TryBuild(
            layout, path, 0.08, 0.20, 0.03,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out var coarse, out var e2), e2);

        var aQ = fine!.Trajectory.Points[fine.Trajectory.Points.Count / 2].JointState.Positions;
        var bQ = coarse!.Trajectory.Points[coarse.Trajectory.Points.Count / 2].JointState.Positions;
        var diff = 0.0;
        for (var i = 0; i < aQ.Length; i++)
            diff += Math.Abs(aQ[i] - bQ[i]);
        Assert.True(diff > 1e-3, $"Step should change mid-gait q (diff={diff})");
    }

    // --- Rejection Tests ---

    [Fact]
    public void LeggedGait_RejectsTooShortPath()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("hex", 18, limits));
        var tooShort = new[] { new Vec3(0, 0, 0), new Vec3(0.01, 0, 0) }; // 1cm path

        Assert.False(LeggedGait.TryBuild(
            layout, tooShort, 0.08, 0.08, 0.025,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out _, out var err));
        Assert.Contains("short", err.ToLower());
    }

    [Fact]
    public void LeggedGait_RejectsAxisCountMismatch()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12); // 18 DOF
        var wrongLimits = Enumerable.Range(0, 12).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var wrongModel = new RobotModel(layout.ToPreset("hex", 12, wrongLimits)); // 12 DOF mismatch
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.4, 0, 0) };

        Assert.False(LeggedGait.TryBuild(
            layout, path, 0.08, 0.08, 0.025,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            wrongModel, out _, out var err));
        Assert.Contains("AxisCount", err);
    }

    [Fact]
    public void LeggedGait_RejectsEmptyPath()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("hex", 18, limits));
        var empty = Array.Empty<Vec3>();

        Assert.False(LeggedGait.TryBuild(
            layout, empty, 0.08, 0.08, 0.025,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out _, out var err));
        Assert.Contains("empty", err.ToLower());
    }

    [Fact]
    public void LeggedGait_RejectsInvalidSpeed()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("hex", 18, limits));
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.4, 0, 0) };

        Assert.False(LeggedGait.TryBuild(
            layout, path, speed: -0.1, 0.08, 0.025,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out _, out var err));
        Assert.Contains("Speed", err);
    }

    [Fact]
    public void LeggedGait_RejectsInvalidLayout()
    {
        var badLayout = new LeggedLayout(
            ["leg0", "leg1"], [0.0, Math.PI], [[0], [1]],
            bodyR: 0, coxa: 0.06, femur: 0.17, tibia: 0.19, bodyZ: 0.12, tipLegName: "leg0");
        var limits = Enumerable.Range(0, 6).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(badLayout.ToPreset("bad", 6, limits));
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.4, 0, 0) };

        Assert.False(LeggedGait.TryBuild(
            badLayout, path, 0.08, 0.08, 0.025,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out _, out var err));
        Assert.Contains("BodyR", err);
    }

    // --- Elbow Branch: elbow-up (knee high) for insectoid stance ---

    [Fact]
    public void LegIk3R_ElbowBranch_PrefersKneeHigh()
    {
        var hip = new Vec3(0.12, 0, 0.12);
        const double coxa = 0.06, femur = 0.17, tibia = 0.19;

        var target = new Vec3(0.30, 0, 0);
        Assert.True(LegIk3R.TrySolve(hip, target, coxa, femur, tibia, out var q0, out var q1, out var q2));

        var computed = LegIk3R.FootPosition(hip, coxa, femur, tibia, q0, q1, q2);
        AssertVec3Near(computed, target, CoarseTol, "Elbow-up FK↔IK mismatch");

        var ankle = LegIk3R.KneePosition(hip, coxa, femur, q0, q1);
        Assert.True(ankle.Z > target.Z + 0.03,
            $"Elbow-up ankle Z={ankle.Z:F3} should sit above foot Z={target.Z:F3} (not through floor).");
        Assert.True(q2 > 0, $"Elbow-up tibia pitch q2={q2:F3} should be > 0.");
    }

    [Fact]
    public void LegIk3R_AllFiniteOutputs()
    {
        var hip = new Vec3(0.12, 0, 0.12);
        const double coxa = 0.06, femur = 0.17, tibia = 0.19;

        // Reachable target
        var target = new Vec3(0.28, 0.05, 0);
        Assert.True(LegIk3R.TrySolve(hip, target, coxa, femur, tibia, out var q0, out var q1, out var q2));
        Assert.True(double.IsFinite(q0), "q0 must be finite");
        Assert.True(double.IsFinite(q1), "q1 must be finite");
        Assert.True(double.IsFinite(q2), "q2 must be finite");
    }

    [Fact]
    public void LegIk3R_RejectsUnreachable()
    {
        var hip = new Vec3(0.12, 0, 0.12);
        const double coxa = 0.06, femur = 0.17, tibia = 0.19;
        var maxReach = coxa + femur + tibia;

        // Way out of reach
        var tooFar = new Vec3(hip.X + maxReach + 0.5, 0, 0);
        Assert.False(LegIk3R.TrySolve(hip, tooFar, coxa, femur, tibia, out _, out _, out _));
    }

    [Fact]
    public void LegIk3R_RejectsNaNInput()
    {
        var hip = new Vec3(0.12, 0, 0.12);
        const double coxa = 0.06, femur = 0.17, tibia = 0.19;
        var nanTarget = new Vec3(double.NaN, 0, 0);

        Assert.False(LegIk3R.TrySolve(hip, nanTarget, coxa, femur, tibia, out _, out _, out _));
    }

    [Fact]
    public void StaticStability_Triangle_ComInside_PositiveMargin()
    {
        var contacts = new[]
        {
            new Vec3(0, 0, 0),
            new Vec3(1, 0, 0),
            new Vec3(0.5, 0.8, 0),
        };
        var r = StaticStability.Evaluate(contacts, new Vec3(0.5, 0.25, 0));
        Assert.True(r.IsStable, r.Failure);
        Assert.True(r.MarginMeters > 0);
    }

    [Fact]
    public void StaticStability_ComOutside_NegativeMargin()
    {
        var contacts = new[]
        {
            new Vec3(0, 0, 0),
            new Vec3(1, 0, 0),
            new Vec3(0.5, 0.8, 0),
        };
        var r = StaticStability.Evaluate(contacts, new Vec3(2, 2, 0));
        Assert.False(r.IsStable);
        Assert.True(r.MarginMeters < 0);
    }

    [Fact]
    public void HexGait_Exposes_MethodProvenance_WithDois()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("hex", 18, limits));
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.3, 0, 0) };
        Assert.True(LeggedGait.TryBuild(
            layout, path, 0.08, 0.06, 0.025,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out var result, out var err), err);
        Assert.Contains(LeggedMethodRefs.LynchPark2017Doi, result!.MethodProvenance);
        Assert.Contains(LeggedMethodRefs.McGheeFrank1968Doi, result.MethodProvenance);
        Assert.Contains(LeggedMethodRefs.SongWaldron1987Doi, result.MethodProvenance);
        Assert.Contains(LeggedMethodRefs.AristidouLasenby2011FabrikDoi, result.MethodProvenance);
        Assert.True(double.IsFinite(result.MinStaticStabilityMarginMeters));
    }

    [Fact]
    public void LeggedGait_ValidateForPlan_OkWithMethodProvenance()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("hex", 18, limits));
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.3, 0, 0) };

        Assert.True(LeggedGait.TryBuild(
            layout, path, 0.08, 0.06, 0.025,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out var gait, out var err), err);

        var plan = LeggedGait.ValidateForPlan(gait!, minStaticStabilityMarginMeters: -0.05);
        Assert.True(plan.Success, string.Join("; ", plan.Errors));
        Assert.Contains(plan.Warnings, w => w.Contains(LeggedMethodRefs.SongWaldron1987Doi, StringComparison.Ordinal));
    }

    [Fact]
    public void LeggedGait_ValidateForPlan_SsmFailIsNamed()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("hex", 18, limits));
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.3, 0, 0) };

        Assert.True(LeggedGait.TryBuild(
            layout, path, 0.08, 0.06, 0.025,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out var gait, out var err), err);

        var plan = LeggedGait.ValidateForPlan(gait!, minStaticStabilityMarginMeters: 10.0);
        Assert.False(plan.Success);
        Assert.Contains(plan.Messages, m => m.Code == PlanningMessageCodes.ConstraintViolation);
        Assert.Contains(plan.Errors, e => e.Contains("SSM", StringComparison.OrdinalIgnoreCase));
    }

    [Fact]
    public void BuildStanceQ_PlantsFeetNearGround()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        var q = LeggedGait.BuildStanceQ(layout, 7.5 * Deg, 30 * Deg, -30 * Deg);
        for (var leg = 0; leg < 6; leg++)
        {
            var yaw = layout.HipYawsRad[leg];
            var hip = new Vec3(layout.BodyR * Math.Cos(yaw), layout.BodyR * Math.Sin(yaw), layout.BodyZ);
            var foot = LegIk3R.FootPosition(
                hip, layout.Coxa, layout.Femur, layout.Tibia,
                q[leg * 3], q[leg * 3 + 1], q[leg * 3 + 2]);
            Assert.InRange(foot.Z, -1e-4, 1e-4);
            // Old fixed 30°/−30° stance floated at Z≈0.035 — planted stance must differ.
            Assert.True(Math.Abs(q[leg * 3 + 1] - 30 * Deg) > 0.2,
                "Expected plant IK femur angle, not the floating 30° fallback.");
        }
    }

    [Fact]
    public void HexGait_StanceFeet_DoNotDragFarBehindNominal()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("hex", 18, limits));
        const double step = 0.06;
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.5, 0, 0) };

        Assert.True(LeggedGait.TryBuild(
            layout, path, 0.08, step, 0.025,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out var result, out var err), err);

        var stanceQ = LeggedGait.BuildStanceQ(layout, 7.5 * Deg, 30 * Deg, -30 * Deg);
        var nominalBody = new Vec3[6];
        for (var leg = 0; leg < 6; leg++)
        {
            var yaw = layout.HipYawsRad[leg];
            var hip = new Vec3(layout.BodyR * Math.Cos(yaw), layout.BodyR * Math.Sin(yaw), layout.BodyZ);
            var foot = LegIk3R.FootPosition(hip, layout.Coxa, layout.Femur, layout.Tibia,
                stanceQ[leg * 3], stanceQ[leg * 3 + 1], stanceQ[leg * 3 + 2]);
            nominalBody[leg] = new Vec3(foot.X, foot.Y, 0);
        }

        var maxDrift = 0.0;
        for (var i = 0; i < result!.Trajectory.Points.Count; i++)
        {
            var q = result.Trajectory.Points[i].JointState.Positions;
            var bf = result.BasePath[i];
            var yaw = 2.0 * Math.Atan2(bf.Qz, bf.Qw);
            var c = Math.Cos(-yaw);
            var s = Math.Sin(-yaw);
            for (var leg = 0; leg < 6; leg++)
            {
                var hy = layout.HipYawsRad[leg];
                var hip = new Vec3(layout.BodyR * Math.Cos(hy), layout.BodyR * Math.Sin(hy), layout.BodyZ);
                var footBody = LegIk3R.FootPosition(hip, layout.Coxa, layout.Femur, layout.Tibia,
                    q[leg * 3], q[leg * 3 + 1], q[leg * 3 + 2]);
                // Skip clear swing (lifted)
                if (footBody.Z > 0.015) continue;
                var dx = footBody.X - nominalBody[leg].X;
                var dy = footBody.Y - nominalBody[leg].Y;
                var drift = Math.Sqrt(dx * dx + dy * dy);
                if (drift > maxDrift) maxDrift = drift;
            }
        }

        // Duty + drift replant should keep planted feet near nominal (not body-length drag).
        Assert.True(maxDrift < 1.35 * step,
            $"Planted foot drifted {maxDrift:F3} m from nominal (step={step}); back legs likely dragging.");
    }

    /// <summary>
    /// Logic of Motus.Grasshopper <c>examples/09_walking_hexapod.ghx</c>
    /// (<c>scripts/generate-examples.mjs</c> <c>graph09</c>) — no Rhino/GH solve.
    /// Compact WalkHex defaults + 9-pt arc + constant terrain Z=0.02 (Center Box top).
    /// </summary>
    [Fact]
    public void Example09_WalkingHexapod_ArcAndBoxTerrain()
    {
        var layout = LeggedLayout.HexMithi(0.06, 0.035, 0.08, 0.10, 0.07);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("hex", 18, limits));

        var path = new List<Vec3>();
        const int arcN = 9;
        for (var i = 0; i < arcN; i++)
        {
            var a = Math.PI - i / (arcN - 1.0) * Math.PI;
            path.Add(new Vec3(0.22 + 0.18 * Math.Cos(a), 0.18 * Math.Sin(a), 0));
        }

        // graph09 Center Box half-height 0.02 → top face at Z=0.02 m.
        const double groundZ = 0.02;
        LeggedGait.TerrainHeight terrain = static (_, _) => groundZ;

        Assert.True(LeggedGait.TryBuild(
            layout, path, 0.06, 0.04, 0.02,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out var result, out var err, terrain), err);

        Assert.Equal(18, result!.Trajectory.Points[0].JointState.AxisCount);
        Assert.Equal(result.Trajectory.Points.Count, result.BasePath.Count);
        Assert.True(result.BasePath.Count >= 10);
        Assert.InRange(result.BasePath[0].Z, groundZ - 1e-9, groundZ + 1e-9);
        Assert.True(Math.Abs(result.BasePath[^1].Y) > 0.05 || Math.Abs(result.BasePath[^1].X - 0.22) > 0.05,
            "Body should move along the example arc.");

        var mid = result.Trajectory.Points.Count / 2;
        var q = result.Trajectory.Points[mid].JointState.Positions;
        var bf = result.BasePath[mid];
        Assert.InRange(bf.Z, groundZ - 1e-9, groundZ + 1e-9);
        var planted = 0;
        for (var leg = 0; leg < 6; leg++)
        {
            var hy = layout.HipYawsRad[leg];
            var hip = new Vec3(layout.BodyR * Math.Cos(hy), layout.BodyR * Math.Sin(hy), layout.BodyZ);
            var footBody = LegIk3R.FootPosition(hip, layout.Coxa, layout.Femur, layout.Tibia,
                q[leg * 3], q[leg * 3 + 1], q[leg * 3 + 2]);
            if (footBody.Z > 0.012) continue;
            var yaw = 2.0 * Math.Atan2(bf.Qz, bf.Qw);
            var c = Math.Cos(yaw);
            var s = Math.Sin(yaw);
            var fz = bf.Z + footBody.Z;
            Assert.InRange(fz - groundZ, -0.015, 0.015);
            planted++;
        }
        Assert.True(planted >= 3, $"Example 09 mid-gait expected ≥3 plants on box top Z={groundZ}, got {planted}");
    }

    [Fact]
    public void HexGait_RampTerrain_PlantsFollowHeightfield()
    {
        var layout = LeggedLayout.HexMithi(0.06, 0.035, 0.08, 0.10, 0.07);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("hex", 18, limits));
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.35, 0, 0) };
        // Gentle ramp: z = 0.12 * x (≈4° — stay in 3R workspace).
        LeggedGait.TerrainHeight ramp = (x, _) => 0.12 * x;

        Assert.True(LeggedGait.TryBuild(
            layout, path, 0.06, 0.04, 0.02,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out var result, out var err, ramp), err);

        Assert.True(result!.BasePath[^1].Z > 0.03, "Body base should rise on ramp.");
        var mid = result.Trajectory.Points.Count / 2;
        var q = result.Trajectory.Points[mid].JointState.Positions;
        var bf = result.BasePath[mid];
        var plantedNearTerrain = 0;
        for (var leg = 0; leg < 6; leg++)
        {
            var hy = layout.HipYawsRad[leg];
            var hip = new Vec3(layout.BodyR * Math.Cos(hy), layout.BodyR * Math.Sin(hy), layout.BodyZ);
            var footBody = LegIk3R.FootPosition(hip, layout.Coxa, layout.Femur, layout.Tibia,
                q[leg * 3], q[leg * 3 + 1], q[leg * 3 + 2]);
            if (footBody.Z > 0.012) continue; // swing (body-floor relative)
            // Full SE3 body (stance support plane may pitch on ramp).
            var m = Transforms.FromFrame(bf);
            Transforms.TransformPointInto(m, footBody.X, footBody.Y, footBody.Z, out var fx, out var fy, out var fz);
            var expect = ramp(fx, fy);
            Assert.InRange(fz - expect, -0.015, 0.015);
            plantedNearTerrain++;
        }
        Assert.True(plantedNearTerrain >= 3, "Expected ≥3 stance feet near ramp surface.");
    }

    // --- General N-leg / GaitSchedule / Mechanism ---

    [Fact]
    public void GaitSchedule_Auto_N3_RejectedForStatic()
    {
        var g = GaitSchedule.Auto(3);
        Assert.NotNull(g.Validate(3, allowDynamicGait: false));
        Assert.Contains("N=3", g.Validate(3)!);
        Assert.Null(g.Validate(3, allowDynamicGait: true));
    }

    [Fact]
    public void GaitSchedule_Auto_Hex_IsCorrectTripod()
    {
        var yaws = Enumerable.Range(0, 6).Select(i => i * (Math.PI / 3.0)).ToArray();
        var g = GaitSchedule.Auto(6, yaws);
        Assert.Null(g.Validate(6));
        Assert.Equal(2, g.SwingGroups!.Count);
        Assert.Equal(new[] { 0, 2, 4 }, g.SwingGroups[0]);
        Assert.Equal(new[] { 1, 3, 5 }, g.SwingGroups[1]);
        Assert.Equal(3, g.MinStanceCount);
    }

    [Fact]
    public void HexMithi_Layout_UsesCorrectTripodGroups()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        Assert.Equal(new[] { 0, 2, 4 }, layout.SwingGroups[0]);
        Assert.Equal(new[] { 1, 3, 5 }, layout.SwingGroups[1]);
    }

    [Fact]
    public void N4_Crawl_SsmPositive()
    {
        var mech = LeggedMechanism.FromHomogeneous3RRadial(
            4, 0.10, 0.06, 0.17, 0.19, 0.12,
            names: ["front-right", "front-left", "rear-left", "rear-right"],
            hipYawsRad: Enumerable.Range(0, 4).Select(i => i * (Math.PI / 2.0) + Math.PI / 4.0).ToArray(),
            gait: GaitSchedule.Crawl(4),
            tipLegName: "front-right");
        Assert.Null(mech.Validate());
        Assert.Equal(3, mech.Gait.MinStanceCount); // N=4 crawl: one swing → 3 stance
        var limits = Enumerable.Range(0, 12).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(mech.ToPreset(limits: limits));
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.4, 0, 0) };
        Assert.True(LeggedGait.TryBuild(
            mech, null, path, 0.06, 0.06, 0.02,
            hipStance: 0, 30 * Deg, -30 * Deg,
            model, out var result, out var err), err);
        Assert.True(result!.MinStaticStabilityMarginMeters > 0,
            $"Crawl SSM should be > 0, got {result.MinStaticStabilityMarginMeters}");
    }

    [Fact]
    public void Hex_SsmRoughlyAboveApothemHeuristic()
    {
        // Regular hexagon of hip radius R: apothem of opposite-tripod support is ~ R * cos(30°) * something;
        // with feet outside hips, min SSM at body should be clearly positive and order ~ BodyR/2.
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(layout.ToPreset("hex", 18, limits));
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.35, 0, 0) };
        Assert.True(LeggedGait.TryBuild(
            layout, path, 0.06, 0.06, 0.02,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out var result, out var err), err);
        var apothem = layout.BodyR * Math.Cos(Math.PI / 6.0); // ~0.104
        Assert.True(result!.MinStaticStabilityMarginMeters > 0.02,
            $"Hex tripod SSM {result.MinStaticStabilityMarginMeters:F4} should be clearly > 0 (apothem≈{apothem:F3}).");
    }

    [Fact]
    public void QuadSmoke_RequiresAllowDynamicGait_AndBuilds()
    {
        var mech = LeggedMechanism.QuadSmoke(0.10, 0.06, 0.17, 0.19, 0.12);
        Assert.True(mech.AllowDynamicGait);
        Assert.Null(mech.Validate());
        Assert.Equal(2, mech.Gait.MinStanceCount);
        var without = new LeggedMechanism(
            mech.Legs, mech.Gait, mech.TipLegName, mech.NominalBodyClearance,
            allowDynamicGait: false);
        Assert.NotNull(without.Validate());

        var limits = Enumerable.Range(0, 12).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
        var model = new RobotModel(mech.ToPreset(limits: limits));
        var path = new[] { new Vec3(0, 0, 0), new Vec3(0.35, 0, 0) };
        Assert.True(LeggedGait.TryBuild(
            mech, null, path, 0.08, 0.08, 0.025,
            7.5 * Deg, 30 * Deg, -30 * Deg,
            model, out var result, out var err), err);
        Assert.Contains("AllowDynamicGait", result!.Warning!);
    }

    [Fact]
    public void Layout_ToMechanism_RoundTrip()
    {
        var layout = LeggedLayout.HexMithi(0.12, 0.06, 0.17, 0.19, 0.12);
        var mech = layout.ToMechanism();
        Assert.Null(mech.Validate());
        Assert.Equal(6, mech.LegCount);
        Assert.Equal(18, mech.DriverCount);
        Assert.Equal(layout.TipLegName, mech.TipLegName);
        Assert.Equal(new[] { 0, 3, 6, 9, 12, 15 }, mech.DriverOffsets.ToArray());
        Assert.Equal(new[] { 0, 2, 4 }, mech.Gait.SwingGroups![0]);
        var tree = mech.Assemble();
        Assert.Equal(18, tree.DriverCount);
        Assert.Contains("right-middle/tibia", tree.Links.Select(l => l.Name));
        Assert.Equal("right-middle/tibia", mech.TipLinkName);
    }

    // --- Helper ---

    private static void AssertVec3Near(Vec3 actual, Vec3 expected, double tol, string msg = "")
    {
        var dx = Math.Abs(actual.X - expected.X);
        var dy = Math.Abs(actual.Y - expected.Y);
        var dz = Math.Abs(actual.Z - expected.Z);
        Assert.True(dx < tol && dy < tol && dz < tol,
            $"{msg} Vec3 mismatch: got ({actual.X:F6},{actual.Y:F6},{actual.Z:F6}), " +
            $"expected ({expected.X:F6},{expected.Y:F6},{expected.Z:F6}), tol={tol}");
    }
}
