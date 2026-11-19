using Motus.Construction;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class Handler9KinematicsTests
{
    private static readonly string Urdf = Path.GetFullPath(Path.Combine(
        AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", "handler9", "handler9.urdf"));

    private static Handler9Kinematics Load() => new(UrdfRobotLoader.LoadTree(Urdf));

    [Fact]
    public void Fixture_CadMeshesExist_AndCollisionBoxesLoad()
    {
        var doc = System.Xml.Linq.XDocument.Load(Urdf);
        var meshes = doc.Descendants("mesh").Select(m => m.Attribute("filename")!.Value).ToList();
        Assert.Equal(12, meshes.Count);
        Assert.All(meshes, f => Assert.True(File.Exists(Path.Combine(Path.GetDirectoryName(Urdf)!, f)), f));
        var collision = UrdfCollisionLoader.LoadTree(doc.Root!, Load().Tree, Path.GetDirectoryName(Urdf)!);
        Assert.NotNull(collision);
    }

    private static double[] RandomQ(Handler9Kinematics k, Random rng) =>
        k.Tree.DriverJointIndices.Select(i => k.Tree.Joints[i])
            .Select(j => j.Lower + rng.NextDouble() * (j.Upper - j.Lower)).ToArray();

    private static double[] P(double[] f) => new[] { f[3], f[7], f[11] };
    private static double[] X(double[] f) => new[] { f[0], f[4], f[8] };

    private static void AssertOnLine(double[] f, double[] p, double[] a)
    {
        double dx = f[3] - p[0], dy = f[7] - p[1], dz = f[11] - p[2];
        var n = Math.Sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        double ux = a[0] / n, uy = a[1] / n, uz = a[2] / n;
        // |d x u| = distance to the line; |x x u| = axis misalignment
        Assert.True(Norm(dy * uz - dz * uy, dz * ux - dx * uz, dx * uy - dy * ux) < 1e-9);
        Assert.True(Norm(f[4] * uz - f[8] * uy, f[8] * ux - f[0] * uz, f[0] * uy - f[4] * ux) < 1e-9);
    }

    private static double Norm(double x, double y, double z) => Math.Sqrt(x * x + y * y + z * z);

    [Fact]
    public void Fixture_HasTenPlannedJointsWithLimits()
    {
        var k = Load();
        Assert.Equal(10, k.Tree.DriverCount);
        var roll = k.Tree.Joints.Single(j => j.Name == "main_clamp_blue_roll");
        Assert.Equal(-40 * Math.PI / 180, roll.Lower, 6);
        Assert.Equal(40 * Math.PI / 180, roll.Upper, 6);
        foreach (var s in new[] { "handler_1", "main_clamp", "handler_2" })
        {
            var yaw = k.Tree.Joints.Single(j => j.Name == $"{s}_yaw");
            Assert.Equal(-Math.PI, yaw.Lower, 6);
            Assert.Equal(Math.PI, yaw.Upper, 6);
            Assert.Equal(KinematicJointType.Prismatic, k.Tree.Joints.Single(j => j.Name == $"{s}_lift").Type);
        }
    }

    [Fact]
    public void FkIk_RoundTrip_RandomInLimitConfigurations()
    {
        var k = Load();
        var rng = new Random(9);
        for (var i = 0; i < 200; i++)
        {
            var q = RandomQ(k, rng);
            var seed = RandomQ(k, rng);
            foreach (var clamp in Handler9Kinematics.ClampLinks)
            {
                var f = k.ClampFrame(q, clamp);
                Assert.True(k.TrySolveClamp(clamp, P(f), X(f), seed, out var sol, out var why), why);
                var g = k.ClampFrame(sol, clamp);
                for (var r = 0; r < 3; r++)
                    Assert.Equal(f[4 * r + 3], g[4 * r + 3], 9);
                AssertOnLine(g, P(f), X(f));
                Assert.All(k.Tree.DriverJointIndices, ji =>
                {
                    var j = k.Tree.Joints[ji];
                    Assert.InRange(sol[j.DriverIndex], j.Lower - 1e-12, j.Upper + 1e-12);
                });
            }
        }
    }

    [Fact]
    public void Limits_LiftAndSlideOutOfRange_FailWithReason()
    {
        var k = Load();
        var zero = new double[k.Tree.DriverCount];
        var f = k.ClampFrame(zero, "main_clamp_blue");

        var high = P(k.ClampFrame(zero, "handler_1_clamp")); high[2] += 1.0;
        Assert.False(k.TrySolveClamp("handler_1_clamp", high, X(f), zero, out _, out var why));
        Assert.Contains("lift", why);

        var far = P(f); far[0] += 1.0;
        Assert.False(k.TrySolveClamp("main_clamp_blue", far, X(f), zero, out _, out why));
        Assert.Contains("slide", why);
    }

    [Fact]
    public void Unreachable_FailsCleanly()
    {
        var k = Load();
        var zero = new double[k.Tree.DriverCount];
        var f = k.ClampFrame(zero, "handler_1_clamp");

        var side = P(f); side[1] += 0.1;
        Assert.False(k.TrySolveClamp("handler_1_clamp", side, X(f), zero, out var q, out var why));
        Assert.Contains("off the station", why);
        Assert.Equal(zero, q);

        Assert.False(k.TrySolveClamp("handler_1_clamp", P(f), new[] { 1.0, 0, 0.2 }, zero, out _, out why));
        Assert.Contains("horizontal", why);
        Assert.False(k.TrySolveClamp("handler_1_clamp", new[] { double.NaN, 0, 0 }, X(f), zero, out _, out _));
        Assert.False(k.TrySolveClamp("not_a_clamp", P(f), X(f), zero, out _, out _));
        Assert.False(k.TrySolveCane(P(f), new[] { 0.0, 0, 0 }, zero, out _, out _));
    }

    [Theory]
    [InlineData(1.0, 0.0)]
    [InlineData(1.0, 0.07)]
    [InlineData(-1.0, 0.085)]
    public void Cane_AllStationsHoldOneStraightCane(double dirX, double raise)
    {
        var k = Load();
        var rng = new Random(3);
        var seed = RandomQ(k, rng);
        seed[k.Slot("main_clamp_lift")] = 0;
        var z0 = k.ClampFrame(new double[k.Tree.DriverCount], "handler_1_clamp")[11];
        var p = new[] { 0.3, 0, z0 + raise };
        var a = new[] { dirX, 0, 0 };

        Assert.True(k.TrySolveCane(p, a, seed, out var q, out var why), why);
        foreach (var clamp in Handler9Kinematics.ClampLinks)
            AssertOnLine(k.ClampFrame(q, clamp), p, a);
        foreach (var j in new[] { "handler_1_feed", "main_clamp_blue_slide", "handler_2_feed" })
            Assert.Equal(seed[k.Slot(j)], q[k.Slot(j)]);
    }

    [Fact]
    public void Cane_OffBeamTiltedOrTooHigh_Fails()
    {
        var k = Load();
        var zero = new double[k.Tree.DriverCount];
        var z0 = k.ClampFrame(zero, "handler_1_clamp")[11];

        Assert.False(k.TrySolveCane(new[] { 0, 0.05, z0 }, new[] { 1.0, 0, 0 }, zero, out _, out _));
        Assert.False(k.TrySolveCane(new[] { 0, 0, z0 }, new[] { Math.Cos(0.5), Math.Sin(0.5), 0 }, zero, out _, out _));
        Assert.False(k.TrySolveCane(new[] { 0, 0, z0 }, new[] { 1.0, 0, 0.1 }, zero, out _, out _));
        Assert.False(k.TrySolveCane(new[] { 0, 0, z0 + 1.0 }, new[] { 1.0, 0, 0 }, zero, out _, out var why));
        Assert.Contains("lift", why);
    }

    [Theory]
    [InlineData(0.0, 0.07)]
    [InlineData(0.015, 0.07)]
    [InlineData(-0.015, 0.07)]
    public void Rod_OuterClampsAlone_HoldATiltedRodInTheBeamPlane(double tilt, double raise)
    {
        var k = Load();
        var seed = RandomQ(k, new Random(5));
        var z0 = k.ClampFrame(new double[k.Tree.DriverCount], "handler_1_clamp")[11];
        var p = new[] { 0.4, 0, z0 + raise };
        var a = new[] { Math.Cos(tilt), 0, Math.Sin(tilt) };

        Assert.True(k.TrySolveRod(p, a, seed, out var q, out var why), why);
        var f1 = k.ClampFrame(q, "handler_1_clamp");
        var f2 = k.ClampFrame(q, "handler_2_clamp");
        foreach (var f in new[] { f1, f2 })
        {
            double dx = f[3] - p[0], dy = f[7] - p[1], dz = f[11] - p[2];
            Assert.True(Norm(dy * a[2] - dz * a[1], dz * a[0] - dx * a[2], dx * a[1] - dy * a[0]) < 1e-9);
            Assert.True(Math.Abs(f[0] * a[1] - f[4] * a[0]) < 1e-9); // clamp +x = rod heading
        }
        // Closed chain: fixed horizontal spacing, so the clamp distance along the rod is spacing / cos(tilt).
        var span = Norm(f2[3] - f1[3], f2[7] - f1[7], f2[11] - f1[11]);
        Assert.Equal(Math.Abs(f2[3] - f1[3]) / Math.Cos(tilt), span, 9);
        for (var i = 0; i < k.Tree.DriverCount; i++)
        {
            var name = k.Tree.Joints[k.Tree.DriverJointIndices[i]].Name;
            if (!name.EndsWith("_yaw", StringComparison.Ordinal) && !name.EndsWith("_lift", StringComparison.Ordinal) || name.StartsWith("main", StringComparison.Ordinal))
                Assert.Equal(seed[i], q[i]);
        }
    }

    [Fact]
    public void Rod_OutOfPlaneSteepOrVertical_Fails()
    {
        var k = Load();
        var zero = new double[k.Tree.DriverCount];
        var z0 = k.ClampFrame(zero, "handler_1_clamp")[11];

        Assert.False(k.TrySolveRod(new[] { 0, 0.05, z0 }, new[] { 1.0, 0, 0 }, zero, out _, out _));
        Assert.False(k.TrySolveRod(new[] { 0, 0, z0 }, new[] { Math.Cos(0.3), Math.Sin(0.3), 0 }, zero, out _, out _));
        Assert.False(k.TrySolveRod(new[] { 0, 0, z0 + 0.2 }, new[] { Math.Cos(0.3), 0, Math.Sin(0.3) }, zero, out _, out var why));
        Assert.Contains("lift", why);
        Assert.False(k.TrySolveRod(new[] { 0, 0, z0 }, new[] { 0, 0, 1.0 }, zero, out _, out why));
        Assert.Contains("vertical", why);
    }

    [Fact]
    public void PassiveYellowRoll_FollowsTheActiveBlueRoll()
    {
        var k = Load();
        var yellow = k.Tree.Joints.Single(j => j.Name == "main_clamp_yellow_roll");
        Assert.Equal(-1, yellow.DriverIndex);
        Assert.Equal("main_clamp_blue_roll", k.Tree.Joints[yellow.Mimic!.Value.JointIndex].Name);

        var q = new double[k.Tree.DriverCount];
        q[k.Slot("main_clamp_blue_roll")] = 0.5;
        var b = k.ClampFrame(q, "main_clamp_blue");
        var y = k.ClampFrame(q, "main_clamp_yellow");
        // Same rotation about the shared cane axis: equal rotation blocks, and both turned by 0.5 rad.
        foreach (var i in new[] { 0, 1, 2, 4, 5, 6, 8, 9, 10 })
            Assert.Equal(b[i], y[i], 12);
        Assert.Equal(0.5, Math.Atan2(y[9], y[5]), 12);
    }

    [Fact]
    public void PassiveYellowSlide_MirrorsTheActiveBlueSlide()
    {
        var k = Load();
        var yellow = k.Tree.Joints.Single(j => j.Name == "main_clamp_yellow_slide");
        Assert.Equal(-1, yellow.DriverIndex);
        Assert.Equal("main_clamp_blue_slide", k.Tree.Joints[yellow.Mimic!.Value.JointIndex].Name);
        Assert.Equal(-1, yellow.Mimic!.Value.Multiplier);
        Assert.Equal(0, yellow.Mimic!.Value.Offset);

        var q = new double[k.Tree.DriverCount];
        q[k.Slot("main_clamp_blue_slide")] = 0.05;
        var zero = new double[k.Tree.DriverCount];
        var b0 = k.ClampFrame(zero, "main_clamp_blue");
        var y0 = k.ClampFrame(zero, "main_clamp_yellow");
        var b = k.ClampFrame(q, "main_clamp_blue");
        var y = k.ClampFrame(q, "main_clamp_yellow");
        // Both axes are +x: blue 0..+110 mm, yellow -110..0 mm. yellow = -blue moves both toward the centre.
        Assert.Equal(b0[3] + 0.05, b[3], 12);
        Assert.Equal(y0[3] - 0.05, y[3], 12);
    }
}
