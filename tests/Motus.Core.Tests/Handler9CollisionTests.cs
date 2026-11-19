using Motus.Construction;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class Handler9CollisionTests
{
    private static readonly string Urdf = Path.GetFullPath(Path.Combine(
        AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", "handler9", "handler9.urdf"));

    private static (Handler9Kinematics k, Handler9Collision c) Load(double? clearance = null)
    {
        var k = new Handler9Kinematics(UrdfRobotLoader.LoadTree(Urdf));
        return (k, new Handler9Collision(k.Tree, Urdf, clearance));
    }

    [Fact]
    public void AllowedAdjacentPairs_ComeFromTheUrdfTree_WalkingThroughEmptyTurntables()
    {
        var (_, c) = Load();
        var got = c.AllowedAdjacentPairs
            .Select(p => string.CompareOrdinal(p.A, p.B) <= 0 ? (p.A, p.B) : (p.B, p.A))
            .OrderBy(p => p.Item1, StringComparer.Ordinal).ThenBy(p => p.Item2, StringComparer.Ordinal)
            .ToArray();
        string[][] expected =
        [
            ["base_link", "handler_1_carriage"],
            ["base_link", "handler_2_carriage"],
            ["base_link", "main_clamp_carriage"],
            ["handler_1_active", "handler_1_carriage"],
            ["handler_1_active", "handler_1_clamp"],
            ["handler_1_carriage", "handler_1_clamp"],
            ["handler_2_active", "handler_2_carriage"],
            ["handler_2_active", "handler_2_clamp"],
            ["handler_2_carriage", "handler_2_clamp"],
            ["main_clamp_blue", "main_clamp_blue_sled"],
            ["main_clamp_blue_sled", "main_clamp_carriage"],
            ["main_clamp_carriage", "main_clamp_yellow_sled"],
            ["main_clamp_yellow", "main_clamp_yellow_sled"],
        ];
        Assert.Equal(expected.Select(p => (p[0], p[1])), got);
    }

    [Fact]
    public void HomePose_IsSelfCollisionFree()
    {
        var (k, c) = Load();
        Assert.Equal(0.002, c.ClearanceMarginM);
        Assert.True(c.IsSelfCollisionFree(new double[k.Tree.DriverCount], out var hit), hit);
    }

    [Fact]
    public void FullInwardSlides_MainClampGrippersMeet_IsDetected()
    {
        var (k, c) = Load();
        var q = new double[k.Tree.DriverCount];
        q[k.Slot("main_clamp_blue_slide")] = 0.110; // yellow follows to -0.110; centres 120 mm apart
        Assert.False(c.IsSelfCollisionFree(q, out var hit), "expected the fully closed MainClamp slides to collide");
        Assert.Contains("main_clamp", hit);
    }

    [Theory]
    [InlineData(1.0, 0.0)]
    [InlineData(1.0, 0.07)]
    [InlineData(-1.0, 0.085)]
    public void CaneIkPose_SelfCollision(double dirX, double raise)
    {
        var (k, c) = Load();
        var seed = new double[k.Tree.DriverCount];
        var z0 = k.ClampFrame(seed, "handler_1_clamp")[11];
        var p = new[] { 0.3, 0, z0 + raise };
        var a = new[] { dirX, 0, 0 };
        Assert.True(k.TrySolveCane(p, a, seed, out var q, out var why), why);
        var cane = Handler9Collision.Cane(p, a, 1.0);
        var clear = c.IsSelfCollisionFree(q, out var hit, cane);
        Assert.True(clear, $"TrySolveCane(dirX={dirX}, raise={raise}) self-collides: {hit}");
    }

    [Theory]
    [InlineData(0.0, 0.07)]
    [InlineData(0.015, 0.07)]
    [InlineData(-0.015, 0.07)]
    public void RodIkPose_SelfCollision(double tilt, double raise)
    {
        var (k, c) = Load();
        var seed = new double[k.Tree.DriverCount];
        var z0 = k.ClampFrame(seed, "handler_1_clamp")[11];
        var p = new[] { 0.4, 0, z0 + raise };
        var a = new[] { Math.Cos(tilt), 0, Math.Sin(tilt) };
        Assert.True(k.TrySolveRod(p, a, seed, out var q, out var why), why);
        var cane = Handler9Collision.Cane(p, a, 1.0);
        var clear = c.IsSelfCollisionFree(q, out var hit, cane);
        Assert.True(clear, $"TrySolveRod(tilt={tilt}, raise={raise}) self-collides: {hit}");
    }

    [Fact]
    public void CaneAtHome_OnlyOuterContacts_HitsTheMiddle()
    {
        var (k, c) = Load();
        var z0 = k.ClampFrame(new double[k.Tree.DriverCount], "handler_1_clamp")[11];
        var cane = Handler9Collision.Cane(new[] { 0.0, 0, z0 }, new[] { 1.0, 0, 0 }, 1.0);
        var q = new double[k.Tree.DriverCount];
        Assert.True(c.IsSelfCollisionFree(q, out var hitAll, cane), hitAll);
        Assert.False(
            c.IsSelfCollisionFree(q, out var hit, cane, new[] { "handler_1_clamp", "handler_1_active" }),
            "the middle clamp sits on the home cane and must be reported when it is not an allowed contact");
        Assert.Contains("cane", hit);
    }
}
