using Motus.Core;
using Motus.Geometry;

namespace Motus.Core.Tests;

/// <summary>
/// Capsules are tested exactly (axis-segment distance vs radius). The old sphere-chain stand-in lies inside the true
/// capsule and missed grazing contact between two chain spheres.
/// </summary>
public class CapsuleExactTests
{
    private static readonly Dictionary<int, BvhNode> Cache = new();
    // r = 0.1, half-length 0.1 → legacy chain spacing 0.025; z = -0.0875 is midway between two chain spheres.
    private static CollisionObject Cap(string n = "cap") => CollisionObject.Capsule(n, Frame.Identity, 0.1, 0.1);

    private static bool LegacyChain(CollisionObject cap, CollisionObject other) =>
        CollisionGeometry.SampleCapsule(cap, ExactNarrowPhase.DenseCapsuleSamples(cap.ExtentX, cap.ExtentY))
            .Any(s => CollisionGeometry.Intersects(CollisionObject.Sphere("_", s.center, s.radius), other, Cache));

    [Fact]
    public void GrazingSphere_BetweenChainSpheres_IsHit_LegacyMissedIt()
    {
        var graze = CollisionObject.Sphere("s", new Frame(0.1005, 0, -0.0875), 0.001); // 0.5 mm inside the true capsule
        Assert.False(LegacyChain(Cap(), graze));
        Assert.True(CollisionGeometry.Intersects(Cap(), graze, Cache));
        Assert.True(CollisionGeometry.Intersects(graze, Cap(), Cache)); // obstacle-side capsule too
        var clear = CollisionObject.Sphere("c", new Frame(0.1015, 0, -0.0875), 0.001);
        Assert.False(CollisionGeometry.Intersects(Cap(), clear, Cache));
    }

    private static CollisionObject Tri(double x) => CollisionObject.Mesh("tri", Frame.Identity,
        new List<double[]> { new[] { x, -0.0005, -0.088 }, new[] { x, 0.0005, -0.088 }, new[] { x, 0, -0.087 } },
        new List<int> { 0, 1, 2 });

    [Fact]
    public void GrazingTriangle_Mesh_IsHit_LegacyMissedIt()
    {
        var tri = Tri(0.0995);
        Assert.False(LegacyChain(Cap(), tri));
        Assert.True(CollisionGeometry.Intersects(Cap(), tri, Cache));   // capsule link vs mesh obstacle
        Assert.True(CollisionGeometry.Intersects(tri, Cap(), Cache));   // mesh link vs capsule obstacle
        Assert.False(CollisionGeometry.Intersects(Cap(), Tri(0.1005), Cache));
        Assert.False(CollisionGeometry.Intersects(Tri(0.1005), Cap(), Cache));
    }

    [Fact]
    public void GrazingBoxEdge_IsHit_LegacyMissedIt()
    {
        // Box corner region near the capsule surface between chain spheres.
        var box = CollisionObject.Box("b", new Frame(0.0995 + 0.01, 0, -0.0875), 0.01, 0.0005, 0.0005);
        Assert.False(LegacyChain(Cap(), box));
        Assert.True(CollisionGeometry.Intersects(Cap(), box, Cache));
        Assert.True(CollisionGeometry.Intersects(box, Cap(), Cache));
        var far = CollisionObject.Box("f", new Frame(0.1005 + 0.01, 0, -0.0875), 0.01, 0.0005, 0.0005);
        Assert.False(CollisionGeometry.Intersects(Cap(), far, Cache));
    }

    [Fact]
    public void CapsuleCapsule_SegmentDistance()
    {
        var other = CollisionObject.Capsule("o", new Frame(0.2 - 0.0005, 0, -0.0875, Math.Sqrt(0.5), Math.Sqrt(0.5), 0, 0), 0.1, 0.0005);
        Assert.True(CollisionGeometry.Intersects(Cap(), other, Cache));
        var apart = CollisionObject.Capsule("o", new Frame(0.2 + 0.0005, 0, -0.0875, Math.Sqrt(0.5), Math.Sqrt(0.5), 0, 0), 0.1, 0.0005);
        Assert.False(CollisionGeometry.Intersects(Cap(), apart, Cache));
    }

    [Fact]
    public void SegmentTriangle_Piercing_IsZero()
    {
        Assert.Equal(0, ExactNarrowPhase.SegmentTriangleDistanceSq(0.2, 0.2, -1, 0.2, 0.2, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0));
        Assert.Equal(1.0, ExactNarrowPhase.SegmentTriangleDistanceSq(0.2, 0.2, 1, 0.2, 0.2, 2, 0, 0, 0, 1, 0, 0, 0, 1, 0), 12);
    }
}
