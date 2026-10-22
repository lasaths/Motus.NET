using Motus.Core;
using Motus.Geometry;

namespace Motus.Core.Tests;

/// <summary>Exact narrow phase: no more sphere-cloud over-reporting for boxes and meshes.</summary>
public class ExactNarrowPhaseTests
{
    private static readonly Dictionary<int, BvhNode> Cache = new();

    [Fact]
    public void CapsuleContactsBetweenOldSphereSamplesAreNeverMissed()
    {
        // Previous 400-sample cap leaves 5 mm gaps along this 2 m axis despite a 1 mm radius.
        var capsule = CollisionObject.Capsule("long", Frame.Identity, 0.001, 1);
        var sphere = CollisionObject.Sphere("sphere", new Frame(0.001, 0, 0.0025), 0.00005);
        var box = CollisionObject.Box("box", new Frame(0.0009, 0, 0.0025), 0.00005, 0.00005, 0.00005);
        var shortCapsule = CollisionObject.Capsule("short", new Frame(0.001, 0, 0.0025), 0.00005, 0.00005);
        var triangle = CollisionObject.Mesh("triangle", Frame.Identity,
            new List<double[]> { new[] { 0.0009, -0.00005, 0.00245 }, new[] { 0.0009, 0.00005, 0.00245 }, new[] { 0.0009, 0.0, 0.00255 } },
            new List<int> { 0, 1, 2 });
        foreach (var other in new[] { sphere, box, shortCapsule, triangle })
        {
            Assert.True(CollisionGeometry.Intersects(capsule, other, Cache), other.Name);
            Assert.True(CollisionGeometry.Intersects(other, capsule, Cache), other.Name);
        }
        var clear = CollisionObject.Sphere("clear", new Frame(0.0011, 0, 0.0025), 0.00005);
        Assert.False(CollisionGeometry.Intersects(capsule, clear, Cache));
    }

    [Fact]
    public void CapsuleContactInSmallGapBetweenDenseUncappedSamplesIsDetected()
    {
        // Even samples spaced r/4 have a concave gap on the true capsule's cylindrical side.
        var capsule = CollisionObject.Capsule("capsule", Frame.Identity, 0.01, 0.02);
        var sphere = CollisionObject.Sphere("sphere", new Frame(0.010009, 0, 0.00125), 0.00001);
        Assert.True(CollisionGeometry.Intersects(capsule, sphere, Cache));
    }

    [Fact]
    public void SegmentDistancesCoverInteriorClosestPointsAndDegenerateTriangles()
    {
        Assert.Equal(1, ExactNarrowPhase.SegmentAabbDistanceSq(new Frame(-2, 2, 0), new Frame(2, 2, 0), 1, 1, 1), 12);
        Assert.Equal(0, ExactNarrowPhase.SegmentAabbDistanceSq(new Frame(-2, -2, 0), new Frame(2, 2, 0), 1, 1, 1), 12);
        Assert.Equal(2, ExactNarrowPhase.SegmentAabbDistanceSq(new Frame(2, 2, 0), new Frame(3, 3, 0), 1, 1, 1), 12);
        Assert.Equal(1, ExactNarrowPhase.SegmentSegmentDistanceSq(new Frame(-2, 0, 0), new Frame(2, 0, 0), new Frame(0, -2, 1), new Frame(0, 2, 1)), 12);
        Assert.Equal(0, ExactNarrowPhase.SegmentTriangleDistanceSq(new Frame(0, 0, -1), new Frame(0, 0, 1),
            new Frame(-1, -1, 0), new Frame(1, -1, 0), new Frame(0, 1, 0)), 12);
        Assert.Equal(1, ExactNarrowPhase.PointTriangleDistanceSq(0, 1, 0, -1, 0, 0, -1, 0, 0, 1, 0, 0), 12);
    }

    [Fact]
    public void ClosedMeshRemainsATriangleSurfaceRatherThanImplicitSolidVolume()
    {
        var v = new List<double[]>();
        foreach (var x in new[] { -1.0, 1.0 }) foreach (var y in new[] { -1.0, 1.0 }) foreach (var z in new[] { -1.0, 1.0 })
            v.Add(new[] { x, y, z });
        var idx = new List<int> { 0,1,3, 0,3,2, 4,6,7, 4,7,5, 0,4,5, 0,5,1, 2,3,7, 2,7,6, 0,2,6, 0,6,4, 1,5,7, 1,7,3 };
        var surface = CollisionObject.Mesh("surface", Frame.Identity, v, idx);
        var box = CollisionObject.Box("inside", Frame.Identity, 0.1, 0.1, 0.1);
        Assert.False(CollisionGeometry.Intersects(surface, box, Cache));
        Assert.False(CollisionGeometry.Intersects(box, surface, Cache));
        var capsule = CollisionObject.Capsule("inside", Frame.Identity, 0.1, 0.1);
        Assert.False(CollisionGeometry.Intersects(surface, capsule, Cache));
        Assert.False(CollisionGeometry.Intersects(capsule, surface, Cache));
    }

    [Fact]
    public void BoxBox_SeparatedByOneMillimetre_IsClear_TouchingOverlapIsHit()
    {
        // Long thin tube (the old corner/centre-sphere test flagged anything within ~36 mm).
        var tube = CollisionObject.Box("tube", new Frame(0, 0, 0), 0.006, 0.036, 0.008);
        var near = CollisionObject.Box("near", new Frame(0.006 + 0.001 + 0.01, 0, 0), 0.01, 0.01, 0.01);
        var hit = CollisionObject.Box("hit", new Frame(0.006 - 0.0001 + 0.01, 0, 0), 0.01, 0.01, 0.01);
        Assert.False(CollisionGeometry.Intersects(tube, near, Cache));
        Assert.True(CollisionGeometry.Intersects(tube, hit, Cache));
    }

    [Fact]
    public void BoxBox_RotatedEdgeCase_UsesCrossAxes()
    {
        // Two boxes rotated 45° about different axes, near but separated only along an edge-edge axis.
        var s = Math.Sqrt(0.5);
        var a = CollisionObject.Box("a", new Frame(0, 0, 0, s, 0, 0, s), 0.01, 0.01, 0.01);   // 90° about Z (still axis-aligned)
        var b = CollisionObject.Box("b", new Frame(0.0300, 0, 0, Math.Cos(Math.PI / 8), Math.Sin(Math.PI / 8), 0, 0), 0.01, 0.01, 0.01);
        Assert.False(CollisionGeometry.Intersects(a, b, Cache));
        var c = CollisionObject.Box("c", new Frame(0.0195, 0, 0, Math.Cos(Math.PI / 8), Math.Sin(Math.PI / 8), 0, 0), 0.01, 0.01, 0.01);
        Assert.True(CollisionGeometry.Intersects(a, c, Cache));
    }

    [Fact]
    public void MeshVsBoxAndPlane_ExactPerTriangle()
    {
        // 2 cm cube mesh.
        var v = new List<double[]>();
        foreach (var x in new[] { -0.01, 0.01 }) foreach (var y in new[] { -0.01, 0.01 }) foreach (var z in new[] { -0.01, 0.01 })
            v.Add(new[] { x, y, z });
        var idx = new List<int> { 0,1,3, 0,3,2, 4,6,7, 4,7,5, 0,4,5, 0,5,1, 2,3,7, 2,7,6, 0,2,6, 0,6,4, 1,5,7, 1,7,3 };
        var meshClear = CollisionObject.Mesh("m", new Frame(0, 0, 0.0105), v, idx);   // 0.5 mm above floor
        var meshHit = CollisionObject.Mesh("m2", new Frame(0, 0, 0.0095), v, idx);    // 0.5 mm into floor
        var floorBox = CollisionObject.Box("floor", new Frame(0, 0, -0.025), 1, 1, 0.025);
        var floorPlane = CollisionObject.Plane("plane", new Frame(0, 0, 0, Math.Cos(-Math.PI / 4), 0, Math.Sin(-Math.PI / 4), 0));
        Assert.False(CollisionGeometry.Intersects(meshClear, floorBox, Cache));
        Assert.True(CollisionGeometry.Intersects(meshHit, floorBox, Cache));
        Assert.False(CollisionGeometry.Intersects(meshClear, floorPlane, Cache));
        Assert.True(CollisionGeometry.Intersects(meshHit, floorPlane, Cache));

        // Box vs mesh obstacle (BVH + triangle–OBB SAT).
        var boxClear = CollisionObject.Box("b", new Frame(0.0215, 0, 0.0105), 0.001, 0.001, 0.001);
        var boxHit = CollisionObject.Box("b2", new Frame(0.0105, 0, 0.0105), 0.001, 0.001, 0.001);
        Assert.False(CollisionGeometry.Intersects(boxClear, meshClear, Cache));
        Assert.True(CollisionGeometry.Intersects(boxHit, meshClear, Cache));
    }

    [Fact]
    public void FindContacts_NamesEveryPair_AndHonoursGeometryLevelAllowedPairs()
    {
        var bamboo = Motus.Construction.BambooIcdCell.LoadFile(Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory, "..", "..", "..", "..", "..", "tests", "fixtures", "bamboo_icd", "bamboo_icd.urdf")));
        var checker = bamboo.Checker(40, null);
        var q = new JointState(Motus.Construction.BambooIcdCell.RadFromDeg(new double[] { -90, 0, 0, 0, 0 }));
        var ground = CollisionObject.Box("ground", new Frame(0, 0, -0.025), 1, 1, 0.025);
        var contacts = checker.FindContacts(q, new CollisionScene(new[] { ground }));
        Assert.NotEmpty(contacts);
        Assert.All(contacts, c => Assert.Equal("ground", c.BodyB));
        Assert.Contains(contacts, c => c.BodyA == "R-grip jaw-" && c.LinkA == "link_5");
        Assert.False(checker.IsCollisionFree(q, new CollisionScene(new[] { ground })));

        // Allow exactly the touching parts by geometry name → clear; links are still checked otherwise.
        var allowed = contacts.Select(c => (c.BodyA, c.BodyB)).ToList();
        Assert.True(checker.IsCollisionFree(q, new CollisionScene(new[] { ground }, allowed)));
    }
}
