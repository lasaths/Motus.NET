using Motus.Core;
using Motus.Geometry;

namespace Motus.Core.Tests;

/// <summary>Exact narrow phase: no more sphere-cloud over-reporting for boxes and meshes.</summary>
public class ExactNarrowPhaseTests
{
    private static readonly Dictionary<int, BvhNode> Cache = new();

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
        var bamboo = Motus.Viewer.Services.BambooIcdCell.LoadFile(Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory, "..", "..", "..", "..", "..", "tests", "fixtures", "bamboo_icd", "bamboo_icd.urdf")));
        var checker = bamboo.Checker(40, null);
        var q = new JointState(Motus.Viewer.Services.BambooIcdCell.RadFromDeg(new double[] { -90, 0, 0, 0, 0 }));
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
