using System.Xml.Linq;
using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>Tree link-to-link collision (G1) on the synthetic 3-link fixture tests/fixtures/tree3.</summary>
public class TreeSelfCollisionTests
{
    private static readonly string Urdf = Path.GetFullPath(Path.Combine(
        AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", "tree3", "tree3.urdf"));

    internal static (KinematicTree Tree, RobotModel Robot, IReadOnlyList<string> Names) Load()
    {
        var tree = UrdfRobotLoader.LoadTree(Urdf);
        var col = UrdfCollisionLoader.LoadTree(XDocument.Load(Urdf).Root!, tree, Path.GetDirectoryName(Urdf)!,
            source: UrdfMeshSource.Visual)!;
        var names = tree.DriverJointIndices.Select(i => tree.Joints[i].Name).ToArray();
        var limits = tree.DriverJointIndices.Select(i => new JointLimit(tree.Joints[i].Lower, tree.Joints[i].Upper)).ToArray();
        var robot = new RobotModel(new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = "tree3",
            Family = "tree3",
            AxisCount = tree.DriverCount,
            JointLimits = limits,
        }, col, names);
        return (tree, robot, names);
    }

    internal static TreeFkCollisionChecker Checker(double clearanceM, bool exactOnly = false, bool self = true)
    {
        var (tree, robot, names) = Load();
        var links = robot.CollisionModel!.Links.Select(l => l.LinkName).ToHashSet(StringComparer.Ordinal);
        return new TreeFkCollisionChecker(robot, tree, planJointNames: names,
            selfCollision: self ? new TreeSelfCollision(clearanceM, TreeAdjacency.AllowedPairs(tree, links, fixedSiblingsTouch: false)) : null)
        {
            ExactOnly = exactOnly
        };
    }

    [Fact]
    public void VisualMeshes_HonourScale_AndBareFileNameFallback()
    {
        var (tree, robot, _) = Load();
        var byLink = robot.CollisionModel!.Links.ToDictionary(l => l.LinkName, l => l.LocalGeometry);
        Assert.Equal(["base", "arm1", "arm2"], byLink.Keys);
        Assert.All(byLink.Values, g => Assert.Equal(CollisionShape.Mesh, g.Shape));
        Assert.Equal(0.3, byLink["arm1"].MeshAabbMax![0] - byLink["arm1"].MeshAabbMin![0], 9);
        Assert.Equal(0.25, byLink["arm2"].MeshAabbMax![0] - byLink["arm2"].MeshAabbMin![0], 9);
        Assert.Equal(0.04, byLink["arm2"].MeshAabbMax![1] - byLink["arm2"].MeshAabbMin![1], 9);

        var boxes = UrdfCollisionLoader.LoadTree(XDocument.Load(Urdf).Root!, tree, Path.GetDirectoryName(Urdf)!)!;
        Assert.Equal(CollisionShape.Box, Assert.Single(boxes.Links).LocalGeometry.Shape);
    }

    [Fact]
    public void MeshScale_NotPositive_FailsClosed()
    {
        var root = XDocument.Load(Urdf).Root!;
        root.Descendants("mesh").First().SetAttributeValue("scale", "0 0.001 0.001");
        var tree = UrdfRobotLoader.LoadTree(Urdf);
        Assert.Throws<InvalidDataException>(() =>
            UrdfCollisionLoader.LoadTree(root, tree, Path.GetDirectoryName(Urdf)!, source: UrdfMeshSource.Visual));
    }

    [Fact]
    public void Adjacency_WalksEmptyLinks_AndFixedSiblingsAreOptIn()
    {
        var (tree, _, _) = Load();
        Assert.Equal([("base", "arm1"), ("arm1", "arm2")],
            TreeAdjacency.AllowedPairs(tree, new HashSet<string> { "base", "arm1", "arm2" }, fixedSiblingsTouch: true));

        // p -> (empty frame) -> {fixed a, moving b, moving c}
        var doc = XDocument.Parse("""
            <robot name="sib">
              <link name="p"/><link name="frame"/><link name="a"/><link name="b"/><link name="c"/>
              <joint name="pf" type="revolute"><parent link="p"/><child link="frame"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1"/></joint>
              <joint name="fa" type="fixed"><parent link="frame"/><child link="a"/></joint>
              <joint name="fb" type="prismatic"><parent link="frame"/><child link="b"/><axis xyz="1 0 0"/><limit lower="0" upper="1" velocity="1"/></joint>
              <joint name="fc" type="prismatic"><parent link="frame"/><child link="c"/><axis xyz="1 0 0"/><limit lower="0" upper="1" velocity="1"/></joint>
            </robot>
            """);
        var sib = UrdfRobotLoader.LoadTree(doc);
        var geo = new HashSet<string> { "p", "a", "b", "c" };
        Assert.Equal([("p", "a"), ("p", "b"), ("p", "c")], TreeAdjacency.AllowedPairs(sib, geo, fixedSiblingsTouch: false));
        Assert.Equal([("p", "a"), ("p", "b"), ("p", "c"), ("a", "b"), ("a", "c")], TreeAdjacency.AllowedPairs(sib, geo, fixedSiblingsTouch: true));
    }

    [Fact]
    public void FoldedArm_HitsBase_OnlyWithSelfCollision()
    {
        var self = Checker(0);
        var folded = new JointState([0.0, Math.PI]);
        Assert.False(self.IsCollisionFree(folded, new CollisionScene()));
        Assert.Equal("arm2 vs base", self.LastHit);
        Assert.True(self.IsCollisionFree(new JointState([Math.PI / 2, Math.PI]), new CollisionScene()));
        Assert.Null(self.LastHit);
        Assert.True(self.IsCollisionFree(new JointState([0.0, 0.0]), new CollisionScene()));

        Assert.True(Checker(0, self: false).IsCollisionFree(folded, new CollisionScene()));
        // A scene (SRDF) allowed pair also skips the self pair.
        Assert.True(self.IsCollisionFree(folded, new CollisionScene(allowedPairs: [("base", "arm2")])));
    }

    [Fact]
    public void FactoryDefault_IsOn_AndAllowsDesignContacts()
    {
        // Two bases fixed to the world touch; two mimic fingers touch when closed. Both are design contacts.
        var doc = XDocument.Parse("""
            <robot name="cell">
              <link name="world"/>
              <link name="base_a"><collision><origin xyz="0 0 0.05"/><geometry><box size="0.2 0.2 0.1"/></geometry></collision></link>
              <link name="base_b"><collision><origin xyz="0 0 0.05"/><geometry><box size="0.2 0.2 0.1"/></geometry></collision></link>
              <link name="arm"><collision><origin xyz="0.2 0 0.025"/><geometry><box size="0.4 0.05 0.05"/></geometry></collision></link>
              <link name="hand"><collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision></link>
              <link name="finger_l"><collision><origin xyz="0 0.01 0"/><geometry><box size="0.04 0.02 0.02"/></geometry></collision></link>
              <link name="finger_r"><collision><origin xyz="0 -0.01 0"/><geometry><box size="0.04 0.02 0.02"/></geometry></collision></link>
              <joint name="wa" type="fixed"><parent link="world"/><child link="base_a"/></joint>
              <joint name="wb" type="fixed"><parent link="world"/><child link="base_b"/><origin xyz="0.2 0 0"/></joint>
              <joint name="j1" type="revolute"><parent link="base_a"/><child link="arm"/><origin xyz="0 0 0.12"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" velocity="1"/></joint>
              <joint name="jh" type="fixed"><parent link="arm"/><child link="hand"/><origin xyz="0.4 0 0.025"/></joint>
              <joint name="fl" type="prismatic"><parent link="hand"/><child link="finger_l"/><origin xyz="0.05 0 0"/><axis xyz="0 1 0"/><limit lower="0" upper="0.04" velocity="1"/></joint>
              <joint name="fr" type="prismatic"><parent link="hand"/><child link="finger_r"/><origin xyz="0.05 0 0"/><axis xyz="0 -1 0"/><limit lower="0" upper="0.04" velocity="1"/><mimic joint="fl"/></joint>
            </robot>
            """);
        var tree = UrdfRobotLoader.LoadTree(doc);
        var col = UrdfCollisionLoader.LoadTree(doc.Root!, tree, ".")!;
        var names = tree.DriverJointIndices.Select(i => tree.Joints[i].Name).ToArray();
        var robot = new RobotModel(new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown, ModelName = "cell", Family = "cell", AxisCount = tree.DriverCount,
            JointLimits = tree.DriverJointIndices.Select(i => new JointLimit(tree.Joints[i].Lower, tree.Joints[i].Upper)).ToArray(),
        }, col, names);
        var empty = new CollisionScene();
        var home = new JointState([0.0, 0.0]);
        var dipped = new JointState([0.6, 0.0]); // arm swings down into base_b

        var factory = Assert.IsType<TreeFkCollisionChecker>(CollisionCheckerFactory.Create(robot, tree, null, names));
        Assert.True(factory.IsCollisionFree(home, empty), factory.LastHit);
        Assert.False(factory.IsCollisionFree(dipped, empty));
        Assert.Contains("base_b", factory.LastHit);

        // Adjacency alone does not allow the design contacts; the explicit opt-out checks no link pairs.
        var adjacentOnly = new TreeFkCollisionChecker(robot, tree, planJointNames: names, selfCollision: new TreeSelfCollision(0,
            TreeAdjacency.AllowedPairs(tree, col.Links.Select(l => l.LinkName).ToHashSet(), fixedSiblingsTouch: true)));
        Assert.False(adjacentOnly.IsCollisionFree(home, empty));
        Assert.True(new TreeFkCollisionChecker(robot, tree, planJointNames: names, selfCollision: null).IsCollisionFree(dipped, empty));
    }

    [Fact]
    public void Clearance_OnlyAddsContacts()
    {
        var poses = Poses(400, seed: 7);
        var hits = new[] { 0.0, 0.002, 0.003 }.Select(c =>
        {
            var checker = Checker(c);
            return poses.Select(q => !checker.IsCollisionFree(q, new CollisionScene())).ToArray();
        }).ToArray();
        for (var i = 0; i < poses.Count; i++)
        {
            Assert.True(!hits[0][i] || hits[1][i]);
            Assert.True(!hits[1][i] || hits[2][i]);
        }
        Assert.Contains(Enumerable.Range(0, poses.Count), i => hits[2][i] && !hits[0][i]);
    }

    [Theory]
    [InlineData(0.002)]
    [InlineData(0.003)]
    public void BroadPhase_AgreesWithExactOnly(double clearanceM)
    {
        var fast = Checker(clearanceM);
        var exact = Checker(clearanceM, exactOnly: true);
        // A capsule across the work area exercises the axis clip; a box keeps one plain scene object.
        var scene = new CollisionScene([
            CollisionObject.Capsule("bar", new Frame(0.15, 0.25, 0, Math.Sqrt(0.5), 0, Math.Sqrt(0.5), 0), 0.035, 0.6),
            CollisionObject.Box("post", new Frame(-0.2, -0.2, 0), 0.02, 0.02, 0.2),
        ]);
        int hit = 0, free = 0;
        foreach (var q in Poses(1500, seed: 11))
        {
            foreach (var s in new[] { scene, new CollisionScene() })
            {
                var f = fast.IsCollisionFree(q, s);
                Assert.Equal(exact.IsCollisionFree(q, s), f);
                Assert.Equal(exact.LastHit, fast.LastHit);
                if (f) free++; else hit++;
            }
        }
        Assert.True(hit > 100 && free > 100, $"hit {hit}, free {free}");
    }

    [Fact]
    public void Grow_IsCachedPublic_AndInflateForwards()
    {
        var box = CollisionObject.Box("b", Frame.Identity, 0.1, 0.1, 0.1);
        Assert.Same(box, CollisionGeometry.Grow(box, 0));
        var g = CollisionGeometry.Grow(box, 0.002);
        Assert.Same(g, CollisionGeometry.Grow(box, 0.002));
        Assert.Equal(0.102, g.ExtentX, 12);
        Assert.Throws<ArgumentOutOfRangeException>(() => CollisionGeometry.Grow(box, -1e-3));
        Assert.Throws<ArgumentOutOfRangeException>(() => CollisionGeometry.Grow(box, double.NaN));
#pragma warning disable CS0618
        Assert.Same(g, Motus.Construction.BambooTrussCollision.Inflate(box, 0.002));
#pragma warning restore CS0618
    }

    [Fact]
    public void SegmentDistance_IsPublic()
    {
        Assert.Equal(1, SegmentDistance.PointSegmentSq(new Frame(0, 1, 0), new Frame(-1, 0, 0), new Frame(1, 0, 0)), 12);
        Assert.Equal(4, SegmentDistance.PointSegmentSq(3, 0, 0, -1, 0, 0, 1, 0, 0), 12);
        Assert.Equal(0, SegmentDistance.SegmentSegmentSq(new Frame(-1, 0, 0), new Frame(1, 0, 0), new Frame(0, -1, 0), new Frame(0, 1, 0)), 12);
    }

    // Arm mostly folded over the base, so the sample has contacts and near misses.
    internal static IReadOnlyList<JointState> Poses(int n, int seed)
    {
        var rng = new Random(seed);
        return Enumerable.Range(0, n)
            .Select(_ => new JointState([rng.NextDouble() * Math.PI / 2, Math.PI + (rng.NextDouble() - 0.5) * 0.8]))
            .ToArray();
    }
}
