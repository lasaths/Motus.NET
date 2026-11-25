using System.Xml.Linq;
using BenchmarkDotNet.Attributes;
using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Benchmarks;

/// <summary>
/// Tree FK collision on the synthetic 3-link fixture (tests/fixtures/tree3), 256 poses per op with a capsule and a box:
/// scene only (self-collision off), self-collision exact path only, and self-collision with the broad phase (2 mm).
/// </summary>
[MemoryDiagnoser]
public class TreeSelfCollisionBenchmarks
{
    private TreeFkCollisionChecker _sceneOnly = null!, _exactOnly = null!, _broadPhase = null!;
    private JointState[] _poses = null!;
    private CollisionScene _scene = null!;

    [GlobalSetup]
    public void Setup()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "tests", "fixtures", "tree3", "tree3.urdf")))
            dir = dir.Parent;
        var urdf = Path.Combine(dir?.FullName ?? throw new DirectoryNotFoundException("tests/fixtures/tree3 not found"),
            "tests", "fixtures", "tree3", "tree3.urdf");
        var tree = UrdfRobotLoader.LoadTree(urdf);
        var col = UrdfCollisionLoader.LoadTree(XDocument.Load(urdf).Root!, tree, Path.GetDirectoryName(urdf)!, source: UrdfMeshSource.Visual)!;
        var names = tree.DriverJointIndices.Select(i => tree.Joints[i].Name).ToArray();
        var robot = new RobotModel(new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = "tree3",
            Family = "tree3",
            AxisCount = tree.DriverCount,
            JointLimits = tree.DriverJointIndices.Select(i => new JointLimit(tree.Joints[i].Lower, tree.Joints[i].Upper)).ToArray(),
        }, col, names);
        var self = new TreeSelfCollision(0.002, TreeAdjacency.AllowedPairs(
            tree, col.Links.Select(l => l.LinkName).ToHashSet(StringComparer.Ordinal), fixedSiblingsTouch: false));
        _sceneOnly = new TreeFkCollisionChecker(robot, tree, planJointNames: names);
        _exactOnly = new TreeFkCollisionChecker(robot, tree, planJointNames: names, selfCollision: self) { ExactOnly = true };
        _broadPhase = new TreeFkCollisionChecker(robot, tree, planJointNames: names, selfCollision: self);
        _scene = new CollisionScene([
            CollisionObject.Capsule("bar", new Frame(0.15, 0.25, 0, Math.Sqrt(0.5), 0, Math.Sqrt(0.5), 0), 0.035, 0.6),
            CollisionObject.Box("post", new Frame(-0.2, -0.2, 0), 0.02, 0.02, 0.2),
        ]);
        var rng = new Random(11);
        _poses = Enumerable.Range(0, 256)
            .Select(_ => new JointState([rng.NextDouble() * Math.PI / 2, Math.PI + (rng.NextDouble() - 0.5) * 0.8]))
            .ToArray();
    }

    [Benchmark(Baseline = true)]
    public int SceneOnly() => Run(_sceneOnly);

    [Benchmark]
    public int SelfExactOnly() => Run(_exactOnly);

    [Benchmark]
    public int SelfBroadPhase() => Run(_broadPhase);

    private int Run(TreeFkCollisionChecker checker)
    {
        var free = 0;
        foreach (var q in _poses)
            if (checker.IsCollisionFree(q, _scene))
                free++;
        return free;
    }
}
