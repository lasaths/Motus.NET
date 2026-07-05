using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class RobotCollisionTests
{
  private static string ResourcesRoot =>
    Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

  [Fact]
  public void Ur5ePreset_LoadsCollisionLinks()
  {
    var model = PresetLoader.LoadRobotModelByName("UR5e", ResourcesRoot);
    Assert.NotNull(model.CollisionModel);
    Assert.Equal(6, model.CollisionModel!.Links.Count);
  }

  [Fact]
  public void RobotMeshChecker_UsesLinkCapsules()
  {
    var model = PresetLoader.LoadRobotModelByName("UR5e", ResourcesRoot);
    var checker = new RobotMeshCollisionChecker(model);
    var start = new JointState(new double[6]);
    Assert.True(checker.IsCollisionFree(start, new CollisionScene()), "home config should be self-collision-free");
    var table = CollisionObject.Box("table", new Frame(2, 2, 2), 0.5, 0.5, 0.05);
    Assert.True(checker.IsCollisionFree(start, new CollisionScene(new[] { table })));
  }

  [Fact]
  public void Ur10ePreset_LoadsCollisionLinks()
  {
    var model = PresetLoader.LoadRobotModelByName("UR10e", ResourcesRoot);
    Assert.NotNull(model.CollisionModel);
    Assert.Equal(6, model.CollisionModel!.Links.Count);
  }

  [Fact]
  public void UrdfCollision_OfficialUr10e_HasNoEmbeddedCollision()
  {
    var path = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", "ur10e", "ur10e.urdf"));
    var urdf = UrdfRobotLoader.Load(path, new UrdfLoadOptions { BaseLink = "base_link", TipLink = "tool0" });
    Assert.Null(urdf.CollisionModel);
  }

  [Fact]
  public void UrdfCollision_LoadsFromFixture()
  {
    var path = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", "ur5e_collision.urdf"));
    var urdf = UrdfRobotLoader.Load(path, new UrdfLoadOptions { BaseLink = "base_link", TipLink = "tool0" });
    Assert.NotNull(urdf.CollisionModel);
    Assert.True(urdf.CollisionModel!.Links.Count >= 2);
  }
}

public class BottleneckRetimerTests
{
  [Fact]
  public void Bottleneck_SlowerThanFastestJointAlone()
  {
    var limits = new List<JointLimit>
    {
      new(-3.14, 3.14, maxVelocityRadiansPerSecond: 0.5),
      new(-3.14, 3.14, maxVelocityRadiansPerSecond: 3.0)
    };
    var robot = new RobotModel(new RobotPreset
    {
      Manufacturer = RobotManufacturer.Unknown,
      ModelName = "test",
      AxisCount = 2,
      JointLimits = limits
    });
    var geo = new Trajectory(robot, new[]
    {
      new TrajectoryPoint(0, new JointState(new[] { 0.0, 0.0 })),
      new TrajectoryPoint(1, new JointState(new[] { 1.0, 0.01 }))
    });
    var retimed = TrajectoryRetimer.Retime(geo, new TrajectoryRetimerOptions { Algorithm = RetimerAlgorithm.Bottleneck });
    Assert.True(retimed.DurationSeconds >= 1.0 / 0.5 - 0.01);
  }
}

public class DenseSegmentCollisionTests
{
  private static string ResourcesRoot =>
    Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

  [Fact]
  public void CoarseStep_NeverClearsSegmentRejectedByFineSampling()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var checker = new SphereCollisionChecker(preset);
    var scene = new CollisionScene(new[] { CollisionObject.Sphere("block", new Frame(0.5, -0.3, 0.4), 0.06) });
    var rng = new Random(42);
    for (var trial = 0; trial < 200; trial++)
    {
      var start = RandomJoint(rng, 6);
      var mid = RandomJoint(rng, 6);
      var coarse = checker.SegmentCollisionFree(start, mid, scene, 0.5);
      var fine = checker.SegmentCollisionFree(start, mid, scene, 0.05);
      Assert.False(coarse && !fine, $"coarse cleared segment that fine rejected (trial {trial})");
    }
  }

  private static JointState RandomJoint(Random rng, int n)
  {
    var q = new double[n];
    for (var i = 0; i < n; i++)
      q[i] = rng.NextDouble() * 2.4 - 1.2;
    return new JointState(q);
  }
}

public class SrdfLoaderTests
{
  private static string Ur10eSrdfPath =>
    Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", "ur10e", "ur10e.srdf"));

  [Fact]
  public void MergeAllowedPairs_AddsToScene()
  {
    var scene = new CollisionScene(new[] { CollisionObject.Box("table", Frame.Identity, 1, 1, 0.05) });
    var pairs = new[] { ("base_link", "table") };
    var merged = SrdfLoader.MergeAllowedPairs(scene, pairs);
    Assert.Contains(merged.AllowedPairs, p => p.A == "base_link" && p.B == "table");
  }

  [Fact]
  public void LoadGroups_OfficialUr10eSrdf_ReturnsManipulatorChain()
  {
    var groups = SrdfLoader.LoadGroups(Ur10eSrdfPath);
    var manipulator = Assert.Single(groups, g => g.Name == "ur_manipulator");
    Assert.Equal("base_link", manipulator.BaseLink);
    Assert.Equal("tool0", manipulator.TipLink);
    Assert.Single(manipulator.JointNames);
    Assert.Contains("..", manipulator.JointNames[0]);
  }

  [Fact]
  public void LoadAllowedPairs_OfficialUr10eSrdf_MergeWithLinkIndices()
  {
    var pairs = SrdfLoader.LoadAllowedPairs(Ur10eSrdfPath);
    var scene = new CollisionScene();
    var linkMap = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase)
    {
      ["base_link"] = 0,
      ["base_link_inertia"] = 1
    };
    var merged = SrdfLoader.MergeAllowedPairs(scene, pairs, linkMap);
    Assert.Contains(merged.AllowedPairs, p => p.A == CollisionBodies.RobotLink(0) && p.B == CollisionBodies.RobotLink(1));
  }

  [Fact]
  public void LoadEndEffectors_ParsesParentLink()
  {
    var doc = System.Xml.Linq.XDocument.Parse("""
      <robot name="ur10e">
        <end_effector name="ee" parent_link="tool0" group="ur_manipulator"/>
      </robot>
      """);
    var map = SrdfLoader.LoadEndEffectors(doc);
    Assert.Equal("tool0", map["ee"]);
  }
}
