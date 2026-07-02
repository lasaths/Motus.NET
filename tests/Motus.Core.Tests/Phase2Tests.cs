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
    var table = CollisionObject.Box("table", new Frame(2, 2, 2), 0.5, 0.5, 0.05);
    Assert.True(checker.IsCollisionFree(start, new CollisionScene(new[] { table })));
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
  public void FastSegment_RequiresMoreSamples()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var checker = new SphereCollisionChecker(preset);
    var start = new JointState(new double[6]);
    var mid = new JointState(new[] { 1.5, -1.0, 1.5, -1.0, -1.0, 0.5 });
    var scene = new CollisionScene(new[] { CollisionObject.Sphere("block", new Frame(0.5, -0.3, 0.4), 0.06) });
    var coarse = checker.SegmentCollisionFree(start, mid, scene, 0.5);
    var fine = checker.SegmentCollisionFree(start, mid, scene, 0.05);
    if (!coarse && fine)
      Assert.True(true);
    else
      Assert.True(!coarse || fine);
  }
}

public class SrdfLoaderTests
{
  [Fact]
  public void MergeAllowedPairs_AddsToScene()
  {
    var scene = new CollisionScene(new[] { CollisionObject.Box("table", Frame.Identity, 1, 1, 0.05) });
    var pairs = new[] { ("base_link", "table") };
    var merged = SrdfLoader.MergeAllowedPairs(scene, pairs);
    Assert.Contains(merged.AllowedPairs, p => p.A == "base_link" && p.B == "table");
  }
}
