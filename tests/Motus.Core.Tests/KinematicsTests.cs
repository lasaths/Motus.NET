using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class KinematicsTests
{
  private static string ResourcesRoot =>
    Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

  [Fact]
  public void ForwardKinematics_AllPresets_HomePosition()
  {
    foreach (var model in PresetLoader.ListAvailableModels(ResourcesRoot))
    {
      var preset = PresetLoader.LoadByModelName(model, ResourcesRoot);
      if (!KinematicsResolver.SupportsModel(preset)) continue;
      var fk = new DhForwardKinematics(preset);
      var state = new JointState(Enumerable.Repeat(0.0, preset.AxisCount).ToArray());
      var tcp = fk.ComputeTcp(state, preset.BaseFrame, preset.ToolFrame);
      Assert.True(double.IsFinite(tcp.Tcp.X));
      Assert.True(double.IsFinite(tcp.Tcp.Z));
    }
  }

  [Fact]
  public void IkRoundTrip_Ur5e()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var fk = new DhForwardKinematics(preset);
    var ik = KinematicsResolver.CreateInverseKinematics(preset);
    var seed = new JointState(new[] { 0.1, -0.5, 0.8, -0.3, -0.4, 0.2 });
    var target = fk.ComputeTcp(seed, preset.BaseFrame, preset.ToolFrame);
    Assert.True(ik.TrySolve(target, seed, out var solved));
    var check = fk.ComputeTcp(solved, preset.BaseFrame, preset.ToolFrame);
    Assert.Equal(target.Tcp.X, check.Tcp.X, 3);
    Assert.Equal(target.Tcp.Y, check.Tcp.Y, 3);
    Assert.Equal(target.Tcp.Z, check.Tcp.Z, 3);
  }
}

public class CollisionTests
{
  private static string ResourcesRoot =>
    Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

  [Fact]
  public void SphereObstacle_BlocksConfiguration()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var checker = new SphereCollisionChecker(preset);
    var fk = new DhForwardKinematics(preset);
    var state = new JointState(new[] { 0.5, -1.0, 1.2, -0.5, -1.0, 0.0 });
    var tcp = fk.ComputeTcp(state, preset.BaseFrame, preset.ToolFrame);
    var obstacle = CollisionObject.Sphere("block", tcp.Tcp, 0.15);
    var scene = new CollisionScene(new[] { obstacle });
    Assert.False(checker.IsCollisionFree(state, scene));
  }
}

public class CartesianPlannerTests
{
  private static string ResourcesRoot =>
    Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

  [Fact]
  public void PlansToCartesianGoal()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var robot = new RobotModel(preset);
    var fk = new DhForwardKinematics(preset);
    var start = new JointState(new[] { 0.1, -0.5, 0.8, -0.3, -0.4, 0.2 });
    var goalPose = fk.ComputeTcp(start, preset.BaseFrame, preset.ToolFrame);
    var result = new CartesianLinearPlanner(preset).Plan(new CartesianPlanningRequest(robot, start, goalPose));
    Assert.True(result.Success, string.Join("; ", result.Errors));
    Assert.NotNull(result.Trajectory);
  }
}
