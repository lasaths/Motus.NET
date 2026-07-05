using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;

namespace Motus.Core.Tests;

public class UrdfImportTests
{
  private static string FixturePath(string name) =>
    Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", name));

  [Fact]
  public void UrdfLoad_TwoLink_HasCorrectDof()
  {
    var robot = UrdfRobotLoader.Load(FixturePath("two_link.urdf"), new UrdfLoadOptions
    {
      BaseLink = "base_link",
      TipLink = "tip_link",
      ModelName = "two_link"
    });

    Assert.Equal(2, robot.Preset.AxisCount);
    Assert.Equal(2, robot.Chain.Joints.Length);
  }

  [Fact]
  public void UrdfLoad_TwoLink_FK_IK_RoundTrip()
  {
    var robot = UrdfRobotLoader.Load(FixturePath("two_link.urdf"), new UrdfLoadOptions
    {
      BaseLink = "base_link",
      TipLink = "tip_link"
    });

    var fk = KinematicsResolver.CreateFkSolver(robot.Preset, robot.Chain);
    var ik = KinematicsResolver.CreateInverseKinematics(robot.Preset, robot.Chain);
    var joints = new JointState(new[] { 0.3, -0.5 });

    var pose = fk.ComputeTcp(joints, robot.Preset.BaseFrame, robot.Preset.ToolFrame);
    Assert.True(ik.TrySolve(pose, joints, out var solved));

    var check = fk.ComputeTcp(solved, robot.Preset.BaseFrame, robot.Preset.ToolFrame);
    var posErr = Math.Sqrt(
      Math.Pow(check.Tcp.X - pose.Tcp.X, 2) +
      Math.Pow(check.Tcp.Y - pose.Tcp.Y, 2) +
      Math.Pow(check.Tcp.Z - pose.Tcp.Z, 2));
    Assert.True(posErr < 0.005, $"Round-trip error {posErr:F4}m");
  }

  [Fact]
  public void RrtConnect_OnUrdfImportedRobot()
  {
    var urdf = UrdfRobotLoader.Load(FixturePath("two_link.urdf"), new UrdfLoadOptions
    {
      BaseLink = "base_link",
      TipLink = "tip_link"
    });
    var model = urdf.ToModel();
    var start = new JointState(new[] { 0.0, 0.0 });
    var goal = new JointState(new[] { 0.4, -0.3 });
    var planner = new RrtConnectPlanner(urdf.Preset, urdf.Chain, new RrtConnectOptions { MaxIterations = 2000, RandomSeed = 1 });
    var result = planner.Plan(new PlanningRequest(model, start, goal));
    Assert.True(result.Success, string.Join("; ", result.Errors));
  }

  [Fact]
  public void UrdfLoad_MissingTipLink_Throws()
  {
    Assert.Throws<InvalidOperationException>(() => UrdfRobotLoader.Load(FixturePath("two_link.urdf"), new UrdfLoadOptions
    {
      BaseLink = "base_link",
      TipLink = "nonexistent_link"
    }));
  }

  [Fact]
  public void UrdfLoad_PrismaticLift_MovesTcpWithPrismaticJoint()
  {
    var robot = UrdfRobotLoader.Load(FixturePath("prismatic_lift.urdf"), new UrdfLoadOptions
    {
      BaseLink = "base_link",
      TipLink = "tip_link"
    });
    var fk = KinematicsResolver.CreateFkSolver(robot.Preset, robot.Chain);
    var low = fk.ComputeTcp(new JointState(new[] { 0.0, 0.0 }), robot.Preset.BaseFrame, robot.Preset.ToolFrame);
    var high = fk.ComputeTcp(new JointState(new[] { 0.4, 0.0 }), robot.Preset.BaseFrame, robot.Preset.ToolFrame);
    Assert.True(high.Tcp.Z > low.Tcp.Z + 0.35);
  }

  [Fact]
  public void UrdfLoad_Ur10eOfficial_HasSixAxes()
  {
    var robot = UrdfRobotLoader.Load(FixturePath("ur10e/ur10e.urdf"), new UrdfLoadOptions
    {
      BaseLink = "base_link",
      TipLink = "tool0",
      ModelName = "UR10e"
    });

    Assert.Equal(6, robot.Preset.AxisCount);
    Assert.Equal(6, robot.JointNames.Count);
    Assert.Null(robot.CollisionModel);
  }

  [Fact]
  public void UrdfLoad_TipLinkCollision_BecomesToolGeometry()
  {
    var robot = UrdfRobotLoader.Load(FixturePath("ur5e_collision.urdf"), new UrdfLoadOptions
    {
      BaseLink = "base_link",
      TipLink = "tool0"
    });

    Assert.NotNull(robot.CollisionModel?.ToolGeometry);
    Assert.Equal(CollisionShape.Box, robot.CollisionModel.ToolGeometry.Shape);
  }

  [Fact]
  public void UrdfLoad_Kr210R3100Ultra_HasSixAxes()
  {
    var robot = UrdfRobotLoader.Load(FixturePath("kr210_r3100_ultra/kr210_r3100_ultra_minimal.urdf"), new UrdfLoadOptions
    {
      BaseLink = "base_link",
      TipLink = "tool0",
      ModelName = "KR 210 R3100 ultra"
    });

    Assert.Equal(6, robot.Preset.AxisCount);
    Assert.Equal(6, robot.JointNames.Count);
    Assert.Contains("joint_1", robot.JointNames);
  }
}
