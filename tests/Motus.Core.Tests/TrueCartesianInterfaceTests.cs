using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>True Cartesian motion interface tests — ponytail: LIN planner exists, basic API works.</summary>
public class TrueCartesianInterfaceTests
{
  private static string ResourcesRoot =>
    Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

  [Fact]
  public void LIN_Planner_Constructs()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var planner = new CartesianLinearPathPlanner(preset);
    Assert.NotNull(planner);
  }

  [Fact]
  public void LIN_StraightLine_HomeToHome()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var fk = new DhForwardKinematics(preset);
    var planner = new CartesianLinearPathPlanner(preset);

    var startJoint = new JointState(new[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    var startPose = fk.ComputeTcp(startJoint, preset.BaseFrame, preset.ToolFrame);

    // PONYTAIL: Zero-length move should work even if IK struggles
    var traj = planner.Plan(startPose, startPose, startJoint);
    Assert.NotNull(traj);
    Assert.Single(traj!.Points);
  }

  private static JointState IkFriendlyStart => new(new[] { 0.0, -0.5, 1.0, -1.0, 0.0, 0.0 });

  [Fact]
  public void LIN_SmallLocalMove_HasPhysicalDuration()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var fk = new DhForwardKinematics(preset);
    var planner = new CartesianLinearPathPlanner(preset);

    var startJoint = IkFriendlyStart;
    var startPose = fk.ComputeTcp(startJoint, preset.BaseFrame, preset.ToolFrame);
    var goalPose = new CartesianPose(new Frame(
      startPose.Tcp.X + 0.02, startPose.Tcp.Y, startPose.Tcp.Z,
      startPose.Tcp.Qw, startPose.Tcp.Qx, startPose.Tcp.Qy, startPose.Tcp.Qz));

    var traj = planner.Plan(startPose, goalPose, startJoint);
    Assert.NotNull(traj);
    Assert.True(traj!.DurationSeconds > 0.01);
    Assert.True(traj.Points[^1].TimeSeconds > 0.01);
  }

  [Fact]
  public void LIN_SmallLocalMove()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var fk = new DhForwardKinematics(preset);
    var planner = new CartesianLinearPathPlanner(preset);

    var startJoint = IkFriendlyStart;
    var startPose = fk.ComputeTcp(startJoint, preset.BaseFrame, preset.ToolFrame);

    // PONYTAIL: 2cm local move along X
    var goalPose = new CartesianPose(new Frame(
      startPose.Tcp.X + 0.02,
      startPose.Tcp.Y,
      startPose.Tcp.Z,
      startPose.Tcp.Qw, startPose.Tcp.Qx, startPose.Tcp.Qy, startPose.Tcp.Qz
    ));

    var traj = planner.Plan(startPose, goalPose, startJoint);
    Assert.NotNull(traj);
    Assert.True(traj!.DurationSeconds >= 0);
    Assert.True(traj.Points.Count >= 2);
  }

  [Fact]
  public void LIN_Toolpath_InterfaceWorks()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var fk = new DhForwardKinematics(preset);
    var planner = new CartesianLinearPathPlanner(preset);

    var startJoint = IkFriendlyStart;
    var startPos = fk.ComputeTcp(startJoint, preset.BaseFrame, preset.ToolFrame);

    // PONYTAIL: L-shaped toolpath, 2cm segments
    var waypoints = new[]
    {
      startPos,
      new CartesianPose(new Frame(startPos.Tcp.X + 0.02, startPos.Tcp.Y, startPos.Tcp.Z, startPos.Tcp.Qw, startPos.Tcp.Qx, startPos.Tcp.Qy, startPos.Tcp.Qz)),
      new CartesianPose(new Frame(startPos.Tcp.X + 0.02, startPos.Tcp.Y + 0.02, startPos.Tcp.Z, startPos.Tcp.Qw, startPos.Tcp.Qx, startPos.Tcp.Qy, startPos.Tcp.Qz))
    };

    var traj = planner.PlanToolpath(waypoints, startJoint);
    Assert.NotNull(traj);
    Assert.True(traj!.DurationSeconds >= 0);
    Assert.True(traj.Points.Count >= 3);
  }

  [Fact]
  public void LIN_PassesOnUnreachableWorkspaceTarget()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var fk = new DhForwardKinematics(preset);
    var planner = new CartesianLinearPathPlanner(preset);

    var startJoint = new JointState(new[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    var startPose = fk.ComputeTcp(startJoint, preset.BaseFrame, preset.ToolFrame);

    // PONYTAIL: Target 5m away — impossible for UR5e
    var goalPose = new CartesianPose(new Frame(
      startPose.Tcp.X + 5.0,
      startPose.Tcp.Y,
      startPose.Tcp.Z,
      startPose.Tcp.Qw, startPose.Tcp.Qx, startPose.Tcp.Qy, startPose.Tcp.Qz
    ));

    var traj = planner.Plan(startPose, goalPose, startJoint);
    Assert.Null(traj);  // Should fail gracefully
  }
}
