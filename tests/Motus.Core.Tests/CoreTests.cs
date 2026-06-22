using Motus.Core;

namespace Motus.Core.Tests;

public class UnitsTests
{
    [Fact]
    public void DegreeRadianRoundTrip()
    {
        Assert.Equal(90, Units.ToDegrees(Units.ToRadians(90)), 6);
        var degs = new[] { 0.0, 45.0, -90.0 };
        var rads = Units.ToRadians(degs);
        var back = Units.ToDegrees(rads);
        Assert.Equal(degs, back);
    }
}

public class JointStateTests
{
  [Fact]
  public void ValidStatePasses()
  {
    var limits = new[] { new JointLimit(-1, 1) };
    var s = new JointState(new[] { 0.0 });
    Assert.True(s.Validate(limits).IsValid);
  }

  [Fact]
  public void OutOfLimitFails()
  {
    var limits = new[] { new JointLimit(-1, 1) };
    var s = new JointState(new[] { 2.0 });
    Assert.False(s.Validate(limits).IsValid);
  }
}

public class JointLinearPlannerTests
{
  private static RobotModel Ur5e()
  {
    var limits = Enumerable.Repeat(new JointLimit(-6.28, 6.28), 6).ToList();
    return new RobotModel(new RobotPreset
    {
      Manufacturer = RobotManufacturer.UniversalRobots,
      ModelName = "UR5e",
      AxisCount = 6,
      JointLimits = limits
    });
  }

  [Fact]
  public void PlansInterpolatedTrajectory()
  {
    var robot = Ur5e();
    var start = new JointState(new double[6]);
    var goal = new JointState(Enumerable.Repeat(0.5, 6).ToArray());
    var result = new JointLinearPlanner().Plan(new PlanningRequest(robot, start, goal));
    Assert.True(result.Success);
    Assert.NotNull(result.Trajectory);
    Assert.True(result.Trajectory!.Points.Count > 1);
    Assert.Equal(0, result.Trajectory.Points[0].JointState.Positions[0], 6);
    Assert.Equal(0.5, result.Trajectory.Points[^1].JointState.Positions[0], 6);
  }

  [Fact]
  public void RejectsInvalidStart()
  {
    var robot = Ur5e();
    var bad = new JointState(Enumerable.Repeat(99.0, 6).ToArray());
    var goal = new JointState(new double[6]);
    var result = new JointLinearPlanner().Plan(new PlanningRequest(robot, bad, goal));
    Assert.False(result.Success);
    Assert.Contains(result.Errors, e => e.Contains("Start"));
  }

  [Fact]
  public void RejectsInvalidGoal()
  {
    var robot = Ur5e();
    var start = new JointState(new double[6]);
    var bad = new JointState(Enumerable.Repeat(99.0, 6).ToArray());
    var result = new JointLinearPlanner().Plan(new PlanningRequest(robot, start, bad));
    Assert.False(result.Success);
    Assert.Contains(result.Errors, e => e.Contains("Goal"));
  }

  [Fact]
  public void RejectsMismatchedAxisCount()
  {
    var robot = Ur5e();
    var start = new JointState(new[] { 0.0 });
    var goal = new JointState(new double[6]);
    var result = new JointLinearPlanner().Plan(new PlanningRequest(robot, start, goal));
    Assert.False(result.Success);
  }

  [Fact]
  public void RespectsMaxJointStep()
  {
    var robot = Ur5e();
    var start = new JointState(new double[6]);
    var goal = new JointState(Enumerable.Repeat(1.0, 6).ToArray());
    var opts = new PlanningOptions { MaxJointStepRadians = 0.1 };
    var traj = new JointLinearPlanner().Plan(new PlanningRequest(robot, start, goal, opts)).Trajectory!;
    for (var i = 1; i < traj.Points.Count; i++)
    {
      for (var j = 0; j < 6; j++)
      {
        var step = Math.Abs(traj.Points[i].JointState.Positions[j] - traj.Points[i - 1].JointState.Positions[j]);
        Assert.True(step <= opts.MaxJointStepRadians + 1e-9);
      }
    }
  }

  [Fact]
  public void TimingIncreasesMonotonically()
  {
    var robot = Ur5e();
    var result = new JointLinearPlanner().Plan(new PlanningRequest(robot, new JointState(new double[6]), new JointState(Enumerable.Repeat(0.3, 6).ToArray())));
    var pts = result.Trajectory!.Points;
    for (var i = 1; i < pts.Count; i++)
      Assert.True(pts[i].TimeSeconds >= pts[i - 1].TimeSeconds);
  }

  [Fact]
  public void VelocityTimingRespectsMaxJointVelocity()
  {
    var limits = Enumerable.Repeat(new JointLimit(-6.28, 6.28, maxVelocityRadiansPerSecond: 3.14), 6).ToList();
    var robot = new RobotModel(new RobotPreset
    {
      Manufacturer = RobotManufacturer.UniversalRobots,
      ModelName = "UR5e",
      AxisCount = 6,
      JointLimits = limits
    });
    var opts = new PlanningOptions { MaxJointStepRadians = 0.1, TimeStepSeconds = 0.04, MaxJointVelocityRadiansPerSecond = 0.5 };
    var result = new JointLinearPlanner().Plan(new PlanningRequest(
      robot, new JointState(new double[6]), new JointState(Enumerable.Repeat(1.0, 6).ToArray()), opts));
    Assert.True(result.Success);
    var pts = result.Trajectory!.Points;
    var minDt = 0.1 / opts.MaxJointVelocityRadiansPerSecond;
    for (var i = 1; i < pts.Count; i++)
    {
      var dt = pts[i].TimeSeconds - pts[i - 1].TimeSeconds;
      Assert.True(dt >= minDt - 1e-9);
      Assert.True(pts[i].TimeSeconds > pts[i - 1].TimeSeconds);
    }
    Assert.True(new TrajectoryValidator().Validate(result.Trajectory).IsValid);
  }
}

public class TrajectoryValidatorTests
{
  [Fact]
  public void ValidTrajectoryPasses()
  {
    var robot = new RobotModel(new RobotPreset { Manufacturer = RobotManufacturer.UniversalRobots, ModelName = "T", AxisCount = 1, JointLimits = new[] { new JointLimit(-1, 1) } });
    var traj = new Trajectory(robot, new[] { new TrajectoryPoint(0, new JointState(new[] { 0.0 })), new TrajectoryPoint(1, new JointState(new[] { 0.5 })) });
    Assert.True(new TrajectoryValidator().Validate(traj).IsValid);
  }
}

public class ExportTests
{
  [Fact]
  public void JsonAndCsvExport()
  {
    var robot = new RobotModel(new RobotPreset { Manufacturer = RobotManufacturer.UniversalRobots, ModelName = "T", AxisCount = 2, JointLimits = new[] { new JointLimit(-1, 1), new JointLimit(-1, 1) } });
    var traj = new Trajectory(robot, new[] { new TrajectoryPoint(0, new JointState(new[] { 0.0, 0.1 })), new TrajectoryPoint(0.5, new JointState(new[] { 0.2, 0.3 })) });
    var json = TrajectoryExport.ToJson(traj);
    Assert.Contains("jointsRadians", json);
    var csv = TrajectoryExport.ToCsv(traj);
    Assert.Contains("time_seconds", csv);
    Assert.Contains("0.500000", csv);
  }
}
