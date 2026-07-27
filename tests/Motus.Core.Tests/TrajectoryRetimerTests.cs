using System.Text.Json;
using Motus.Core;

namespace Motus.Core.Tests;

public class TotgRetimerTests
{
  [Fact]
  public void Totg_IsDeterministicAndFinite()
  {
    var trajectory = DemoTrajectory();
    var options = new TrajectoryRetimerOptions { Algorithm = RetimerAlgorithm.Totg };

    var a = TrajectoryRetimer.Retime(trajectory, options);
    var b = TrajectoryRetimer.Retime(trajectory, options);

    Assert.Equal(a.Points.Count, b.Points.Count);
    for (var i = 0; i < a.Points.Count; i++)
    {
      Assert.True(double.IsFinite(a.Points[i].TimeSeconds));
      Assert.Equal(a.Points[i].TimeSeconds, b.Points[i].TimeSeconds, 12);
    }
    Assert.True(a.DurationSeconds > 0);
  }

  [Fact]
  public void Totg_RespectsJointVelocityLimits()
  {
    var retimed = TrajectoryRetimer.Retime(
      DemoTrajectory(),
      new TrajectoryRetimerOptions { Algorithm = RetimerAlgorithm.Totg });
    var limits = retimed.Robot.Preset.JointLimits;

    for (var i = 1; i < retimed.Points.Count; i++)
    {
      var dt = retimed.Points[i].TimeSeconds - retimed.Points[i - 1].TimeSeconds;
      Assert.True(dt > 0);
      for (var j = 0; j < limits.Count; j++)
      {
        var velocity = Math.Abs(retimed.Points[i].JointState.Positions[j] - retimed.Points[i - 1].JointState.Positions[j]) / dt;
        Assert.True(velocity <= limits[j].MaxVelocity!.Value + 1e-9, $"joint {j}: {velocity} > {limits[j].MaxVelocity}");
      }
    }
  }

  [Fact]
  public void TotgExport_WritesRetimeProvenance()
  {
    var json = TrajectoryExport.ToJson(DemoTrajectory(), new TrajectoryExportOptions
    {
      Retime = true,
      Retimer = new TrajectoryRetimerOptions { Algorithm = RetimerAlgorithm.Totg }
    });

    using var doc = JsonDocument.Parse(json);
    var provenance = doc.RootElement.GetProperty("provenance");
    Assert.Equal("Totg", provenance.GetProperty("retimeAlgorithm").GetString());
    Assert.Contains(TotgMethodRefs.PhamPham2018ToppraDoi, provenance.GetProperty("settingsHash").GetString());
  }

  private static Trajectory DemoTrajectory()
  {
    var preset = new RobotPreset
    {
      Manufacturer = RobotManufacturer.Unknown,
      ModelName = "retimer_demo",
      Family = "test",
      AxisCount = 2,
      JointLimits = new[]
      {
        JointLimit.Radians(-2, 2, maxVelocity: 0.5, maxAcceleration: 1.0),
        JointLimit.Radians(-2, 2, maxVelocity: 0.4, maxAcceleration: 0.8)
      }
    };
    var robot = new RobotModel(preset);
    return new Trajectory(robot, new[]
    {
      new TrajectoryPoint(0, new JointState(new[] { 0.0, 0.0 })),
      new TrajectoryPoint(1, new JointState(new[] { 0.2, 0.1 })),
      new TrajectoryPoint(2, new JointState(new[] { 0.5, -0.1 }))
    });
  }
}
