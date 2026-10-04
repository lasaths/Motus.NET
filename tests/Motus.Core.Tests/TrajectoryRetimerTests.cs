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

  [Fact]
  public void Totg_StewartLegLengthsInMeters_UseMeterDefaults()
  {
    // Stewart platform leg lengths are meters; retimer must not apply radian velocity defaults.
    var preset = new RobotPreset
    {
      Manufacturer = RobotManufacturer.Unknown,
      ModelName = "stewart_retimer_test",
      Family = Units.StewartFamily,
      AxisCount = 6,
      // No maxVelocity/maxAcceleration set → retimer falls back to defaults
      JointLimits = Enumerable.Range(0, 6)
        .Select(_ => JointLimit.Meters(0.45, 0.75))
        .ToList()
    };
    var robot = new RobotModel(preset);
    // Small leg-length motion: 0.01 m over geometric time 0.01 s → 1 m/s if geometric, much slower after retime
    var trajectory = new Trajectory(robot, new[]
    {
      new TrajectoryPoint(0, new JointState(Enumerable.Repeat(0.60, 6).ToArray())),
      new TrajectoryPoint(0.01, new JointState(Enumerable.Repeat(0.61, 6).ToArray()))
    });

    var retimed = TrajectoryRetimer.Retime(trajectory, new TrajectoryRetimerOptions { Algorithm = RetimerAlgorithm.Totg });

    // The retimer should use meter-appropriate defaults (0.5 m/s, 1.0 m/s²), not radian defaults (1.5 rad/s as m/s).
    // For 0.01 m motion with amax=1.0 m/s², triangular profile: v_peak=sqrt(a*d)=0.1 m/s, duration=2*sqrt(d/a)=0.2 s.
    // If it incorrectly used 1.5 rad/s as 1.5 m/s, duration would be ~0.01 s (too fast).
    Assert.True(retimed.DurationSeconds > 0.02, 
      $"Stewart leg retime duration {retimed.DurationSeconds:F4} s is too short; " +
      $"likely using radian velocity default (1.5) as m/s instead of meter default.");
    Assert.True(retimed.DurationSeconds <= 0.25,
      $"Retimed duration {retimed.DurationSeconds:F4} s unexpectedly high (expected ~0.2 s triangular profile).");
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
