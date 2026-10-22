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

  [Theory]
  [InlineData(RetimerAlgorithm.Totg)]
  [InlineData(RetimerAlgorithm.TotgLite)]
  [InlineData(RetimerAlgorithm.Bottleneck)]
  [InlineData(RetimerAlgorithm.SegmentTrapezoid)]
  public void StewartLegLengthsInMeters_UseMeterDefaults(RetimerAlgorithm algorithm)
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

    var retimed = TrajectoryRetimer.Retime(trajectory, new TrajectoryRetimerOptions { Algorithm = algorithm });

    // The retimer should use meter-appropriate defaults (0.5 m/s, 1.0 m/s²), not radian defaults (1.5 rad/s as m/s).
    // For 0.01 m motion with amax=1.0 m/s², triangular profile: v_peak=sqrt(a*d)=0.1 m/s, duration=2*sqrt(d/a)=0.2 s.
    // Applying the old radian acceleration default instead would produce 0.11547 s.
    Assert.Equal(0.2, retimed.DurationSeconds, 10);
  }

  [Theory]
  [InlineData(RetimerAlgorithm.Totg)]
  [InlineData(RetimerAlgorithm.TotgLite)]
  [InlineData(RetimerAlgorithm.Bottleneck)]
  [InlineData(RetimerAlgorithm.SegmentTrapezoid)]
  public void TwoPointMeterMotion_UsesVelocityCappedTrapezoid(RetimerAlgorithm algorithm)
  {
    var geometric = MakeTrajectory(new[] { JointLimit.Meters(0, 3) }, new[] { 0.0 }, new[] { 2.0 });
    var retimed = TrajectoryRetimer.Retime(geometric, new TrajectoryRetimerOptions { Algorithm = algorithm });

    // 2 m at 0.5 m/s and 1 m/s²: 0.5 s acceleration/deceleration, 3.5 s cruise.
    Assert.Equal(4.5, retimed.DurationSeconds, 10);
    Assert.True(2.0 / retimed.DurationSeconds <= 0.5);
  }

  [Theory]
  [InlineData(RetimerAlgorithm.Totg)]
  [InlineData(RetimerAlgorithm.TotgLite)]
  [InlineData(RetimerAlgorithm.Bottleneck)]
  [InlineData(RetimerAlgorithm.SegmentTrapezoid)]
  public void MixedUnits_SelectDefaultsForEachMovingAxis(RetimerAlgorithm algorithm)
  {
    var limits = new[] { JointLimit.Radians(-1, 1), JointLimit.Meters(0, 1) };
    var options = new TrajectoryRetimerOptions { Algorithm = algorithm };

    var meter = TrajectoryRetimer.Retime(MakeTrajectory(limits, new[] { 0.0, 0.0 }, new[] { 0.0, 0.01 }), options);
    var radians = TrajectoryRetimer.Retime(MakeTrajectory(limits, new[] { 0.0, 0.0 }, new[] { 0.01, 0.0 }), options);
    var together = TrajectoryRetimer.Retime(MakeTrajectory(limits, new[] { 0.0, 0.0 }, new[] { 0.09, 0.01 }), options);

    Assert.Equal(0.2, meter.DurationSeconds, 10);
    Assert.Equal(2 * Math.Sqrt(0.01 / 3.0), radians.DurationSeconds, 10);
    Assert.Equal(2 * Math.Sqrt(0.09 / 3.0), together.DurationSeconds, 10);
  }

  [Theory]
  [InlineData(RetimerAlgorithm.Totg)]
  [InlineData(RetimerAlgorithm.TotgLite)]
  [InlineData(RetimerAlgorithm.Bottleneck)]
  [InlineData(RetimerAlgorithm.SegmentTrapezoid)]
  public void MixedUnits_SwitchingMovingAxis_RespectsEverySegmentVelocity(RetimerAlgorithm algorithm)
  {
    var limits = new[] {
      JointLimit.Radians(-2, 2, maxVelocity: 10, maxAcceleration: 1),
      JointLimit.Meters(0, 1, maxVelocity: 0.001, maxAcceleration: 1)
    };
    var geometric = MakeTrajectory(limits, new[] { 0.0, 0.0 }, new[] { 1.0, 0.0 }, new[] { 1.0, 0.01 });
    var retimed = TrajectoryRetimer.Retime(geometric, new TrajectoryRetimerOptions { Algorithm = algorithm });

    for (var i = 1; i < retimed.Points.Count; i++)
    {
      var dt = retimed.Points[i].TimeSeconds - retimed.Points[i - 1].TimeSeconds;
      Assert.True(double.IsFinite(dt) && dt > 0);
      for (var j = 0; j < limits.Length; j++)
      {
        var dq = Math.Abs(retimed.Points[i].JointState.Positions[j] - retimed.Points[i - 1].JointState.Positions[j]);
        Assert.True(dq / dt <= limits[j].MaxVelocity!.Value + 1e-9,
          $"{algorithm}, segment {i}, joint {j}: {dq / dt} > {limits[j].MaxVelocity}");
      }
    }
  }

  [Theory]
  [InlineData(RetimerAlgorithm.Totg)]
  [InlineData(RetimerAlgorithm.TotgLite)]
  [InlineData(RetimerAlgorithm.Bottleneck)]
  public void RepeatedWaypoints_PreserveStartAndEndRestConstraints(RetimerAlgorithm algorithm)
  {
    var limits = new[] { JointLimit.Meters(0, 1) };
    var geometric = MakeTrajectory(limits,
      new[] { 0.0 }, new[] { 0.0 }, new[] { 0.0 }, new[] { 0.01 }, new[] { 0.01 }, new[] { 0.01 });
    var retimed = TrajectoryRetimer.Retime(geometric, new TrajectoryRetimerOptions { Algorithm = algorithm });

    Assert.Equal(geometric.Points.Count, retimed.Points.Count);
    Assert.Equal(0.2, retimed.DurationSeconds, 10);
    Assert.All(retimed.Points, p => Assert.True(double.IsFinite(p.TimeSeconds)));
    Assert.Equal(0, retimed.Points[2].TimeSeconds);
    Assert.Equal(retimed.Points[3].TimeSeconds, retimed.Points[^1].TimeSeconds);
  }

  [Theory]
  [InlineData(RetimerAlgorithm.Totg)]
  [InlineData(RetimerAlgorithm.TotgLite)]
  [InlineData(RetimerAlgorithm.Bottleneck)]
  [InlineData(RetimerAlgorithm.SegmentTrapezoid)]
  public void ConfiguredMeterLimits_OverrideDefaults(RetimerAlgorithm algorithm)
  {
    var geometric = MakeTrajectory(new[] { JointLimit.Meters(0, 3, maxVelocity: 0.2, maxAcceleration: 0.4) },
      new[] { 0.0 }, new[] { 2.0 });
    var options = new TrajectoryRetimerOptions {
      Algorithm = algorithm,
      DefaultMaxVelocityMetersPerSecond = double.NaN,
      DefaultMaxAccelerationMetersPerSecondSquared = double.NaN
    };
    var retimed = TrajectoryRetimer.Retime(geometric, options);

    Assert.Equal(10.5, retimed.DurationSeconds, 10);
  }

  public static IEnumerable<object[]> InvalidDynamicLimits()
  {
    foreach (var algorithm in Enum.GetValues<RetimerAlgorithm>())
    foreach (var value in new[] { double.NaN, double.PositiveInfinity, 0.0, -1.0 })
    foreach (var isVelocity in new[] { true, false })
      yield return new object[] { algorithm, value, isVelocity };
  }

  [Theory]
  [MemberData(nameof(InvalidDynamicLimits))]
  public void InvalidDynamicLimits_AreRejectedWithJointAndQuantity(
    RetimerAlgorithm algorithm, double value, bool isVelocity)
  {
    var limit = JointLimit.Meters(0, 1,
      maxVelocity: isVelocity ? value : 0.5,
      maxAcceleration: isVelocity ? 1.0 : value);
    var geometric = MakeTrajectory(new[] { limit }, new[] { 0.0 }, new[] { 0.01 });

    var exception = Assert.Throws<InvalidOperationException>(() =>
      TrajectoryRetimer.Retime(geometric, new TrajectoryRetimerOptions { Algorithm = algorithm }));

    Assert.Contains($"joint 1 {(isVelocity ? "velocity" : "acceleration")}", exception.Message);
    Assert.Contains("finite and positive", exception.Message);
  }

  [Theory]
  [InlineData(RetimerAlgorithm.Totg)]
  [InlineData(RetimerAlgorithm.TotgLite)]
  [InlineData(RetimerAlgorithm.Bottleneck)]
  [InlineData(RetimerAlgorithm.SegmentTrapezoid)]
  public void InvalidMeterDefaults_AreRejected(RetimerAlgorithm algorithm)
  {
    var geometric = MakeTrajectory(new[] { JointLimit.Meters(0, 1) }, new[] { 0.0 }, new[] { 0.01 });
    var exception = Assert.Throws<InvalidOperationException>(() =>
      TrajectoryRetimer.Retime(geometric, new TrajectoryRetimerOptions {
        Algorithm = algorithm,
        DefaultMaxVelocityMetersPerSecond = double.NaN
      }));
    Assert.Contains("joint 1 velocity", exception.Message);
  }

  [Theory]
  [InlineData(RetimerAlgorithm.Totg)]
  [InlineData(RetimerAlgorithm.TotgLite)]
  [InlineData(RetimerAlgorithm.Bottleneck)]
  [InlineData(RetimerAlgorithm.SegmentTrapezoid)]
  public void ZeroAxisBodyTrajectory_PreservesBodyPosesAndClock(RetimerAlgorithm algorithm)
  {
    var robot = new RobotModel(new RobotPreset {
      Manufacturer = RobotManufacturer.Unknown,
      ModelName = "free_flyer_retimer_test",
      Family = Units.AerialFamily,
      AxisCount = 0,
      JointLimits = Array.Empty<JointLimit>()
    });
    var goal = new BaseFrame(new Frame(0.5, -0.2, 1.0));
    var geometric = new Trajectory(robot, new[] {
      new TrajectoryPoint(0, new JointState(Array.Empty<double>()), baseFrameOverride: new BaseFrame(Frame.Identity)),
      new TrajectoryPoint(2, new JointState(Array.Empty<double>()), baseFrameOverride: goal)
    });

    var retimed = TrajectoryRetimer.Retime(geometric, new TrajectoryRetimerOptions { Algorithm = algorithm });

    // A joint-only retimer must not invent limits or replace a body's planned clock.
    Assert.Equal(2, retimed.DurationSeconds);
    Assert.Same(goal, retimed.Points[^1].BaseFrameOverride);
  }

  [Theory]
  [InlineData(JointCoordinateUnit.Meters)]
  [InlineData(JointCoordinateUnit.Radians)]
  public void SegmentSpacing_UsesUnitAppropriateJerkSettingInSeconds(JointCoordinateUnit unit)
  {
    var limit = new JointLimit(-1, 1, unit, maxVelocity: 0.5, maxAcceleration: 1.0);
    var geometric = MakeTrajectory(new[] { limit }, new[] { 0.0 }, new[] { 0.01 }, new[] { 0.0 });
    var retimed = TrajectoryRetimer.Retime(geometric, new TrajectoryRetimerOptions {
      Algorithm = RetimerAlgorithm.SegmentTrapezoid,
      DefaultMaxJerkMetersPerSecondCubed = unit == JointCoordinateUnit.Meters ? 0.1 : 1e-9,
      DefaultMaxJerkRadiansPerSecondCubed = unit == JointCoordinateUnit.Radians ? 0.1 : 1e-9
    });

    // Base segments each take 0.2 s, with average velocities +0.05 and -0.05 units/s.
    // The heuristic estimates jerk as Δv/dt², giving sqrt(0.1 / 0.1) = 1 s spacing.
    Assert.Equal(1.2, retimed.DurationSeconds, 10);
  }

  [Theory]
  [InlineData(double.NaN)]
  [InlineData(double.PositiveInfinity)]
  [InlineData(0.0)]
  [InlineData(-1.0)]
  public void SegmentSpacing_InvalidMeterJerk_IsRejected(double value)
  {
    var geometric = MakeTrajectory(new[] { JointLimit.Meters(0, 1) }, new[] { 0.0 }, new[] { 0.01 });
    var exception = Assert.Throws<InvalidOperationException>(() =>
      TrajectoryRetimer.Retime(geometric, new TrajectoryRetimerOptions {
        Algorithm = RetimerAlgorithm.SegmentTrapezoid,
        DefaultMaxJerkMetersPerSecondCubed = value
      }));

    Assert.Contains("joint 1 jerk", exception.Message);
    Assert.Contains("finite and positive", exception.Message);
  }

  private static Trajectory MakeTrajectory(IReadOnlyList<JointLimit> limits, params double[][] positions)
  {
    var robot = new RobotModel(new RobotPreset {
      Manufacturer = RobotManufacturer.Unknown,
      ModelName = "mixed_unit_retimer_test",
      Family = "test",
      AxisCount = limits.Count,
      JointLimits = limits
    });
    return new Trajectory(robot, positions.Select((q, i) => new TrajectoryPoint(i * 0.01, new JointState(q))).ToArray());
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
