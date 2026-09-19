using System.Text.Json;
using Motus.Core;

namespace Motus.Core.Tests;

/// <summary>
/// Aerial / HolonomicSE3 export honesty — body SE(3) poses, never MoveJ radians.
/// </summary>
public class AerialExportTests
{
    private static RobotModel AerialRobot() =>
        new(new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = "free_flyer_export",
            Family = Units.AerialFamily,
            AxisCount = 0,
            JointLimits = Array.Empty<JointLimit>(),
            BaseFrame = new BaseFrame(new Frame(0, 0, 0.5))
        });

    [Fact]
    public void JsonExport_Aerial_UsesBodyPoseNotJointsRadians()
    {
        var robot = AerialRobot();
        var a = new MobilityModel.HolonomicSE3(0.1, -0.2, 0.8, 0.05, -0.1, 0.3);
        var b = new MobilityModel.HolonomicSE3(0.5, 0.1, 1.2, 0, 0, 0.5);
        var traj = new Trajectory(robot, new[]
        {
            new TrajectoryPoint(0.0, new JointState(Array.Empty<double>()), baseFrameOverride: new BaseFrame(a.BaseFrame)),
            new TrajectoryPoint(1.0, new JointState(Array.Empty<double>()), baseFrameOverride: new BaseFrame(b.BaseFrame))
        });

        var json = TrajectoryExport.ToJson(traj);
        using var doc = JsonDocument.Parse(json);
        var root = doc.RootElement;

        Assert.Equal(Units.AerialFamily, root.GetProperty("family").GetString());
        Assert.Equal("meters", root.GetProperty("units").GetProperty("bodyPosition").GetString());
        Assert.Equal("radians_rpy_fixed_xyz", root.GetProperty("units").GetProperty("bodyOrientation").GetString());
        Assert.Equal("not_ur_movej", root.GetProperty("units").GetProperty("waypointsQ").GetString());
        Assert.False(root.TryGetProperty("units", out var units) && units.TryGetProperty("jointAngles", out _));

        var points = root.GetProperty("points");
        Assert.Equal(2, points.GetArrayLength());
        var p0 = points[0].GetProperty("bodyPose");
        Assert.InRange(p0.GetProperty("x").GetDouble() - 0.1, -1e-9, 1e-9);
        Assert.InRange(p0.GetProperty("z").GetDouble() - 0.8, -1e-9, 1e-9);
        Assert.False(points[0].TryGetProperty("jointsRadians", out _));
    }

    [Fact]
    public void CsvExport_Aerial_UsesBodyColumnsNotJointRad()
    {
        var robot = AerialRobot();
        var pose = new MobilityModel.HolonomicSE3(0.2, 0.3, 1.0, 0.1, -0.05, 0.4);
        var traj = new Trajectory(robot, new[]
        {
            new TrajectoryPoint(0.0, new JointState(Array.Empty<double>()), baseFrameOverride: new BaseFrame(pose.BaseFrame))
        });

        var csv = TrajectoryExport.ToCsv(traj);
        Assert.Contains("body_x_m,body_y_m,body_z_m,body_roll_rad,body_pitch_rad,body_yaw_rad", csv);
        Assert.DoesNotContain("joint_1_rad", csv);
        Assert.DoesNotContain("jointsRadians", csv);
        Assert.Contains("0.200000", csv);
        Assert.Contains("1.000000", csv);
    }

    [Fact]
    public void SerialExport_Unchanged_StillUsesJointsRadians()
    {
        var robot = new RobotModel(new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = "serial_smoke",
            Family = "urdf",
            AxisCount = 1,
            JointLimits = new[] { JointLimit.Radians(-1, 1) }
        }, jointNames: new[] { "j1" });
        var traj = new Trajectory(robot, new[]
        {
            new TrajectoryPoint(0.0, new JointState(new[] { 0.25 }))
        });

        var json = TrajectoryExport.ToJson(traj);
        using var doc = JsonDocument.Parse(json);
        Assert.False(doc.RootElement.TryGetProperty("family", out _));
        Assert.Equal("radians", doc.RootElement.GetProperty("units").GetProperty("jointAngles").GetString());
        Assert.True(doc.RootElement.GetProperty("points")[0].TryGetProperty("jointsRadians", out _));
    }
}
