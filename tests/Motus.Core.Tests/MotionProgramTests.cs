using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class MotionProgramTests
{
    private static string ResourcesRoot =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

    [Fact]
    public void MixedProgram_PtpLinCirc_ProducesTrajectoryWithMetadata()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var fk = new DhForwardKinematics(preset);
        var planner = new IndustrialMotionPlanner(preset);

        var start = new JointState(new[] { 0.0, -0.5, 1.0, -1.0, 0.0, 0.0 });
        var ptpGoal = new JointState(start.Positions);
        var afterPtpPose = fk.ComputeTcp(ptpGoal, preset.BaseFrame, preset.ToolFrame);
        var linGoal = new CartesianPose(new Frame(
            afterPtpPose.Tcp.X + 0.006, afterPtpPose.Tcp.Y, afterPtpPose.Tcp.Z,
            afterPtpPose.Tcp.Qw, afterPtpPose.Tcp.Qx, afterPtpPose.Tcp.Qy, afterPtpPose.Tcp.Qz));
        var circVia = new CartesianPose(new Frame(
            linGoal.Tcp.X + 0.003, linGoal.Tcp.Y + 0.002, linGoal.Tcp.Z,
            linGoal.Tcp.Qw, linGoal.Tcp.Qx, linGoal.Tcp.Qy, linGoal.Tcp.Qz));
        var circGoal = new CartesianPose(new Frame(
            linGoal.Tcp.X, linGoal.Tcp.Y + 0.004, linGoal.Tcp.Z,
            linGoal.Tcp.Qw, linGoal.Tcp.Qx, linGoal.Tcp.Qy, linGoal.Tcp.Qz));

        var req = new MotionProgramRequest(
            robot,
            start,
            new MotionSegment[]
            {
                new PtpSegment(ptpGoal, blendRadiusMeters: 0.004),
                new LinSegment(linGoal, stepMeters: 0.005, blendRadiusMeters: 0.003),
                new CircSegment(circVia, circGoal, arcSamples: 10)
            });

        var result = planner.Plan(req);

        Assert.True(result.Success, string.Join("; ", result.Errors));
        Assert.NotNull(result.Trajectory);
        Assert.True(result.Trajectory!.Points.Count > 6);
        Assert.DoesNotContain(result.Warnings, w => w.Contains("fallback to exact-stop transition", StringComparison.OrdinalIgnoreCase));
        Assert.Contains(result.Trajectory.Points, p => p.MotionType == MotionPrimitiveType.Ptp);
        Assert.Contains(result.Trajectory.Points, p => p.MotionType == MotionPrimitiveType.Lin);
        Assert.Contains(result.Trajectory.Points, p => p.MotionType == MotionPrimitiveType.Circ);
    }

    [Fact]
    public void Blend_InfeasibleRadius_WarnsExactStopFallback()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var fk = new DhForwardKinematics(preset);
        var planner = new IndustrialMotionPlanner(preset);

        var start = new JointState(new[] { 0.0, -0.5, 1.0, -1.0, 0.0, 0.0 });
        var tcp = fk.ComputeTcp(start, preset.BaseFrame, preset.ToolFrame);
        var shortGoal = new CartesianPose(new Frame(
            tcp.Tcp.X + 0.002, tcp.Tcp.Y, tcp.Tcp.Z,
            tcp.Tcp.Qw, tcp.Tcp.Qx, tcp.Tcp.Qy, tcp.Tcp.Qz));
        var farGoal = new CartesianPose(new Frame(
            tcp.Tcp.X + 0.02, tcp.Tcp.Y, tcp.Tcp.Z,
            tcp.Tcp.Qw, tcp.Tcp.Qx, tcp.Tcp.Qy, tcp.Tcp.Qz));

        var req = new MotionProgramRequest(
            robot,
            start,
            new MotionSegment[]
            {
                new LinSegment(shortGoal, stepMeters: 0.001, blendRadiusMeters: 0.01),
                new LinSegment(farGoal, stepMeters: 0.005)
            });

        var result = planner.Plan(req);
        Assert.True(result.Success);
        Assert.Contains(result.Warnings, w => w.Contains("fallback to exact-stop transition", StringComparison.OrdinalIgnoreCase));
    }

    [Fact]
    public void Export_IncludesMotionMetadataColumns()
    {
        var robot = new RobotModel(new RobotPreset
        {
            Manufacturer = RobotManufacturer.UniversalRobots,
            ModelName = "Test",
            AxisCount = 2,
            JointLimits = new[] { new JointLimit(-6.28, 6.28), new JointLimit(-6.28, 6.28) }
        });

        var traj = new Trajectory(robot, new[]
        {
            new TrajectoryPoint(0, new JointState(new[] { 0.0, 0.0 })),
            new TrajectoryPoint(0.2, new JointState(new[] { 0.2, 0.1 }), MotionPrimitiveType.Ptp, segmentIndex: 0, blendRadiusMeters: 0.002)
        });

        var json = TrajectoryExport.ToJson(traj);
        var csv = TrajectoryExport.ToCsv(traj);
        Assert.Contains("motionType", json);
        Assert.Contains("blendRadiusMeters", json);
        Assert.Contains("motion_type,segment_index,blend_radius_m", csv);
    }
}
