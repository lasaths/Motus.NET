using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class ToolDefinitionTests
{
    [Fact]
    public void WithTool_OverridesTcpAndGeometry()
    {
        var preset = PresetLoader.LoadRobotModelByName("UR5e");
        var tool = new ToolDefinition(
            "gripper",
            new Frame(0, 0, 0.12, 1, 0, 0, 0),
            CollisionObject.Box("gripper", Frame.Identity, 0.02, 0.02, 0.03));

        var session = preset.WithTool(tool);
        Assert.Equal("gripper", session.Preset.ToolFrame.Name);
        Assert.Equal(0.12, session.Preset.ToolFrame.Frame.Z, 3);
        Assert.NotNull(session.CollisionModel?.ToolGeometry);
        Assert.Equal(CollisionShape.Box, session.CollisionModel!.ToolGeometry!.Shape);
    }

    [Fact]
    public void WithTool_CollisionCheckerUsesSessionTcp()
    {
        var preset = PresetLoader.LoadRobotModelByName("UR5e");
        var tool = new ToolDefinition(
            "offset",
            new Frame(0, 0, 0.1, 1, 0, 0, 0),
            CollisionObject.Sphere("gripper", Frame.Identity, 0.03));
        var session = preset.WithTool(tool);
        var fk = KinematicsResolver.CreateFkSolver(session.Preset);
        var home = new JointState(new double[] { 0, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });

        var obstacle = CollisionObject.Sphere("obs", fk.ComputeTcp(home, session.Preset.BaseFrame, session.Preset.ToolFrame).Tcp, 0.02);
        var scene = new CollisionScene(new[] { obstacle });
        var checker = CollisionCheckerFactory.Create(session);
        Assert.False(checker.IsCollisionFree(home, scene));
    }

    [Fact]
    public void WithTool_ChangesTcpPosition()
    {
        var preset = PresetLoader.LoadRobotModelByName("UR5e");
        var tool = new ToolDefinition("ext", new Frame(0, 0, 0.08, 1, 0, 0, 0));
        var session = preset.WithTool(tool);
        var fk = KinematicsResolver.CreateFkSolver(session.Preset);
        var home = new JointState(new double[] { 0, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });
        var presetTcp = fk.ComputeTcp(home, preset.Preset.BaseFrame, preset.Preset.ToolFrame).Tcp;
        var sessionTcp = fk.ComputeTcp(home, session.Preset.BaseFrame, session.Preset.ToolFrame).Tcp;
        var dz = sessionTcp.Z - presetTcp.Z;
        var dist = Math.Sqrt(
            Math.Pow(sessionTcp.X - presetTcp.X, 2) +
            Math.Pow(sessionTcp.Y - presetTcp.Y, 2) +
            Math.Pow(sessionTcp.Z - presetTcp.Z, 2));
        Assert.True(dist > 0.07, $"expected TCP offset, dz={dz:F3} dist={dist:F3}");
    }

    [Fact]
    public void Export_IncludesToolFrameWhenSessionDiffers()
    {
        var preset = PresetLoader.LoadRobotModelByName("UR5e");
        var tool = new ToolDefinition("gripper", new Frame(0, 0, 0.05, 1, 0, 0, 0));
        var session = preset.WithTool(tool);
        var traj = new Trajectory(session, new[] { new TrajectoryPoint(0, new JointState(new double[6])) });
        var json = TrajectoryExport.ToJson(traj, new TrajectoryExportOptions
        {
            SessionToolFrame = session.Preset.ToolFrame
        });
        Assert.Contains("\"toolFrame\"", json);
        Assert.Contains("\"gripper\"", json);
        Assert.Contains("0.05", json);
    }

    [Fact]
    public void Export_OmitsToolFrameWhenDefault()
    {
        var preset = PresetLoader.LoadRobotModelByName("UR5e");
        var traj = new Trajectory(preset, new[] { new TrajectoryPoint(0, new JointState(new double[6])) });
        var json = TrajectoryExport.ToJson(traj, new TrajectoryExportOptions());
        Assert.DoesNotContain("\"toolFrame\"", json);
    }
}
