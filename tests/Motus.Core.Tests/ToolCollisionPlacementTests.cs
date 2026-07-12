using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class ToolCollisionPlacementTests
{
    [Fact]
    public void WithTool_RobotiqMesh_UsesFlangePlacementAndAttachOffset()
    {
        var tool = new ToolDefinition(
            "robotiq_2f85",
            new Frame(0, 0, 0.1633, 0.7071067811865476, 0, 0.7071067811865476, 0),
            CollisionObject.Mesh("robotiq_2f85", Frame.Identity, new List<double[]> { new[] { 0.0, 0.0, 0.05 } }, new List<int> { 0 }),
            ToolCapabilities.Robotiq2F85)
        {
            GeometryInFlangeFrame = true,
            GeometryAttachOffset = UrdfFixedChain.TryTipAttachOffset(FixtureUrdf(), "base_link", "tool0")
        };

        var preset = UrdfRobotLoader.Load(FixtureUrdf(), new UrdfLoadOptions { ModelName = "ur10e_robotiq" }).ToModel();
        var session = preset.WithTool(tool);

        Assert.True(session.CollisionModel?.ToolGeometryInFlangeFrame);
        Assert.NotNull(session.CollisionModel?.ToolGeometryAttachOffset);
    }

    [Fact]
    public void UrdfFixedChain_Ur10e_Wrist3ToTool0_IsNonIdentity()
    {
        var offset = UrdfFixedChain.TryTipAttachOffset(FixtureUrdf(), "base_link", "tool0");
        Assert.NotNull(offset);
        var o = offset!.Value;
        var identity = new Frame(0, 0, 0, 1, 0, 0, 0);
        Assert.False(
            Math.Abs(o.X - identity.X) < 1e-9 &&
            Math.Abs(o.Y - identity.Y) < 1e-9 &&
            Math.Abs(o.Z - identity.Z) < 1e-9 &&
            Math.Abs(o.Qw - identity.Qw) < 1e-9,
            "wrist_3 → tool0 offset should be non-identity");
    }

    [Fact]
    public void UsesFlangePlacement_DetectsBundledRobotiqName()
    {
        var geom = CollisionObject.Mesh("robotiq_2f85", Frame.Identity, new List<double[]> { new[] { 0.0, 0.0, 0.05 } }, new List<int> { 0 });
        Assert.True(ToolCollisionPlacement.UsesFlangePlacement(geom));
    }

    private static string FixtureUrdf() =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", "ur10e", "ur10e.urdf"));
}
