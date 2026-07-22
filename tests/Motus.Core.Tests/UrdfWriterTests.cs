using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class UrdfWriterTests
{
    private static RobotDescription SmallArm() =>
        RobotDescription.Assemble(
            "box_arm",
            [
                new UrdfLink("base_link", visuals: [UrdfGeometry.Box(0.1, 0.1, 0.05)]),
                new UrdfLink("link1", visuals: [UrdfGeometry.Box(0.05, 0.05, 0.2)]),
            ],
            [
                new UrdfJoint("j0", "revolute", "base_link", "link1",
                    0, 0, 0.05, 0, 0, 1, -Math.PI, Math.PI),
            ],
            tipLink: "link1");

    [Fact]
    public void Write_RoundTrip_LoadTree_DriverCountAndLinks()
    {
        var desc = SmallArm();
        var dir = Path.Combine(Path.GetTempPath(), "motus_urdf_writer_" + Guid.NewGuid().ToString("N"));
        try
        {
            var path = UrdfWriter.Write(desc, dir);
            Assert.True(File.Exists(path));

            var tree = UrdfRobotLoader.LoadTree(path);
            Assert.Equal(1, tree.DriverCount);
            Assert.Contains(tree.Links, l => l.Name == "base_link");
            Assert.Contains(tree.Links, l => l.Name == "link1");
        }
        finally
        {
            if (Directory.Exists(dir))
                Directory.Delete(dir, recursive: true);
        }
    }

    [Fact]
    public void Write_MimicPreserved()
    {
        var gripper = RobotDescription.Assemble(
            "grip",
            [new UrdfLink("palm"), new UrdfLink("L"), new UrdfLink("R")],
            [
                new UrdfJoint("j_left", "revolute", "palm", "L", 0, 0, 0, 0, 0, 1, 0, 0.8),
                new UrdfJoint("j_right", "revolute", "palm", "R", 0, 0, 0, 0, 0, 1, 0, 0.8,
                    mimicJoint: "j_left", mimicMultiplier: -1),
            ]);

        var dir = Path.Combine(Path.GetTempPath(), "motus_urdf_mimic_" + Guid.NewGuid().ToString("N"));
        try
        {
            var path = UrdfWriter.Write(gripper, dir);
            var tree = UrdfRobotLoader.LoadTree(path);
            Assert.Equal(1, tree.DriverCount);
            var follower = tree.Joints.Single(j => j.Name == "j_right");
            Assert.NotNull(follower.Mimic);
            Assert.Equal(-1, follower.Mimic!.Value.Multiplier);
        }
        finally
        {
            if (Directory.Exists(dir))
                Directory.Delete(dir, recursive: true);
        }
    }

    [Fact]
    public void Sanitize_MakesSafeNames()
    {
        Assert.Equal("foo_bar", UrdfName.Sanitize("foo/bar"));
        Assert.Equal("_9bad", UrdfName.Sanitize("9bad"));
    }

    [Fact]
    public void Write_SanitizedFileName_StaysInsideOutputDirectory()
    {
        var desc = SmallArm();
        var dir = Path.Combine(Path.GetTempPath(), "motus_urdf_safe_" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(dir);
        try
        {
            // Sanitize turns "../escape" into safe underscores — file must land under dir.
            var path = UrdfWriter.Write(desc, dir, fileName: "../escape");
            Assert.StartsWith(Path.GetFullPath(dir), Path.GetFullPath(path), StringComparison.OrdinalIgnoreCase);
            Assert.DoesNotContain("..", Path.GetFileName(path), StringComparison.Ordinal);
        }
        finally
        {
            if (Directory.Exists(dir))
                Directory.Delete(dir, recursive: true);
        }
    }

    [Fact]
    public void Write_MeshSidecar_CreatesStlAndRelativeFilename()
    {
        var verts = new[] { new[] { 0.0, 0.0, 0.0 }, new[] { 1.0, 0.0, 0.0 }, new[] { 0.0, 1.0, 0.0 } };
        var indices = new[] { 0, 1, 2 };
        var desc = RobotDescription.Assemble(
            "mesh_bot",
            [new UrdfLink("base_link", visuals: [UrdfGeometry.Mesh(verts, indices)])],
            []);

        var dir = Path.Combine(Path.GetTempPath(), "motus_urdf_mesh_" + Guid.NewGuid().ToString("N"));
        try
        {
            var path = UrdfWriter.Write(desc, dir, writeMeshes: true);
            var xml = File.ReadAllText(path);
            Assert.Contains("meshes/base_link_0.stl", xml, StringComparison.Ordinal);
            Assert.True(File.Exists(Path.Combine(dir, "meshes", "base_link_0.stl")));
        }
        finally
        {
            if (Directory.Exists(dir))
                Directory.Delete(dir, recursive: true);
        }
    }

    [Fact]
    public void Write_RejectsUnsafeMeshFilePath()
    {
        var verts = new[] { new[] { 0.0, 0.0, 0.0 }, new[] { 1.0, 0.0, 0.0 }, new[] { 0.0, 1.0, 0.0 } };
        var indices = new[] { 0, 1, 2 };
        var desc = RobotDescription.Assemble(
            "unsafe",
            [new UrdfLink("base_link", visuals: [UrdfGeometry.Mesh(verts, indices, filePath: "../escape.stl")])],
            []);

        var dir = Path.Combine(Path.GetTempPath(), "motus_urdf_unsafe_" + Guid.NewGuid().ToString("N"));
        try
        {
            var ex = Assert.Throws<InvalidOperationException>(() =>
                UrdfWriter.Write(desc, dir, writeMeshes: false));
            Assert.Contains("relative path", ex.Message, StringComparison.OrdinalIgnoreCase);
        }
        finally
        {
            if (Directory.Exists(dir))
                Directory.Delete(dir, recursive: true);
        }
    }

    [Fact]
    public void Write_WriteMeshesFalse_SkipsSidecarDir()
    {
        var verts = new[] { new[] { 0.0, 0.0, 0.0 }, new[] { 1.0, 0.0, 0.0 }, new[] { 0.0, 1.0, 0.0 } };
        var indices = new[] { 0, 1, 2 };
        var desc = RobotDescription.Assemble(
            "mesh_bot",
            [new UrdfLink("base_link", visuals: [UrdfGeometry.Mesh(verts, indices)])],
            []);

        var dir = Path.Combine(Path.GetTempPath(), "motus_urdf_nomesh_" + Guid.NewGuid().ToString("N"));
        try
        {
            UrdfWriter.Write(desc, dir, writeMeshes: false);
            Assert.False(Directory.Exists(Path.Combine(dir, "meshes")));
        }
        finally
        {
            if (Directory.Exists(dir))
                Directory.Delete(dir, recursive: true);
        }
    }
}
