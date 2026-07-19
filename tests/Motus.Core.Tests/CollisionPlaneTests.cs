using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class CollisionPlaneTests
{
    /// <summary>WorldXY floor: Motus local +X = world +Z (same remap as GH FrameConversion.FromPlane).</summary>
    private static Frame WorldXyFloor(double z = 0)
    {
        // Motus X = world Z, Motus Y = world X, Motus Z = world Y
        var m = new double[]
        {
            0, 1, 0, 0,
            0, 0, 1, 0,
            1, 0, 0, z,
            0, 0, 0, 1
        };
        return Transforms.ToFrame(m);
    }

    [Fact]
    public void HalfSpace_SignedDistance_UsesMotusLocalX()
    {
        var floor = CollisionObject.Plane("floor", WorldXyFloor());
        Assert.True(SphereHitsPlane(new Frame(0, 0, -0.05), 0.01, floor));
        Assert.False(SphereHitsPlane(new Frame(0, 0, 0.05), 0.01, floor));
        Assert.True(SphereHitsPlane(new Frame(0, 0, 0.0), 0.01, floor)); // on plane within radius
    }

    [Fact]
    public void Scene_AutoAllowsBaseLinks_AgainstPlanes()
    {
        var scene = new CollisionScene(new[] { CollisionObject.Plane("floor", WorldXyFloor()) });
        Assert.True(scene.IsPairAllowed(CollisionBodies.RobotLink(-1), "floor"));
        Assert.True(scene.IsPairAllowed(CollisionBodies.RobotLink(0), "floor"));
        Assert.True(scene.IsPairAllowed(CollisionBodies.RobotLink(1), "floor"));
        Assert.False(scene.IsPairAllowed(CollisionBodies.RobotLink(2), "floor"));
    }

    [Fact]
    public void UprightHome_WithCoLocatedFloor_IsCollisionFree()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", FindResources());
        var checker = new SphereCollisionChecker(preset);
        var floor = CollisionObject.Plane("floor", WorldXyFloor());
        var scene = new CollisionScene(new[] { floor });
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        Assert.True(checker.IsCollisionFree(home, scene));
    }

    [Fact]
    public void DistalLink_BelowFloor_IsInCollision()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", FindResources());
        var checker = new SphereCollisionChecker(preset);
        var scene = new CollisionScene(new[] { CollisionObject.Plane("floor", WorldXyFloor()) });
        // Zero joints drive distal envelopes through Z=0 on UR5e
        Assert.False(checker.IsCollisionFree(new JointState(new double[6]), scene));
    }

    private static bool SphereHitsPlane(Frame center, double radius, CollisionObject plane)
    {
        var m = Transforms.FromFrame(plane.Pose);
        var nx = m[0]; var ny = m[4]; var nz = m[8];
        var signed = (center.X - plane.Pose.X) * nx + (center.Y - plane.Pose.Y) * ny + (center.Z - plane.Pose.Z) * nz;
        return signed < radius;
    }

    private static string FindResources()
    {
        var dir = AppContext.BaseDirectory;
        for (var i = 0; i < 8 && dir is not null; i++)
        {
            var candidate = Path.Combine(dir, "resources", "robots");
            if (Directory.Exists(candidate)) return candidate;
            dir = Directory.GetParent(dir)?.FullName;
        }
        throw new DirectoryNotFoundException("resources/robots not found");
    }
}
