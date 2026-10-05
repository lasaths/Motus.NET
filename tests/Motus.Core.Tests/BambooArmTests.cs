using Motus.Core;
using Motus.Geometry;
using Motus.Presets;
using System.Xml.Linq;

namespace Motus.Core.Tests;

/// <summary>
/// Tests for the Bamboo ICD 5-DoF arm model, FK, collision, and task validation.
/// Validates that the Blazor viewer's C# backend correctly handles kinematics and collision.
/// </summary>
public class BambooArmTests
{
    private static string FixturesRoot =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "tests", "fixtures"));

    [Fact]
    public void BambooArm_LoadsFromUrdf()
    {
        var urdfPath = Path.Combine(FixturesRoot, "bamboo_icd", "bamboo_icd.urdf");
        var robot = UrdfRobotLoader.Load(urdfPath, new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tool0"
        });

        Assert.Equal(5, robot.Preset.AxisCount);
        Assert.Equal(5, robot.JointNames.Count);
        Assert.Equal(5, robot.Preset.JointLimits.Count);
    }

    [Fact]
    public void BambooArm_ForwardKinematics_HomePosition()
    {
        var urdfPath = Path.Combine(FixturesRoot, "bamboo_icd", "bamboo_icd.urdf");
        var robot = UrdfRobotLoader.Load(urdfPath);
        var fk = KinematicsResolver.CreateFkSolver(robot.Preset, robot.Chain);

        var homeState = new JointState(new double[5]);
        var linkOrigins = fk.ComputeLinkOrigins(homeState.Positions, Frame.Identity);

        Assert.NotEmpty(linkOrigins);
        
        var tcp = fk.ComputeTcp(homeState, BaseFrame.Identity, robot.Preset.ToolFrame);
        Assert.True(tcp.Tcp.Z > 0, "TCP should be above ground at home position");
    }

    [Fact]
    public void BambooArm_JointLimits_MatchSpec()
    {
        var urdfPath = Path.Combine(FixturesRoot, "bamboo_icd", "bamboo_icd.urdf");
        var robot = UrdfRobotLoader.Load(urdfPath);

        var limits = robot.Preset.JointLimits;
        Assert.Equal(5, limits.Count);

        Assert.InRange(limits[0].Min, -3.0, -2.9);
        Assert.InRange(limits[0].Max, 2.9, 3.0);
        
        Assert.InRange(limits[1].Min, -1.6, -1.5);
        Assert.InRange(limits[1].Max, 1.5, 1.6);
        
        Assert.InRange(limits[2].Min, -2.4, -2.3);
        Assert.InRange(limits[2].Max, 2.3, 2.4);
    }

    [Fact]
    public void BambooArm_CollisionCheck_NoSelfCollision()
    {
        var urdfPath = Path.Combine(FixturesRoot, "bamboo_icd", "bamboo_icd.urdf");
        var robot = UrdfRobotLoader.Load(urdfPath);
        var checker = new SphereCollisionChecker(robot.Preset, robot.Chain);

        var homeState = new JointState(new double[5]);
        var scene = new CollisionScene();

        Assert.True(checker.IsCollisionFree(homeState, scene));
    }

    [Fact]
    public void BambooArm_TaskPath_SimpleReachableFrames_ValidatesSuccessfully()
    {
        var urdfPath = Path.Combine(FixturesRoot, "bamboo_icd", "bamboo_icd.urdf");
        var robot = UrdfRobotLoader.Load(urdfPath);
        var model = robot.ToModel();
        var ik = KinematicsResolver.CreateInverseKinematics(robot.Preset, robot.Chain);
        var checker = new SphereCollisionChecker(robot.Preset, robot.Chain);
        var fk = KinematicsResolver.CreateFkSolver(robot.Preset, robot.Chain);

        var seed = new JointState(new double[] { 0, 0.5, 0, 0.5, 0 });
        var forwardTcp = fk.ComputeTcp(seed, BaseFrame.Identity, robot.Preset.ToolFrame);
        
        var reachableFrame = forwardTcp.Tcp;
        var task = new TaskInstance("test",
            new Dictionary<string, Frame>
            {
                ["target"] = reachableFrame
            },
            new Dictionary<string, object>
            {
                ["description"] = "Forward-kinematics reachable frame"
            });

        var path = new TaskPath(task);

        var scene = new CollisionScene();

        var result = TaskPathValidator.Validate(path, model, ik, checker, scene, seed: seed);

        if (!result.IsValid)
        {
            var errors = string.Join("\n  ", result.Errors);
            Assert.Fail($"Task path should be valid but got errors:\n  {errors}");
        }

        Assert.True(result.IsValid);
        Assert.Empty(result.Errors);
        Assert.NotEmpty(result.ReachableStates);
    }

    [Fact]
    public void BambooArm_TaskPath_CollidingPath_FailsValidation()
    {
        var urdfPath = Path.Combine(FixturesRoot, "bamboo_icd", "bamboo_icd.urdf");
        var robot = UrdfRobotLoader.Load(urdfPath);
        var model = robot.ToModel();
        var ik = KinematicsResolver.CreateInverseKinematics(robot.Preset, robot.Chain);
        var checker = new SphereCollisionChecker(robot.Preset, robot.Chain);

        var collisionFrame = new Frame(0.15, 0, 0.15, qw: 1, qx: 0, qy: 0, qz: 0);
        var task = new TaskInstance("collision_test",
            new Dictionary<string, Frame> { ["target"] = collisionFrame });

        var path = new TaskPath(task);

        var obstacle = CollisionObject.Box("big_box", new Frame(0.15, 0, 0.15), 0.2, 0.2, 0.2);
        var scene = new CollisionScene(new[] { obstacle });

        var result = TaskPathValidator.Validate(path, model, ik, checker, scene);

        Assert.False(result.IsValid, "Path should fail validation due to collision");
        Assert.NotEmpty(result.Errors);
    }

    [Fact]
    public void BambooArm_MeshCollisionChecker_Works()
    {
        var urdfPath = Path.Combine(FixturesRoot, "bamboo_icd", "bamboo_icd.urdf");
        var robot = UrdfRobotLoader.Load(urdfPath);
        var checker = new RobotMeshCollisionChecker(robot.ToModel(), robot.Chain);

        var homeState = new JointState(new double[5]);
        var scene = new CollisionScene();

        Assert.True(checker.IsCollisionFree(homeState, scene));
    }
}
