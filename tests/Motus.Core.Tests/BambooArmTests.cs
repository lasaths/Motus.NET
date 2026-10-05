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
        var robot = UrdfRobotLoader.Load(urdfPath, new UrdfLoadOptions { BaseLink = "base_link", TipLink = "tool0" });
        var fk = KinematicsResolver.CreateFkSolver(robot.Preset, robot.Chain);

        // Arm hangs under the pole (Motus Z = 0.40). Viewer home pose keeps the jaw above ground.
        var home = new JointState(Motus.Viewer.Services.BambooIcdCell.RadFromDeg(Motus.Viewer.Services.BambooIcdMotion.HomeDeg));
        var linkOrigins = fk.ComputeLinkOrigins(home.Positions, Frame.Identity);
        Assert.NotEmpty(linkOrigins);

        var tcp = fk.ComputeTcp(home, BaseFrame.Identity, robot.Preset.ToolFrame);
        Assert.True(tcp.Tcp.Z > 0, "TCP should be above ground at the viewer home pose");

        // Straight down (all zeros) the 0.42 m chain reaches 2 cm below the ground: Motus FK says so too.
        var down = fk.ComputeTcp(new JointState(new double[5]), BaseFrame.Identity, robot.Preset.ToolFrame);
        Assert.Equal(0.40 - 0.42, down.Tcp.Z, 9);
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
        var checker = new RobotMeshCollisionChecker(robot.ToModel(), robot.Chain, null,
            Motus.Viewer.Services.BambooIcdCell.SelfCollisionMinLinkGap);

        var homeState = new JointState(Motus.Viewer.Services.BambooIcdCell.RadFromDeg(Motus.Viewer.Services.BambooIcdMotion.HomeDeg));
        var scene = new CollisionScene();

        Assert.True(checker.IsCollisionFree(homeState, scene));
    }

    [Fact]
    public void BambooArm_TaskPath_SimpleReachableFrames_ValidatesSuccessfully()
    {
        var urdfPath = Path.Combine(FixturesRoot, "bamboo_icd", "bamboo_icd.urdf");
        var robot = UrdfRobotLoader.Load(urdfPath);
        var model = robot.ToModel();
        IInverseKinematics ik = new Motus.Viewer.Services.BambooIcdIk();
        var checker = new RobotMeshCollisionChecker(model, robot.Chain, null, Motus.Viewer.Services.BambooIcdCell.SelfCollisionMinLinkGap);
        var fk = KinematicsResolver.CreateFkSolver(robot.Preset, robot.Chain);

        // In the arm's working plane (θ1 = -90°, θ5 = 0) — the planar IK branch family the viewer plays.
        var seed = new JointState(Motus.Viewer.Services.BambooIcdCell.RadFromDeg(new double[] { -90, -40, -50, 60, 0 }));
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
        IInverseKinematics ik = new Motus.Viewer.Services.BambooIcdIk();
        var checker = new RobotMeshCollisionChecker(model, robot.Chain, null, Motus.Viewer.Services.BambooIcdCell.SelfCollisionMinLinkGap);

        // A reachable pick frame (the default grasp plane) with an obstacle sitting on it.
        var target = Motus.Viewer.Services.BambooIcdMotion.PlaneToMotusFrame(0, 0.00778, 0.09194, 0);
        var task = new TaskInstance("collision_test",
            new Dictionary<string, Frame> { ["target"] = target });
        var path = new TaskPath(task);

        var obstacle = CollisionObject.Box("big_box", new Frame(target.X, target.Y, target.Z + 0.02), 0.03, 0.03, 0.03);
        var scene = new CollisionScene(new[] { obstacle });

        var result = TaskPathValidator.Validate(path, model, ik, checker, scene);

        Assert.False(result.IsValid, "Path should fail validation due to collision");
        Assert.NotEmpty(result.Errors);
        Assert.Contains(result.Errors, e => e.Contains("collides"));

        // Same frame without the obstacle is valid.
        Assert.True(TaskPathValidator.Validate(path, model, ik, checker, new CollisionScene()).IsValid);
    }

    [Fact]
    public void BambooArm_MeshCollisionChecker_Works()
    {
        var urdfPath = Path.Combine(FixturesRoot, "bamboo_icd", "bamboo_icd.urdf");
        var robot = UrdfRobotLoader.Load(urdfPath);
        var checker = new RobotMeshCollisionChecker(robot.ToModel(), robot.Chain);

        var homeState = new JointState(Motus.Viewer.Services.BambooIcdCell.RadFromDeg(Motus.Viewer.Services.BambooIcdMotion.HomeDeg));
        var scene = new CollisionScene();

        Assert.True(checker.IsCollisionFree(homeState, scene));
    }
}
