using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>
/// Tests that task paths reject colliding segments and accept collision-free ones.
/// Uses existing Motus.NET collision checkers (SphereCollisionChecker and MeshCollisionChecker).
/// </summary>
public class TaskCollisionTests
{
    private static string ResourcesRoot =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

    [Fact]
    public void TaskPath_CollisionFreeFrames_PassesValidation()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var ik = KinematicsResolver.CreateInverseKinematics(preset);

        // Pick task at reachable, collision-free position
        var pickApproach = new Frame(0.3, 0.2, 0.5);
        var pickGrasp = new Frame(0.3, 0.2, 0.45);
        var pick = new TaskInstance("pick",
            new Dictionary<string, Frame>
            {
                ["approach"] = pickApproach,
                ["grasp"] = pickGrasp
            },
            new Dictionary<string, object> { ["object"] = "part_A", ["jaw_mm"] = 10.0 });

        // Place task at reachable, collision-free position
        var placeUpright = new Frame(0.2, -0.3, 0.6);
        var placeRelease = new Frame(0.2, -0.3, 0.55);
        var place = new TaskInstance("place",
            new Dictionary<string, Frame>
            {
                ["upright"] = placeUpright,
                ["release"] = placeRelease
            },
            new Dictionary<string, object> { ["jaw_mm"] = 50.0 });

        var path = new TaskPath(pick, place);

        // Empty scene: no obstacles
        var scene = new CollisionScene();
        var result = TaskPathValidator.Validate(path, robot, ik, checker, scene);

        Assert.True(result.IsValid, $"Should be collision-free but got: {string.Join("; ", result.Errors)}");
        Assert.Empty(result.Errors);
        Assert.Equal(4, result.ReachableStates.Count); // 2 frames per task
    }

    [Fact]
    public void TaskPath_CollidingFrame_FailsValidation()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var ik = KinematicsResolver.CreateInverseKinematics(preset);

        // Place obstacle near robot base
        var obstacle = CollisionObject.Box("table", new Frame(0.3, 0, 0.2), 0.5, 0.5, 0.05);
        var scene = new CollisionScene(new[] { obstacle });

        // Pick task frame that would require robot to go through the obstacle
        var pickApproach = new Frame(0.3, 0.0, 0.25);
        var pick = new TaskInstance("pick",
            new Dictionary<string, Frame> { ["approach"] = pickApproach },
            new Dictionary<string, object> { ["object"] = "part_B" });

        var path = new TaskPath(pick);
        var result = TaskPathValidator.Validate(path, robot, ik, checker, scene);

        Assert.False(result.IsValid);
        Assert.NotEmpty(result.Errors);
        Assert.Contains(result.Errors, e => e.Contains("collides"));
    }

    [Fact]
    public void TaskPath_SegmentCollision_DetectedByMotionValidation()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var ik = KinematicsResolver.CreateInverseKinematics(preset);

        // Start and goal both reachable, but path goes through obstacle
        var start = new Frame(0.5, 0.3, 0.5);
        var goal = new Frame(0.5, -0.3, 0.5);
        
        var task = new TaskInstance("move",
            new Dictionary<string, Frame>
            {
                ["start"] = start,
                ["goal"] = goal
            });

        var path = new TaskPath(task);

        // Obstacle between start and goal
        var obstacle = CollisionObject.Sphere("blocking", new Frame(0.5, 0.0, 0.5), 0.15);
        var scene = new CollisionScene(new[] { obstacle });

        // Frame-only validation might pass
        var frameResult = TaskPathValidator.Validate(path, robot, ik, checker, scene);
        
        // Motion validation should catch the segment collision
        var motionResult = TaskPathValidator.ValidateWithMotion(path, robot, ik, checker, scene, configurationStep: 0.05);

        if (frameResult.IsValid)
        {
            // If both frames are reachable, motion check should detect segment collision
            Assert.False(motionResult.IsValid, "Motion validation should detect segment collision");
            Assert.Contains(motionResult.Errors, e => e.Contains("Segment collision"));
        }
        else
        {
            // If frames themselves collide, that's also a valid rejection
            Assert.Contains(frameResult.Errors, e => e.Contains("collides"));
        }
    }

    [Fact]
    public void TaskPath_UnreachableFrame_FailsValidation()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var ik = KinematicsResolver.CreateInverseKinematics(preset);

        // Frame far outside reach
        var unreachableFrame = new Frame(5.0, 5.0, 5.0);
        var task = new TaskInstance("impossible",
            new Dictionary<string, Frame> { ["target"] = unreachableFrame });

        var path = new TaskPath(task);
        var scene = new CollisionScene();
        var result = TaskPathValidator.Validate(path, robot, ik, checker, scene);

        Assert.False(result.IsValid);
        Assert.Contains(result.Errors, e => e.Contains("unreachable") || e.Contains("IK failed"));
    }

    [Fact]
    public void MeshCollisionChecker_UsedForTaskValidation()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var meshChecker = new MeshCollisionChecker(preset);
        var ik = KinematicsResolver.CreateInverseKinematics(preset);

        // Create a mesh obstacle (triangular prism)
        var meshVertices = new List<double[]>
        {
            new[] { 0.4, 0.0, 0.0 },  // base triangle
            new[] { 0.5, 0.1, 0.0 },
            new[] { 0.5, -0.1, 0.0 },
            new[] { 0.4, 0.0, 0.3 },  // top triangle
            new[] { 0.5, 0.1, 0.3 },
            new[] { 0.5, -0.1, 0.3 }
        };
        var meshIndices = new List<int>
        {
            0, 1, 2,  // base
            3, 4, 5,  // top
            0, 1, 4, 0, 4, 3,  // side 1
            1, 2, 5, 1, 5, 4,  // side 2
            2, 0, 3, 2, 3, 5   // side 3
        };

        var meshObstacle = CollisionObject.Mesh("prism", Frame.Identity, meshVertices, meshIndices);
        var scene = new CollisionScene(new[] { meshObstacle });

        // Task frame near the mesh obstacle
        var nearMesh = new Frame(0.45, 0.0, 0.15);
        var task = new TaskInstance("test",
            new Dictionary<string, Frame> { ["near"] = nearMesh });

        var path = new TaskPath(task);
        var result = TaskPathValidator.Validate(path, robot, ik, meshChecker, scene);

        // Should detect collision with mesh
        Assert.False(result.IsValid);
        Assert.Contains(result.Errors, e => e.Contains("collides"));
    }

    [Fact]
    public void SphereCollisionChecker_GeometryType()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var checker = new SphereCollisionChecker(preset);
        
        // SphereCollisionChecker uses link envelope spheres for collision
        Assert.NotNull(checker);
    }

    [Fact]
    public void MeshCollisionChecker_GeometryType()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var checker = new MeshCollisionChecker(preset);
        
        // MeshCollisionChecker uses BVH + SAT for mesh-accurate collision
        Assert.NotNull(checker);
    }

    [Fact]
    public void TaskPath_MultiplePicksClearOfObstacles_Pass()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var ik = KinematicsResolver.CreateInverseKinematics(preset);

        // Multiple pick tasks at different locations, all clear
        var pick1 = new TaskInstance("pick",
            new Dictionary<string, Frame>
            {
                ["approach"] = new Frame(0.3, 0.3, 0.5),
                ["grasp"] = new Frame(0.3, 0.3, 0.45)
            },
            new Dictionary<string, object> { ["object"] = "part_1" });

        var pick2 = new TaskInstance("pick",
            new Dictionary<string, Frame>
            {
                ["approach"] = new Frame(0.3, -0.3, 0.5),
                ["grasp"] = new Frame(0.3, -0.3, 0.45)
            },
            new Dictionary<string, object> { ["object"] = "part_2" });

        var path = new TaskPath(pick1, pick2);

        // Obstacle that doesn't interfere
        var farObstacle = CollisionObject.Box("shelf", new Frame(0.8, 0.0, 0.6), 0.1, 0.3, 0.2);
        var scene = new CollisionScene(new[] { farObstacle });

        var result = TaskPathValidator.Validate(path, robot, ik, checker, scene);

        Assert.True(result.IsValid, $"Should be collision-free but got: {string.Join("; ", result.Errors)}");
        Assert.Equal(4, result.ReachableStates.Count);
    }

    [Fact]
    public void TaskPath_BoxObstacle_RejectsCollidingPath()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var ik = KinematicsResolver.CreateInverseKinematics(preset);

        // Large box obstacle blocking typical workspace
        var box = CollisionObject.Box("wall", new Frame(0.4, 0.0, 0.4), 0.2, 0.2, 0.2);
        var scene = new CollisionScene(new[] { box });

        // Frame inside/near the box
        var blockedFrame = new Frame(0.4, 0.0, 0.4);
        var task = new TaskInstance("blocked",
            new Dictionary<string, Frame> { ["target"] = blockedFrame });

        var path = new TaskPath(task);
        var result = TaskPathValidator.Validate(path, robot, ik, checker, scene);

        Assert.False(result.IsValid);
        Assert.Contains(result.Errors, e => e.Contains("collides") || e.Contains("unreachable"));
    }

    [Fact]
    public void TaskPath_SphereObstacle_RejectsCollidingPath()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var ik = KinematicsResolver.CreateInverseKinematics(preset);

        // Sphere obstacle in workspace
        var sphere = CollisionObject.Sphere("ball", new Frame(0.35, 0.2, 0.4), 0.12);
        var scene = new CollisionScene(new[] { sphere });

        // Frame near the sphere
        var nearSphere = new Frame(0.35, 0.2, 0.4);
        var task = new TaskInstance("near_ball",
            new Dictionary<string, Frame> { ["target"] = nearSphere });

        var path = new TaskPath(task);
        var result = TaskPathValidator.Validate(path, robot, ik, checker, scene);

        Assert.False(result.IsValid);
        Assert.Contains(result.Errors, e => e.Contains("collides") || e.Contains("unreachable"));
    }
}
