using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;

namespace Motus.Core.Tests;

public class GroupPlanningTests
{
    [Fact]
    public void GroupMap_LocksUnmappedJointsDuringRrt()
    {
        var robot = PresetLoader.LoadRobotModelByName("UR5e");
        var start = new JointState(new double[] { 0, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });
        var goal = new JointState(new double[] { 0.35, -1.1, 1.4, 0.15, 1.0, 0.9 });

        var group = new PlanningGroup("arm", "base_link", "tool0",
            ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2"]);
        var map = JointIndexMap.Resolve(robot, group);

        var opts = new PlanningOptions
        {
            CollisionChecker = CollisionCheckerFactory.Create(robot),
            GroupMap = map,
            MaxJointStepRadians = 0.08
        };

        var planner = new RrtConnectPlanner(opts.CollisionChecker, new RrtConnectOptions { MaxIterations = 6000, RandomSeed = 3 });
        var result = planner.Plan(new PlanningRequest(robot, start, goal, opts));
        Assert.True(result.Success, string.Join("; ", result.Errors));

        foreach (var pt in result.Trajectory!.Points)
            Assert.Equal(start.Positions[5], pt.JointState.Positions[5], 1e-9);
    }

    [Fact]
    public void SrdfGroup_ForGroup_PlanFullManipulator()
    {
        var fixtures = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures"));
        var urdfPath = Path.Combine(fixtures, "ur10e", "ur10e.urdf");
        var srdfPath = Path.Combine(fixtures, "ur10e", "ur10e.srdf");

        var loaded = UrdfRobotLoader.Load(urdfPath, new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tool0",
            ModelName = "UR10e"
        });
        var robot = loaded.ToModel();
        var group = Assert.Single(SrdfLoader.LoadGroups(srdfPath), g => g.Name == "ur_manipulator");

        var start = new JointState(new double[] { 0, -Math.PI / 2, Math.PI / 2, 0, 0, 0 });
        var goal = new JointState(new double[] { 0.3, -1.0, 1.2, 0.1, 0.5, 0.2 });
        var checker = CollisionCheckerFactory.Create(robot);
        var ctx = PlanningContext.Create(robot).ForGroup(group);
        var opts = ctx.ToPlanningOptions(new PlanningOptions
        {
            CollisionChecker = checker,
            MaxJointStepRadians = 0.08
        });

        var planner = new RrtConnectPlanner(checker, new RrtConnectOptions { MaxIterations = 6000, RandomSeed = 5 });
        var result = planner.Plan(new PlanningRequest(robot, start, goal, opts));
        Assert.True(result.Success, string.Join("; ", result.Errors));
    }
}

public class ToolCollisionPresetTests
{
    [Fact]
    public void ToolCollision_LoadsFromJson()
    {
        var json = """
        {
          "manufacturer": "UniversalRobots",
          "modelName": "TestTool",
          "family": "test",
          "axisCount": 6,
          "jointLimits": [
            { "minRadians": -3.14, "maxRadians": 3.14, "maxVelocityRadiansPerSecond": 1, "maxAccelerationRadiansPerSecondSquared": 1 },
            { "minRadians": -3.14, "maxRadians": 3.14, "maxVelocityRadiansPerSecond": 1, "maxAccelerationRadiansPerSecondSquared": 1 },
            { "minRadians": -3.14, "maxRadians": 3.14, "maxVelocityRadiansPerSecond": 1, "maxAccelerationRadiansPerSecondSquared": 1 },
            { "minRadians": -3.14, "maxRadians": 3.14, "maxVelocityRadiansPerSecond": 1, "maxAccelerationRadiansPerSecondSquared": 1 },
            { "minRadians": -3.14, "maxRadians": 3.14, "maxVelocityRadiansPerSecond": 1, "maxAccelerationRadiansPerSecondSquared": 1 },
            { "minRadians": -3.14, "maxRadians": 3.14, "maxVelocityRadiansPerSecond": 1, "maxAccelerationRadiansPerSecondSquared": 1 }
          ],
          "collisionLinks": [
            { "link": 0, "shape": "sphere", "radius": 0.08 }
          ],
          "toolCollision": { "shape": "box", "halfX": 0.02, "halfY": 0.03, "halfZ": 0.04 }
        }
        """;

        var model = PresetLoader.LoadRobotModelFromJson(json, "preset.json");
        Assert.NotNull(model.CollisionModel?.ToolGeometry);
        Assert.Equal(CollisionShape.Box, model.CollisionModel.ToolGeometry.Shape);
        Assert.Equal(0.02, model.CollisionModel.ToolGeometry.ExtentX);
    }

    [Fact]
    public void BundledUr5e_HasToolCollision()
    {
        var model = PresetLoader.LoadRobotModelByName("UR5e");
        Assert.NotNull(model.CollisionModel?.ToolGeometry);
        Assert.Equal(CollisionShape.Box, model.CollisionModel.ToolGeometry.Shape);
    }
}
