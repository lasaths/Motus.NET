using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>
/// Contract for Motus.Grasshopper PlanExecutor plane-goal path:
/// TCP-LIN dies on collision → IK goal → SamplingPlanner RRT.
/// </summary>
public class LinCollisionRrtFallbackTests
{
    [Fact]
    public void PlaneGoal_LinBlockedBySphere_ThenRrtSucceeds()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", FindResources());
        var robot = new RobotModel(preset);
        var checker = new SphereCollisionChecker(preset);
        var fk = KinematicsResolver.CreateFkSolver(preset);
        var home = new JointState(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });
        var goalJ = new JointState(new[] { 1.2, -1.0, 1.2, -1.6, -1.5708, 0.0 });
        var goalTcp = fk.ComputeTcp(goalJ, preset.BaseFrame, preset.ToolFrame);

        var freeLin = new CartesianLinearPathPlanner(preset).PlanToResult(
            new CartesianPlanningRequest(robot, home, goalTcp, new PlanningOptions { MaxJointStepRadians = 0.05 }),
            new CartesianLinOptions(StepMeters: 0.005, ContinueOnIkFailure: false));
        Assert.True(freeLin.Success, string.Join("; ", freeLin.Errors));

        CollisionScene? blocking = null;
        foreach (var pt in freeLin.Trajectory!.Points)
        {
            var origins = fk.ComputeLinkOrigins(pt.JointState.Positions, preset.BaseFrame.Frame);
            foreach (var origin in origins)
            {
                var trial = new CollisionScene(new[] { CollisionObject.Sphere("block", origin, 0.12) });
                if (checker.IsCollisionFree(home, trial)
                    && checker.IsCollisionFree(goalJ, trial)
                    && !checker.IsCollisionFree(pt.JointState, trial))
                {
                    blocking = trial;
                    break;
                }
            }
            if (blocking is not null) break;
        }
        Assert.NotNull(blocking);

        var opts = new PlanningOptions
        {
            CollisionScene = blocking,
            CollisionChecker = checker,
            MaxJointStepRadians = 0.05
        };
        var blockedLin = new CartesianLinearPathPlanner(preset).PlanToResult(
            new CartesianPlanningRequest(robot, home, goalTcp, opts, blocking),
            new CartesianLinOptions(StepMeters: 0.005, ContinueOnIkFailure: false));
        Assert.False(blockedLin.Success);
        Assert.Contains(blockedLin.Errors, e => e.Contains("Collision", StringComparison.OrdinalIgnoreCase));

        var reach = new CartesianGoalSolver().TryReach(
            robot, goalTcp, CartesianGoalSolver.EnumerateDefaultSeeds(home, robot));
        Assert.True(reach.Success, string.Join("; ", reach.Errors));

        var rrt = SamplingPlanner.Create(checker, new SamplingPlannerOptions
        {
            PlannerId = SamplingPlannerId.RrtConnect,
            MaxIterations = 12000,
            MaxPlanTimeSeconds = 30,
            RandomSeed = 42,
            GoalBias = 0.08,
            StepRadians = 0.12
        }).Plan(new PlanningRequest(robot, home, reach.Solution!, opts));
        Assert.True(rrt.Success, string.Join("; ", rrt.Errors));
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
