using Motus.Core;
using Motus.Geometry;
using Motus.Native;
using Motus.OMPL.NET;
using Motus.OMPL.Native;
using Motus.Presets;

namespace Motus.Core.Tests;

public class NativeOmplIntegrationTests
{
    private static bool ExpectFullNative =>
        string.Equals(Environment.GetEnvironmentVariable("MOTUS_NATIVE_FULL"), "1", StringComparison.Ordinal);

    [Fact]
    public void NativeOmpl_WhenBuilt_PlanSmoke()
    {
        if (!NativeOmpl.IsAvailable)
        {
            if (ExpectFullNative)
                Assert.Fail($"Native OMPL expected: {NativeBindings.LastError()}");
            return;
        }

        var robot = PresetLoader.LoadRobotModelByName("UR5e");
        var start = new JointState(new double[] { 0, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });
        var goal = new JointState(new double[] { 0.25, -1.1, 1.3, 0.1, 1.0, 0.2 });
        var checker = CollisionCheckerFactory.Create(robot);
        var planner = new RrtConnectPlanner(checker, new RrtConnectOptions
        {
            MaxIterations = 4000,
            MaxPlanTimeSeconds = 2.0,
            RandomSeed = 7
        });

        var result = planner.Plan(new PlanningRequest(robot, start, goal, new PlanningOptions
        {
            CollisionChecker = checker,
            MaxJointStepRadians = 0.08
        }));

        Assert.True(result.Success, string.Join("; ", result.Errors));
        Assert.Contains("native", string.Join(" ", result.Warnings), StringComparison.OrdinalIgnoreCase);
    }
}
