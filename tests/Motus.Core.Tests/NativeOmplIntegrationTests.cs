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

        var preset = PresetLoader.LoadByModelName("UR5e");
        var robot = new RobotModel(preset);
        var start = new JointState(new double[6]);
        var goal = new JointState(new[] { 0.5, -0.5, 0.5, -0.5, -0.5, 0.2 });
        var planner = new RrtConnectPlanner(preset, new RrtConnectOptions
        {
            MaxIterations = 3000,
            RandomSeed = 7
        });

        var result = planner.Plan(new PlanningRequest(robot, start, goal));

        Assert.True(result.Success, string.Join("; ", result.Errors));
        Assert.Contains("native", string.Join(" ", result.Warnings), StringComparison.OrdinalIgnoreCase);
    }
}
