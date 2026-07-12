using Motus.Core;
using Motus.OMPL.NET;
using Motus.Presets;

namespace Motus.OMPL.Tests;

public class SamplingPlannerRegistryTests
{
    [Fact]
    public void ListAvailable_IncludesManagedRrtConnectOnStubBuild()
    {
        var available = SamplingPlannerRegistry.ListAvailable();
        Assert.Contains(available, d => d.Id == SamplingPlannerId.RrtConnect && d.ManagedSupported);
    }

    [Fact]
    public void TryParse_KnownPlannerNames()
    {
        Assert.True(SamplingPlannerRegistry.TryParse("RrtConnect", out var id));
        Assert.Equal(SamplingPlannerId.RrtConnect, id);

        Assert.True(SamplingPlannerRegistry.TryParse("RRT-Connect", out id));
        Assert.Equal(SamplingPlannerId.RrtConnect, id);
    }

    [Fact]
    public void Resolve_ReturnsLabel()
    {
        var desc = SamplingPlannerRegistry.Resolve(SamplingPlannerId.RrtConnect);
        Assert.NotNull(desc);
        Assert.Equal("RRT-Connect", desc!.Label);
        Assert.Equal("RrtConnect", desc.ShortName);
    }

    [Fact]
    public void SamplingPlanner_CreatePlansFreeSpace()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var start = new JointState(new double[6]);
        var goal = new JointState(new[] { 0.5, -0.5, 0.5, -0.5, -0.5, 0.2 });
        var planner = SamplingPlanner.Create(preset, new SamplingPlannerOptions
        {
            PlannerId = SamplingPlannerId.RrtConnect,
            MaxIterations = 3000,
            RandomSeed = 3,
            PreferManaged = true
        });
        var result = planner.Plan(new PlanningRequest(robot, start, goal));
        Assert.True(result.Success, string.Join("; ", result.Errors));
    }

    private static string ResourcesRoot =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));
}
