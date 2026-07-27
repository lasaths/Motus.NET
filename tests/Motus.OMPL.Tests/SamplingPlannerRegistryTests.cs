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

        Assert.True(SamplingPlannerRegistry.TryParse("PRM*", out id));
        Assert.Equal(SamplingPlannerId.PrmStar, id);
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

    [Fact]
    public void SamplingPlanner_PrmStarPlansFreeSpace()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var start = new JointState(new double[6]);
        var goal = new JointState(new[] { 0.25, -0.2, 0.2, -0.15, -0.1, 0.1 });
        var planner = SamplingPlanner.Create(preset, new SamplingPlannerOptions
        {
            PlannerId = SamplingPlannerId.PrmStar,
            MaxIterations = 200,
            MaxPathStates = 80,
            RandomSeed = 7,
            PreferManaged = true
        });

        var result = planner.Plan(new PlanningRequest(robot, start, goal));
        Assert.True(result.Success, string.Join("; ", result.Errors));
        Assert.True(result.Trajectory!.Points.Count >= 2);
    }

    [Fact]
    public void SamplingPlanner_RejectsConstraintViolationWithReasonCode()
    {
        var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
        var robot = new RobotModel(preset);
        var planner = SamplingPlanner.Create(preset, new SamplingPlannerOptions
        {
            PlannerId = SamplingPlannerId.RrtConnect,
            MaxIterations = 200,
            PreferManaged = true
        });
        var result = planner.Plan(new PlanningRequest(
            robot,
            new JointState(new double[6]),
            new JointState(new[] { 0.2, -0.1, 0.1, -0.1, 0.0, 0.0 }),
            new PlanningOptions { ConstraintChecker = new AlwaysFailConstraint() }));

        Assert.False(result.Success);
        Assert.Contains(result.Messages, m => m.Code == PlanningMessageCodes.ConstraintViolation);
        Assert.Contains("ConstraintViolation", string.Join("; ", result.Errors));
    }

    private static string ResourcesRoot =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

    private sealed class AlwaysFailConstraint : IConstraintChecker
    {
        public bool TryValidate(Frame tcp, out string reason)
        {
            reason = "ConstraintViolation: test constraint rejects all TCP frames.";
            return false;
        }
    }
}
