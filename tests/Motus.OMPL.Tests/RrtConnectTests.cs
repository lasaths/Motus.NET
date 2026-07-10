using System.Diagnostics;
using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;
using Xunit.Abstractions;

namespace Motus.OMPL.Tests;

public class RrtConnectTests
{
  private readonly ITestOutputHelper? _output;

  public RrtConnectTests(ITestOutputHelper? output = null) => _output = output;  private static string ResourcesRoot =>
    Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

  [Fact]
  public void PlansFreeSpace()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var robot = new RobotModel(preset);
    var start = new JointState(new double[6]);
    var goal = new JointState(new[] { 0.5, -0.5, 0.5, -0.5, -0.5, 0.2 });
    var planner = new RrtConnectPlanner(preset, new RrtConnectOptions { MaxIterations = 3000, RandomSeed = 3 });
    var result = planner.Plan(new PlanningRequest(robot, start, goal));
    Assert.True(result.Success, string.Join("; ", result.Errors));
  }

  [Fact]
  public void PlansAroundObstacle()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var robot = new RobotModel(preset);
    var start = new JointState(new double[6]);
    var goal = new JointState(new[] { 0.6, -0.6, 0.6, -0.6, -0.6, 0.3 });
    var checker = new SphereCollisionChecker(preset);
    var scene = FindBlockingScene(checker, preset, start, goal, 0.08)
      ?? throw new InvalidOperationException("Could not place a blocking obstacle for RRT test.");

    var opts = new PlanningOptions { CollisionScene = scene, MaxJointStepRadians = 0.08, CollisionChecker = checker };
    var planner = new RrtConnectPlanner(checker, new RrtConnectOptions { MaxIterations = 10000, RandomSeed = 11 });
    var result = planner.Plan(new PlanningRequest(robot, start, goal, opts));
    Assert.True(result.Success, string.Join("; ", result.Errors));
    var val = new TrajectoryValidator().Validate(result.Trajectory!, new TrajectoryValidationOptions
    {
      CollisionChecker = checker,
      CollisionScene = scene,
      CheckAcceleration = false
    });
    Assert.True(val.IsValid, string.Join("; ", val.Errors));
  }

  [Fact]
  public void PlansAroundObstacle_WithRobotMeshCheckerViaOptions()
  {
    var model = PresetLoader.LoadRobotModelByName("UR5e", ResourcesRoot);
    var robot = model;
    var start = new JointState(new double[6]);
    var goal = new JointState(new[] { 0.6, -0.6, 0.6, -0.6, -0.6, 0.3 });
    var checker = new RobotMeshCollisionChecker(model);
    var scene = FindBlockingScene(checker, model.Preset, start, goal, 0.08)
      ?? throw new InvalidOperationException("Could not place a blocking obstacle for mesh RRT test.");

    var opts = new PlanningOptions { CollisionScene = scene, MaxJointStepRadians = 0.08, CollisionChecker = checker };
    var planner = new RrtConnectPlanner(model.Preset, new RrtConnectOptions { MaxIterations = 10000, RandomSeed = 11 });
    var result = planner.Plan(new PlanningRequest(robot, start, goal, opts));
    Assert.True(result.Success, string.Join("; ", result.Errors));
  }

  [Fact]
  public void PlansMultiGoalSequenceAroundObstacle()
  {
    var total = Stopwatch.StartNew();
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var robot = new RobotModel(preset);
    var checker = new SphereCollisionChecker(preset);
    var planner = new RrtConnectPlanner(checker, new RrtConnectOptions { MaxIterations = 6000, RandomSeed = 17 });

    var start = new JointState(new double[6]);
    var goals = new[]
    {
      new JointState(new[] { 0.35, -0.45, 0.55, -0.45, -0.45, 0.20 }),
      new JointState(new[] { 0.55, -0.70, 0.80, -0.60, -0.55, 0.25 }),
      new JointState(new[] { 0.70, -0.80, 0.90, -0.70, -0.65, 0.30 }),
    };

    var sceneSw = Stopwatch.StartNew();
    var scene = FindBlockingScene(checker, preset, start, goals[0], 0.08)
      ?? throw new InvalidOperationException("Could not place a blocking obstacle for multi-goal RRT test.");
    sceneSw.Stop();
    Assert.False(checker.SegmentCollisionFree(start, goals[0], scene, 0.08));

    var validator = new TrajectoryValidator();
    var from = start;
    var planMs = 0L;
    var validateMs = 0L;
    for (var i = 0; i < goals.Length; i++)
    {
      var goal = goals[i];
      var opts = new PlanningOptions
      {
        CollisionScene = scene,
        CollisionChecker = checker,
        MaxJointStepRadians = 0.08
      };

      var planSw = Stopwatch.StartNew();
      var result = planner.Plan(new PlanningRequest(robot, from, goal, opts));
      planSw.Stop();
      planMs += planSw.ElapsedMilliseconds;
      Assert.True(result.Success, $"Segment {i} failed: {string.Join("; ", result.Errors)}");

      var valSw = Stopwatch.StartNew();
      var val = validator.Validate(result.Trajectory!, new TrajectoryValidationOptions
      {
        CollisionChecker = checker,
        CollisionScene = scene,
        CheckAcceleration = false
      });
      valSw.Stop();
      validateMs += valSw.ElapsedMilliseconds;
      Assert.True(val.IsValid, $"Segment {i} invalid: {string.Join("; ", val.Errors)}");
      from = goal;
    }

    total.Stop();
    _output?.WriteLine($"scene={sceneSw.ElapsedMilliseconds}ms plan={planMs}ms validate={validateMs}ms total={total.ElapsedMilliseconds}ms");
    Assert.True(total.ElapsedMilliseconds < 250, $"Multi-goal obstacle planning too slow: {total.ElapsedMilliseconds}ms");
  }

  private static CollisionScene? FindBlockingScene(
    ICollisionChecker checker,
    RobotPreset preset,
    JointState start,
    JointState goal,
    double stepRadians)
  {
    var fk = new DhForwardKinematics(preset);
    for (var s = 1; s <= 7; s++)
    {
      var alpha = s / 8.0;
      var q = new double[start.AxisCount];
      for (var i = 0; i < q.Length; i++)
        q[i] = start.Positions[i] + alpha * (goal.Positions[i] - start.Positions[i]);
      var mid = fk.ComputeTcp(new JointState(q), preset.BaseFrame, preset.ToolFrame);
      var trial = new CollisionScene(new[] { CollisionObject.Sphere("block", mid.Tcp, 0.05) });
      if (checker.IsCollisionFree(start, trial) && checker.IsCollisionFree(goal, trial)
          && !checker.SegmentCollisionFree(start, goal, trial, stepRadians))
        return trial;
    }
    return null;
  }

  [Fact]
  public void PathSimplifier_ReducesWaypoints()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var robot = new RobotModel(preset);
    var path = new[]
    {
      new JointState(new double[6]),
      new JointState(new[] { 0.1, 0, 0, 0, 0, 0 }),
      new JointState(new[] { 0.2, 0, 0, 0, 0, 0 }),
      new JointState(new[] { 0.3, 0, 0, 0, 0, 0 }),
    };
    var simplified = PathSimplifier.Simplify(path, robot, null, new CollisionScene(), 0.05);
    Assert.True(simplified.Count <= path.Length);
    Assert.Equal(0, simplified[0].Positions[0], 6);
    Assert.Equal(0.3, simplified[^1].Positions[0], 6);
  }

  [Fact]
  public void NativeOmpl_IsUnavailableWithoutNativeBuild()
  {
    Assert.False(Motus.OMPL.Native.NativeOmpl.IsAvailable);
  }

  [Fact]
  public void RejectsNonPositiveStepRadians()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var robot = new RobotModel(preset);
    var planner = new RrtConnectPlanner(preset, new RrtConnectOptions { StepRadians = 0 });
    var result = planner.Plan(new PlanningRequest(
      robot,
      new JointState(new double[6]),
      new JointState(new[] { 0.3, -0.2, 0.2, -0.1, 0.1, 0.1 })));
    Assert.False(result.Success);
    Assert.Contains(result.Errors, e => e.Contains("StepRadians", StringComparison.OrdinalIgnoreCase));
  }
}
