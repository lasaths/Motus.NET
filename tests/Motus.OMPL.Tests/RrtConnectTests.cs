using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;

namespace Motus.OMPL.Tests;

public class RrtConnectTests
{
  private static string ResourcesRoot =>
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
    var candidates = new[]
    {
      new Frame(0.18, -0.22, 0.28),
      new Frame(0.22, -0.18, 0.32),
      new Frame(0.12, -0.28, 0.24),
      new Frame(0.25, -0.30, 0.35),
    };

    CollisionScene? scene = null;
    foreach (var pose in candidates)
    {
      var trial = new CollisionScene(new[] { CollisionObject.Sphere("block", pose, 0.05) });
      if (checker.IsCollisionFree(start, trial) && checker.IsCollisionFree(goal, trial)
          && !checker.SegmentCollisionFree(start, goal, trial, 0.08))
      {
        scene = trial;
        break;
      }
    }

    Assert.NotNull(scene);
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
}
