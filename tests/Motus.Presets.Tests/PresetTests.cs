using Motus.Core;
using Motus.Presets;

namespace Motus.Presets.Tests;

public class PresetLoaderTests
{
  private static string ResourcesRoot =>
    Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

  [Theory]
  [InlineData("UR5e")]
  [InlineData("KR 6 R900")]
  public void LoadsKnownPreset(string model)
  {
    var preset = PresetLoader.LoadByModelName(model, ResourcesRoot);
    Assert.Equal(model, preset.ModelName);
    Assert.Equal(preset.AxisCount, preset.JointLimits.Count);
  }

  [Fact]
  public void InvalidJsonThrows()
  {
    Assert.Throws<InvalidOperationException>(() => PresetLoader.LoadFromJson("{}"));
  }

  [Fact]
  public void MissingModelThrows()
  {
    Assert.Throws<FileNotFoundException>(() => PresetLoader.LoadByModelName("NoSuchRobot", ResourcesRoot));
  }

  [Fact]
  public void ListAvailableModels()
  {
    var models = PresetLoader.ListAvailableModels(ResourcesRoot);
    Assert.Contains("UR5e", models);
    Assert.Contains("LBR iiwa 7 R800", models);
  }

  [Fact]
  public void AllPresetsPlanAndValidate()
  {
    var models = PresetLoader.ListAvailableModels(ResourcesRoot);
    Assert.Equal(14, models.Count);
    var planner = new JointLinearPlanner();
    var validator = new TrajectoryValidator();
    foreach (var model in models)
    {
      var preset = PresetLoader.LoadByModelName(model, ResourcesRoot);
      var robot = new RobotModel(preset);
      var start = new JointState(Enumerable.Repeat(0.0, preset.AxisCount).ToArray());
      var goalVal = 0.1 * preset.AxisCount;
      var goal = new JointState(Enumerable.Repeat(goalVal, preset.AxisCount).ToArray());
      var result = planner.Plan(new PlanningRequest(robot, start, goal));
      Assert.True(result.Success, $"Plan failed for {model}: {string.Join("; ", result.Errors)}");
      Assert.True(validator.Validate(result.Trajectory!).IsValid, $"Validation failed for {model}");
    }
  }
}
