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
}
