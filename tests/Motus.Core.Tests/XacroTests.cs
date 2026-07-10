using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class XacroTests
{
    private static string FixturePath(string name) =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", name));

    [Fact]
    public void XacroLoad_TwoLink_MatchesUrdfImport()
    {
        var fromUrdf = UrdfRobotLoader.Load(FixturePath("two_link.urdf"), new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tip_link",
            ModelName = "two_link"
        });
        var fromXacro = UrdfRobotLoader.LoadXacro(FixturePath("two_link.xacro"), new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tip_link",
            ModelName = "two_link"
        });

        Assert.Equal(fromUrdf.Preset.AxisCount, fromXacro.Preset.AxisCount);
        Assert.Equal(fromUrdf.JointNames, fromXacro.JointNames);
    }

    [Fact]
    public void XacroExpand_SubstitutesProperty()
    {
        var xml = XacroPreprocessor.Expand(FixturePath("two_link.xacro"));
        Assert.Contains("xyz=\"0.2 0 0\"", xml);
        Assert.DoesNotContain("${link_len}", xml);
    }
}
