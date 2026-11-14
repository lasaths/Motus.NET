using System.Xml.Linq;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>Malformed numbers and nested xacro macros stop the load with a named error (no silent default).</summary>
public class UrdfFailClosedTests
{
    private static XDocument Robot(string limit = "lower=\"-1\" upper=\"1\"", string origin = "0 0 0.1", string collision = "<sphere radius=\"0.03\"/>") =>
        XDocument.Parse($"""
            <robot name="r">
              <link name="base_link"/>
              <link name="tip_link"><collision><geometry>{collision}</geometry></collision></link>
              <joint name="joint1" type="revolute">
                <parent link="base_link"/><child link="tip_link"/>
                <origin xyz="{origin}"/><axis xyz="0 0 1"/>
                <limit {limit}/>
              </joint>
            </robot>
            """);

    private static UrdfRobot Load(XDocument doc) =>
        UrdfRobotLoader.Load(doc, new UrdfLoadOptions { BaseLink = "base_link", TipLink = "tip_link" }, ".");

    [Fact]
    public void WellFormed_Loads_AndAbsentAttributesKeepDefaults()
    {
        var r = Load(Robot(limit: "velocity=\"1\""));
        Assert.Equal(-Math.PI, r.Preset.JointLimits[0].MinRadians);
        Assert.Equal(Math.PI, r.Preset.JointLimits[0].MaxRadians);
        Assert.Equal(0.03, UrdfCollisionLoader.Load(Robot().Root!, new[] { "tip_link" }, ".")!.Links[0].LocalGeometry.ExtentX);
    }

    [Theory]
    [InlineData("lower=\"abc\" upper=\"1\"", "joint 'joint1' limit lower")]
    [InlineData("lower=\"-1\" upper=\"NaN\"", "joint 'joint1' limit upper")]
    [InlineData("lower=\"-1\" upper=\"1e999\"", "joint 'joint1' limit upper")]
    [InlineData("lower=\"-1\" upper=\"1\" velocity=\"fast\"", "joint 'joint1' limit velocity")]
    public void MalformedJointLimit_Throws(string limit, string what)
    {
        var e = Assert.Throws<InvalidDataException>(() => Load(Robot(limit: limit)));
        Assert.Contains(what, e.Message);
    }

    [Theory]
    [InlineData("0 0")]
    [InlineData("0 0 x")]
    public void MalformedJointOrigin_Throws(string origin)
    {
        var e = Assert.Throws<InvalidDataException>(() => Load(Robot(origin: origin)));
        Assert.Contains("joint 'joint1' origin xyz", e.Message);
    }

    [Theory]
    [InlineData("<sphere radius=\"NaN\"/>", "sphere radius")]
    [InlineData("<cylinder radius=\"0.1\" length=\"long\"/>", "cylinder length")]
    [InlineData("<box size=\"0.1 0.1\"/>", "box size")]
    public void MalformedCollisionGeometry_Throws(string collision, string what)
    {
        var e = Assert.Throws<InvalidDataException>(() =>
            UrdfCollisionLoader.Load(Robot(collision: collision).Root!, new[] { "tip_link" }, "."));
        Assert.Contains("tip_link_col0", e.Message);
        Assert.Contains(what, e.Message);
    }

    [Theory]
    [InlineData("""<xacro:macro name="loop"><link name="a"><xacro:loop/></link></xacro:macro><xacro:loop/>""", "loop")]
    [InlineData("""<xacro:macro name="ping"><xacro:pong/></xacro:macro><xacro:macro name="pong"><xacro:ping/></xacro:macro><xacro:pong/>""", "ping")]
    public void SelfOrNestedMacroCall_ThrowsClearError(string body, string inner)
    {
        var path = Path.Combine(Path.GetTempPath(), $"motus_xacro_{Guid.NewGuid():N}.xacro");
        File.WriteAllText(path, $"""<robot name="r" xmlns:xacro="http://www.ros.org/wiki/xacro">{body}</robot>""");
        try
        {
            var e = Assert.Throws<NotSupportedException>(() => XacroPreprocessor.Expand(path));
            Assert.Contains($"calls macro '{inner}'", e.Message);
        }
        finally { File.Delete(path); }
    }
}
