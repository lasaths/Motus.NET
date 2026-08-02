using Motus.Core;
using Motus.Geometry;

namespace Motus.Core.Tests;

public class ToolParameterBindingTests
{
    [Fact]
    public void Robotiq_OpenClosed_DriverAngles()
    {
        Assert.Equal(0, ToolParameterBinding.Robotiq2F85DriverAngleRadians(0.085), 9);
        Assert.Equal(0.8, ToolParameterBinding.Robotiq2F85DriverAngleRadians(0), 9);
    }

    [Fact]
    public void ApplyInto_WritesPrimaryKnuckle()
    {
        var names = new[] { "shoulder_pan_joint", "robotiq_85_left_knuckle_joint", "other" };
        var q = new double[3];
        var n = ToolParameterBinding.ApplyInto(
            ToolCapabilities.Robotiq2F85,
            new EndEffectorState(new Dictionary<string, double> { ["width"] = 0 }),
            names,
            q);
        Assert.Equal(1, n);
        Assert.Equal(0.8, q[1], 9);
        Assert.Equal(0, q[0]);
    }

    [Fact]
    public void WidthBinding_CustomCap_ApplyIntoExactJoint()
    {
        var caps = ToolCapabilities.WidthSchema(0, 0.12, 0.12);
        var binding = ToolParameterBinding.WidthBinding("j_jaw", openWidthMeters: 0.12, closedDriverValue: 1.0);
        var names = new[] { "j_other", "j_jaw" };
        var q = new double[2];
        var n = ToolParameterBinding.ApplyInto(
            caps,
            new EndEffectorState(new Dictionary<string, double> { ["width"] = 0 }),
            names,
            q,
            [binding]);
        Assert.Equal(1, n);
        Assert.Equal(1.0, q[1], 9);
        Assert.Equal(0, q[0]);
    }
}

public class MobilityModelTests
{
    [Fact]
    public void HolonomicSE2_YawQuarterTurn()
    {
        var m = new MobilityModel.HolonomicSE2(1, 2, Math.PI / 2);
        var f = m.BaseFrame;
        Assert.Equal(1, f.X, 9);
        Assert.Equal(2, f.Y, 9);
        Assert.Equal(0, f.Z, 9);
        Assert.True(Math.Abs(f.Qw - Math.Sqrt(0.5)) < 1e-9);
        Assert.True(Math.Abs(f.Qz - Math.Sqrt(0.5)) < 1e-9);
    }
}

public class JointTableTreesTests
{
    [Fact]
    public void FromRows_SerialTwoRevolute()
    {
        var tree = JointTableTrees.FromRows(new[]
        {
            new JointTableRow("j0", "base_link", "link1", "R", 0, 0, 0.2, 0, 0, 1, -Math.PI, Math.PI),
            new JointTableRow("j1", "link1", "tool0", "R", 0.3, 0, 0, 0, 0, 1, -Math.PI, Math.PI),
        });
        Assert.Equal(2, tree.DriverCount);
        var tip = tree.ExtractSerialTip("base_link", "tool0");
        Assert.Equal(2, tip.Chain.Joints.Length);
        Assert.Contains(tree.Links, l => l.Name == "tool0");
    }

    [Fact]
    public void FromRows_Branch_TwoChildren()
    {
        var tree = JointTableTrees.FromRows(new[]
        {
            new JointTableRow("j0", "base_link", "link1", "R", 0, 0, 0.1, 0, 0, 1, -1, 1),
            new JointTableRow("j1", "link1", "left", "R", 0.1, 0.05, 0, 0, 0, 1, -1, 1),
            new JointTableRow("j2", "link1", "right", "R", 0.1, -0.05, 0, 0, 0, 1, -1, 1),
        });
        Assert.Equal(3, tree.DriverCount);
        Assert.Equal(4, tree.Links.Count); // base, link1, left, right
    }
}

public class KinematicsResolverNDofTests
{
    [Fact]
    public void NonUrSerial_UsesNumericalIk()
    {
        var tree = SerialKinematicTrees.FromLengths(new[] { 0.2, 0.3, 0.2, 0.15, 0.1, 0.08, 0.05 }, rail: true);
        var tip = tree.ExtractSerialTip("base_link", "tool0");
        var limits = new List<JointLimit>(tree.DriverCount);
        for (var i = 0; i < tree.DriverCount; i++)
        {
            var j = tree.Joints[tree.DriverJointIndices[i]];
            limits.Add(new JointLimit(j.Lower, j.Upper, Math.PI, Math.PI * 2));
        }
        var preset = new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = "rail_arm",
            Family = "serial",
            AxisCount = tip.Chain.Joints.Length,
            JointLimits = limits,
            BaseFrame = BaseFrame.Identity,
            ToolFrame = ToolFrame.Identity,
        };
        var ik = KinematicsResolver.CreateInverseKinematics(preset, tip.Chain);
        Assert.IsType<NumericalInverseKinematics>(ik);
    }
}
