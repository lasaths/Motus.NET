using Motus.Core;
using Motus.Geometry;

namespace Motus.Core.Tests;

public class RobotDescriptionTests
{
    [Fact]
    public void Assemble_SerialTwoLinkRevolute_DriverCountAndTipExtract()
    {
        var desc = RobotDescription.Assemble(
            "two_link_arm",
            new[] { new UrdfLink("base_link"), new UrdfLink("link1") },
            new[]
            {
                new UrdfJoint("j0", "revolute", "base_link", "link1",
                    0, 0, 0.2, 0, 0, 1, -Math.PI, Math.PI),
            },
            tipLink: "link1");

        var tree = desc.ToKinematicTree();

        Assert.Equal(1, tree.DriverCount);
        Assert.Equal(2, tree.Links.Count);

        var tip = tree.ExtractSerialTip("base_link", "link1");
        Assert.Single(tip.Chain.Joints);
    }

    [Fact]
    public void Assemble_DuplicateLinkNames_Throws()
    {
        var ex = Assert.Throws<InvalidOperationException>(() => RobotDescription.Assemble(
            "dup_links",
            new[] { new UrdfLink("base_link"), new UrdfLink("base_link") },
            Array.Empty<UrdfJoint>()));

        Assert.Contains("Duplicate link name", ex.Message);
    }

    [Fact]
    public void Assemble_MissingParentLink_Throws()
    {
        var ex = Assert.Throws<InvalidOperationException>(() => RobotDescription.Assemble(
            "missing_parent",
            new[] { new UrdfLink("base_link"), new UrdfLink("link1") },
            new[] { new UrdfJoint("j0", "revolute", "does_not_exist", "link1", 0, 0, 0, 0, 0, 1, -1, 1) }));

        Assert.Contains("unknown parent link", ex.Message);
    }

    [Fact]
    public void Assemble_Cycle_Throws()
    {
        var ex = Assert.Throws<InvalidOperationException>(() => RobotDescription.Assemble(
            "cycle",
            new[] { new UrdfLink("a"), new UrdfLink("b"), new UrdfLink("c") },
            new[]
            {
                new UrdfJoint("j0", "revolute", "a", "b", 0, 0, 0, 0, 0, 1, -1, 1),
                new UrdfJoint("j1", "revolute", "b", "c", 0, 0, 0, 0, 0, 1, -1, 1),
                new UrdfJoint("j2", "revolute", "c", "a", 0, 0, 0, 0, 0, 1, -1, 1),
            }));

        Assert.Contains("cycle", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    private static RobotDescription BuildGripper() => RobotDescription.Assemble(
        "gripper",
        new[] { new UrdfLink("palm"), new UrdfLink("L"), new UrdfLink("R") },
        new[]
        {
            new UrdfJoint("j_left", "revolute", "palm", "L", 0, 0, 0, 0, 0, 1, 0, 0.8),
            new UrdfJoint("j_right", "revolute", "palm", "R", 0, 0, 0, 0, 0, 1, 0, 0.8,
                mimicJoint: "j_left", mimicMultiplier: -1),
        });

    [Fact]
    public void Assemble_MimicJoint_GripperDriverCountIsOne()
    {
        var tree = BuildGripper().ToKinematicTree();

        Assert.Equal(1, tree.DriverCount);
        Assert.Equal(3, tree.Links.Count);

        var driverIdx = IndexOfJoint(tree, "j_left");
        var driver = tree.Joints[driverIdx];
        var follower = tree.Joints[IndexOfJoint(tree, "j_right")];

        Assert.Equal(0, driver.DriverIndex);
        Assert.Null(driver.Mimic);

        Assert.Equal(-1, follower.DriverIndex);
        Assert.NotNull(follower.Mimic);
        Assert.Equal(-1, follower.Mimic!.Value.Multiplier);
        Assert.Equal(driverIdx, follower.Mimic.Value.JointIndex);
    }

    private static RobotDescription BuildTwoJointArm(string? tipLink = "tool0") => RobotDescription.Assemble(
        "arm",
        new[] { new UrdfLink("base_link"), new UrdfLink("link1"), new UrdfLink("tool0") },
        new[]
        {
            new UrdfJoint("j0", "revolute", "base_link", "link1", 0, 0, 0.2, 0, 0, 1, -Math.PI, Math.PI),
            new UrdfJoint("j1", "revolute", "link1", "tool0", 0.3, 0, 0, 0, 0, 1, -Math.PI, Math.PI),
        },
        tipLink: tipLink);

    [Fact]
    public void Attach_GripperOntoArmTool0_CombinesTreeAndPreservesMimic()
    {
        var arm = BuildTwoJointArm();
        var gripper = BuildGripper();

        var tipBefore = arm.ToKinematicTree().ExtractSerialTip("base_link", "tool0").Chain.Joints.Length;

        var combined = arm.Attach(gripper, "tool0", new Frame(0, 0, 0.05));
        var tree = combined.ToKinematicTree();

        // base_link, link1, tool0 (arm) + palm, L, R (gripper)
        Assert.Equal(6, tree.Links.Count);
        // j0, j1 (arm) + tool0_to_palm_fixed (attach) + j_left, j_right (gripper)
        Assert.Equal(5, tree.Joints.Count);
        // j0, j1, j_left drive; the attach fixed joint and the j_right mimic do not.
        Assert.Equal(3, tree.DriverCount);

        var driverIdx = IndexOfJoint(tree, "j_left");
        Assert.Equal(2, tree.Joints[driverIdx].DriverIndex);
        var follower = tree.Joints[IndexOfJoint(tree, "j_right")];
        Assert.NotNull(follower.Mimic);
        Assert.Equal(-1, follower.Mimic!.Value.Multiplier);
        Assert.Equal(driverIdx, follower.Mimic.Value.JointIndex);

        var attach = tree.Joints.Single(j => j.Name.Contains("tool0_to_palm", StringComparison.Ordinal));
        Assert.Equal(KinematicJointType.Fixed, attach.Type);
        Assert.Equal(0.05, attach.OriginZ, 9);

        var tipAfter = tree.ExtractSerialTip("base_link", "tool0").Chain.Joints.Length;
        Assert.Equal(tipBefore, tipAfter);
        Assert.Equal(2, tipAfter);
    }

    [Fact]
    public void Attach_RotatedFrame_ThrowsNotSupported()
    {
        var arm = BuildTwoJointArm();
        var gripper = BuildGripper();
        // 90° about Z: qw=qy=√2/2 style — non-identity rotation
        var rotated = new Frame(0, 0, 0.05, qw: Math.Sqrt(0.5), qx: 0, qy: 0, qz: Math.Sqrt(0.5));
        var ex = Assert.Throws<NotSupportedException>(() => arm.Attach(gripper, "tool0", rotated));
        Assert.Contains("identity rotation", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void Attach_LinkNameClash_Throws()
    {
        var arm = BuildTwoJointArm();
        var clash = RobotDescription.Assemble(
            "clash_tool",
            [new UrdfLink("tool0"), new UrdfLink("finger")],
            [new UrdfJoint("jf", "revolute", "tool0", "finger", 0, 0, 0, 0, 0, 1, 0, 1)]);
        var ex = Assert.Throws<InvalidOperationException>(() =>
            arm.Attach(clash, "tool0", Frame.Identity));
        Assert.Contains("link name clash", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void Attach_JointNameClash_Throws()
    {
        var arm = BuildTwoJointArm();
        var clash = RobotDescription.Assemble(
            "clash_j",
            [new UrdfLink("palm"), new UrdfLink("finger")],
            [new UrdfJoint("j0", "revolute", "palm", "finger", 0, 0, 0, 0, 0, 1, 0, 1)]);
        var ex = Assert.Throws<InvalidOperationException>(() =>
            arm.Attach(clash, "tool0", Frame.Identity));
        Assert.Contains("joint name clash", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void TryAssemble_EmptyLinks_Fails()
    {
        var ok = RobotDescription.TryAssemble(
            "empty", [], [], tipLink: null, out _, out var diagnostics);
        Assert.False(ok);
        Assert.Contains(diagnostics.Errors, e => e.Contains("at least one link", StringComparison.OrdinalIgnoreCase));
    }

    [Fact]
    public void Explode_AfterAssemble_RoundTripsLinkAndJointNames()
    {
        var original = BuildTwoJointArm();
        var (links, joints) = original.Explode();

        Assert.Equal(
            new[] { "base_link", "link1", "tool0" },
            links.Select(l => l.Name));

        var byName = joints.ToDictionary(j => j.Name, j => (j.ParentLink, j.ChildLink));
        Assert.Equal(("base_link", "link1"), byName["j0"]);
        Assert.Equal(("link1", "tool0"), byName["j1"]);
    }

    [Fact]
    public void TipTcp_ZeroOriginJoint_ReturnsIdentityFrame()
    {
        var desc = RobotDescription.Assemble(
            "identity_check",
            new[] { new UrdfLink("base_link"), new UrdfLink("link1") },
            new[]
            {
                new UrdfJoint("j0", "revolute", "base_link", "link1", 0, 0, 0, 0, 0, 1, -Math.PI, Math.PI),
            },
            tipLink: "link1");

        var tcp = desc.TipTcp();

        Assert.Equal(Frame.Identity, tcp);
    }

    [Fact]
    public void ToKinematicTree_JointAxis_StoredAsGivenInParentFrame()
    {
        var desc = RobotDescription.Assemble(
            "axis_check",
            new[] { new UrdfLink("base_link"), new UrdfLink("link1") },
            new[]
            {
                new UrdfJoint("j0", "revolute", "base_link", "link1",
                    0.1, 0.2, 0.3, 0, 0, 1, -1, 1),
            });

        var tree = desc.ToKinematicTree();
        var joint = tree.Joints[0];

        Assert.Equal(0, joint.AxisX, 12);
        Assert.Equal(0, joint.AxisY, 12);
        Assert.Equal(1, joint.AxisZ, 12);
    }

    [Fact]
    public void KinematicTreeAttach_RemapsMechanismDriverIndexIntoCombinedDriverQ()
    {
        var arm = BuildTwoJointArm().ToKinematicTree();
        var gripper = BuildGripper().ToKinematicTree();
        Assert.Equal(2, arm.DriverCount);
        Assert.Equal(1, gripper.DriverCount);

        var merged = arm.Attach("tool0", gripper, gripper.Links[gripper.RootLinkIndex].Name, Frame.Identity);
        Assert.Equal(3, merged.DriverCount);

        var left = merged.Joints[IndexOfJoint(merged, "j_left")];
        Assert.Equal(2, left.DriverIndex); // after arm's two drivers

        var right = merged.Joints[IndexOfJoint(merged, "j_right")];
        Assert.Equal(-1, right.DriverIndex);
        Assert.NotNull(right.Mimic);
        Assert.Equal(IndexOfJoint(merged, "j_left"), right.Mimic!.Value.JointIndex);

        // Tip path AxisCount unchanged (arm only).
        Assert.Equal(2, merged.ExtractSerialTip("base_link", "tool0").Chain.Joints.Length);
    }

    [Fact]
    public void KinematicTreeAttach_RequiresMechanismRootLink()
    {
        var arm = BuildTwoJointArm().ToKinematicTree();
        var gripper = BuildGripper().ToKinematicTree();
        var nonRoot = gripper.Links.First(l => l.Name != gripper.Links[gripper.RootLinkIndex].Name).Name;

        var ex = Assert.Throws<ArgumentException>(() =>
            arm.Attach("tool0", gripper, nonRoot, Frame.Identity));
        Assert.Contains("mechanism root", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void KinematicTreeAttach_RejectsJoinNameClashWithMechanismJoint()
    {
        var arm = BuildTwoJointArm().ToKinematicTree();
        // Join name is "{onto}_to_{root}_fixed" — plant that name on the mechanism.
        var joinName = "tool0_to_palm_fixed";
        var gripperDesc = RobotDescription.Assemble("grip",
            [
                new UrdfLink("palm"),
                new UrdfLink("finger"),
            ],
            [
                new UrdfJoint(joinName, "revolute", "palm", "finger",
                    0, 0, 0.01, 0, 0, 1, -1, 1),
            ]);
        var gripper = gripperDesc.ToKinematicTree();

        var ex = Assert.Throws<InvalidOperationException>(() =>
            arm.Attach("tool0", gripper, "palm", Frame.Identity));
        Assert.Contains(joinName, ex.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void Project_DefaultsTipToDescriptionTipLink()
    {
        var arm = BuildTwoJointArm();
        Assert.Equal("tool0", arm.TipLink);

        var (_, tip) = RobotDescriptionSession.Project(arm);
        Assert.NotNull(tip);
        Assert.Equal(2, tip!.Value.Chain.Joints.Length);
    }

    [Fact]
    public void TipTcp_UsesHomeQ_WhenNonZero()
    {
        var atZero = BuildTwoJointArm();
        var tcpZero = atZero.TipTcp();

        var posed = RobotDescription.Assemble(
            "arm",
            new[] { new UrdfLink("base_link"), new UrdfLink("link1"), new UrdfLink("tool0") },
            new[]
            {
                new UrdfJoint("j0", "revolute", "base_link", "link1", 0, 0, 0.2, 0, 0, 1, -Math.PI, Math.PI),
                new UrdfJoint("j1", "revolute", "link1", "tool0", 0.3, 0, 0, 0, 0, 1, -Math.PI, Math.PI),
            },
            tipLink: "tool0",
            homeQ: new[] { Math.PI / 2, 0.0 });

        var tcpHome = posed.TipTcp();
        Assert.True(
            Math.Abs(tcpZero.X - tcpHome.X) + Math.Abs(tcpZero.Y - tcpHome.Y) + Math.Abs(tcpZero.Z - tcpHome.Z) > 1e-6,
            "HomeQ must move TipTcp off the q=0 origin sum.");
    }

    private static int IndexOfJoint(KinematicTree tree, string name)
    {
        for (var i = 0; i < tree.Joints.Count; i++)
            if (tree.Joints[i].Name == name)
                return i;
        throw new InvalidOperationException($"Joint '{name}' not found.");
    }
}
