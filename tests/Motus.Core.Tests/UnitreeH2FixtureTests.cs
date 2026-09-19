using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>
/// Experimental Unitree H2 fixture smoke (meshless pelvis-rooted tree).
/// Honest scope: LoadTree + Tree FK + optional arm PlanningGroup — NOT biped walk/balance.
/// Do not confuse with Unitree H1-2 (<c>h1_2_description</c>).
/// </summary>
public class UnitreeH2FixtureTests
{
    private static string FixturePath(string name) =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", name));

    private static readonly string[] LeftArmJoints =
    [
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
    ];

    [Fact]
    public void LoadTree_H2Minimal_HasThirtyOneRevoluteDrivers()
    {
        var tree = UrdfRobotLoader.LoadTree(FixturePath("unitree_h2/h2_minimal.urdf"));

        Assert.Equal("h2_minimal", tree.Name);
        Assert.Equal("pelvis", tree.Links[tree.RootLinkIndex].Name);
        Assert.Equal(31, tree.DriverCount);
        Assert.DoesNotContain(tree.Joints, j => j.Type == KinematicJointType.Prismatic);
        Assert.All(
            tree.DriverJointIndices.Select(i => tree.Joints[i]),
            j => Assert.Equal(KinematicJointType.Revolute, j.Type));

        // Marketing layout: 6/leg ×2 + 7/arm ×2 + 3 waist + 2 head.
        Assert.Equal(6, CountDrivers(tree, "left_hip_", "left_knee_", "left_ankle_"));
        Assert.Equal(6, CountDrivers(tree, "right_hip_", "right_knee_", "right_ankle_"));
        Assert.Equal(7, CountDrivers(tree, "left_shoulder_", "left_elbow_", "left_wrist_"));
        Assert.Equal(7, CountDrivers(tree, "right_shoulder_", "right_elbow_", "right_wrist_"));
        Assert.Equal(3, CountDrivers(tree, "waist_"));
        Assert.Equal(2, CountDrivers(tree, "head_"));
    }

    [Fact]
    public void TreeFk_H2Minimal_HomeAndArmMove()
    {
        var tree = UrdfRobotLoader.LoadTree(FixturePath("unitree_h2/h2_minimal.urdf"));
        var fk = new TreeForwardKinematics(tree);
        var mats = AllocMats(tree.Links.Count);
        var qHome = new double[tree.DriverCount];

        fk.ComputeLinkTransformsInto(qHome, mats);
        var leftHand = tree.IndexOfLink("left_hand_link");
        var x0 = mats[leftHand][3];
        var y0 = mats[leftHand][7];
        var z0 = mats[leftHand][11];
        Assert.False(double.IsNaN(x0) || double.IsInfinity(x0));

        var qArm = (double[])qHome.Clone();
        for (var i = 0; i < tree.DriverCount; i++)
        {
            var name = tree.Joints[tree.DriverJointIndices[i]].Name;
            if (name == "left_shoulder_pitch_joint")
                qArm[i] = 0.35;
            else if (name == "left_elbow_joint")
                qArm[i] = -0.5;
        }

        fk.ComputeLinkTransformsInto(qArm, mats);
        var dist = Math.Sqrt(
            Math.Pow(mats[leftHand][3] - x0, 2) +
            Math.Pow(mats[leftHand][7] - y0, 2) +
            Math.Pow(mats[leftHand][11] - z0, 2));
        Assert.True(dist > 1e-3, $"left_hand_link should move under arm drive; dist={dist}");
    }

    [Fact]
    public void ExtractSerialTip_LeftArm_SevenRevolute()
    {
        var tree = UrdfRobotLoader.LoadTree(FixturePath("unitree_h2/h2_minimal.urdf"));
        var tip = tree.ExtractSerialTip("torso_link", "left_hand_link");
        Assert.Equal(7, tip.Chain.Joints.Length);
        Assert.Equal(LeftArmJoints, tip.JointNames);
    }

    [Fact]
    public void PlanningGroup_LeftArm_LocksNonArmDrivers()
    {
        // Experimental: one-arm group over the full 31R tree — not locomotion.
        var tree = UrdfRobotLoader.LoadTree(FixturePath("unitree_h2/h2_minimal.urdf"));
        var (robot, jointNames) = ToFullDriverModel(tree);
        Assert.Equal(31, robot.Preset.AxisCount);

        var start = new JointState(new double[31]);
        var goalPos = new double[31];
        for (var i = 0; i < 31; i++)
        {
            var name = jointNames[i];
            if (name == "left_shoulder_pitch_joint") goalPos[i] = 0.4;
            else if (name == "left_elbow_joint") goalPos[i] = -0.6;
            else if (name == "left_wrist_yaw_joint") goalPos[i] = 0.25;
            // Non-arm drivers intentionally differ from start so lock must hold them.
            else if (name.StartsWith("right_", StringComparison.Ordinal) ||
                     name.StartsWith("left_hip_", StringComparison.Ordinal) ||
                     name.StartsWith("left_knee_", StringComparison.Ordinal) ||
                     name.StartsWith("waist_", StringComparison.Ordinal))
                goalPos[i] = 0.15;
        }
        var goal = new JointState(goalPos);

        var group = new PlanningGroup("left_arm", "torso_link", "left_hand_link", LeftArmJoints);
        var opts = new PlanningOptions
        {
            GroupMap = JointIndexMap.Resolve(robot, group),
            MaxJointStepRadians = 0.08
        };

        var result = new JointLinearPlanner().Plan(new PlanningRequest(robot, start, goal, opts));
        Assert.True(result.Success, string.Join("; ", result.Errors));

        var lockedNames = jointNames.Where(n => !LeftArmJoints.Contains(n, StringComparer.OrdinalIgnoreCase)).ToArray();
        foreach (var pt in result.Trajectory!.Points)
        {
            for (var i = 0; i < 31; i++)
            {
                if (LeftArmJoints.Contains(jointNames[i], StringComparer.OrdinalIgnoreCase))
                    continue;
                Assert.Equal(0.0, pt.JointState.Positions[i], 1e-9);
            }
        }

        var last = result.Trajectory.Points[^1].JointState.Positions;
        Assert.Equal(0.4, last[IndexOf(jointNames, "left_shoulder_pitch_joint")], 6);
        Assert.Equal(-0.6, last[IndexOf(jointNames, "left_elbow_joint")], 6);
        Assert.Equal(0.25, last[IndexOf(jointNames, "left_wrist_yaw_joint")], 6);
        Assert.True(lockedNames.Length >= 24);
    }

    private static int CountDrivers(KinematicTree tree, params string[] prefixes)
    {
        var n = 0;
        for (var i = 0; i < tree.DriverCount; i++)
        {
            var name = tree.Joints[tree.DriverJointIndices[i]].Name;
            if (prefixes.Any(p => name.StartsWith(p, StringComparison.Ordinal)))
                n++;
        }
        return n;
    }

    private static (RobotModel Model, string[] JointNames) ToFullDriverModel(KinematicTree tree)
    {
        var names = new string[tree.DriverCount];
        var limits = new List<JointLimit>(tree.DriverCount);
        for (var i = 0; i < tree.DriverCount; i++)
        {
            var j = tree.Joints[tree.DriverJointIndices[i]];
            names[i] = j.Name;
            var lo = double.IsFinite(j.Lower) ? j.Lower : -Math.PI;
            var hi = double.IsFinite(j.Upper) ? j.Upper : Math.PI;
            var vel = j.Velocity is > 0 and var v ? v : Math.PI;
            limits.Add(new JointLimit(lo, hi, vel, vel * 2));
        }

        var preset = new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = "Unitree H2 (experimental tree)",
            // Intentionally not Family=humanoid GA — tree FK / arm group only.
            Family = "urdf",
            AxisCount = tree.DriverCount,
            JointLimits = limits,
            BaseFrame = BaseFrame.Identity,
            ToolFrame = ToolFrame.Identity,
            Notes = "Experimental H2 meshless tree — not biped walk/balance.",
        };
        return (new RobotModel(preset, jointNames: names), names);
    }

    private static int IndexOf(IReadOnlyList<string> names, string name)
    {
        for (var i = 0; i < names.Count; i++)
        {
            if (string.Equals(names[i], name, StringComparison.OrdinalIgnoreCase))
                return i;
        }
        throw new InvalidOperationException($"Missing joint '{name}'.");
    }

    private static double[][] AllocMats(int n)
    {
        var mats = new double[n][];
        for (var i = 0; i < n; i++)
            mats[i] = new double[16];
        return mats;
    }
}
