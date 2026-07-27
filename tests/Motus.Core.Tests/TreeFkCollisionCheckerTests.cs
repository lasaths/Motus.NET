using System.Xml.Linq;
using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class TreeFkCollisionCheckerTests
{
    [Fact]
    public void TreeFk_SideBranchYaw_HitsKeepout_TipSerialBlind()
    {
        const string urdf = """
            <?xml version="1.0"?>
            <robot name="arm_dkp">
              <link name="world"/>
              <link name="base_link">
                <collision><origin xyz="0 0 0.05"/><geometry><cylinder length="0.1" radius="0.05"/></geometry></collision>
              </link>
              <link name="link1">
                <collision><origin xyz="0.1 0 0"/><geometry><cylinder length="0.2" radius="0.03"/></geometry></collision>
              </link>
              <link name="tip_link"/>
              <link name="dkp_base">
                <collision><origin xyz="0 0 0.05"/><geometry><cylinder length="0.1" radius="0.08"/></geometry></collision>
              </link>
              <link name="dkp_yaw_link">
                <collision><origin xyz="0.12 0 0"/><geometry><box size="0.24 0.08 0.04"/></geometry></collision>
              </link>
              <joint name="world_base" type="fixed"><parent link="world"/><child link="base_link"/><origin xyz="0 0 0"/></joint>
              <joint name="j1" type="revolute">
                <parent link="base_link"/><child link="link1"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
                <limit lower="-3.14" upper="3.14" velocity="1"/>
              </joint>
              <joint name="j_tip" type="fixed"><parent link="link1"/><child link="tip_link"/><origin xyz="0.2 0 0"/></joint>
              <joint name="dkp_fixed" type="fixed"><parent link="world"/><child link="dkp_base"/><origin xyz="1.0 0 0"/></joint>
              <joint name="dkp_yaw" type="revolute">
                <parent link="dkp_base"/><child link="dkp_yaw_link"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
                <limit lower="-3.14" upper="3.14" velocity="1"/>
              </joint>
            </robot>
            """;

        var doc = XDocument.Parse(urdf);
        var tree = UrdfRobotLoader.LoadTree(doc);
        var tip = UrdfRobotLoader.Load(doc, new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tip_link",
            ModelName = "arm_dkp"
        });
        var col = UrdfCollisionLoader.LoadTree(doc.Root!, tree, ".");
        Assert.NotNull(col);
        Assert.Contains(col!.Links, l => l.LinkName == "dkp_yaw_link");

        var planNames = tip.JointNames.Concat(["dkp_yaw"]).ToList();
        var limits = tip.Preset.JointLimits.ToList();
        limits.Add(new JointLimit(-Math.PI, Math.PI, Math.PI, 2 * Math.PI));
        var robot = new RobotModel(new RobotPreset
        {
            Manufacturer = tip.Preset.Manufacturer,
            ModelName = "arm_dkp_all",
            Family = tip.Preset.Family,
            AxisCount = 2,
            JointLimits = limits,
            ReachMeters = tip.Preset.ReachMeters,
            PayloadKg = tip.Preset.PayloadKg,
            BaseFrame = tip.Preset.BaseFrame,
            ToolFrame = tip.Preset.ToolFrame
        }, col, planNames);

        var treeChecker = new TreeFkCollisionChecker(robot, tree, tip.Chain, planNames, new double[tree.DriverCount]);
        var tipChecker = CollisionCheckerFactory.Create(tip.ToModel(), tip.Chain);
        var keepout = new CollisionScene([
            CollisionObject.Box("keepout", new Frame(1.0, 0.22, 0.12), 0.05, 0.04, 0.03)
        ]);

        var q0 = new JointState([0.0, 0.0]);
        var qYaw = new JointState([0.0, 1.2]);
        Assert.True(treeChecker.IsCollisionFree(q0, keepout));
        Assert.False(treeChecker.IsCollisionFree(qYaw, keepout));
        Assert.True(tipChecker.IsCollisionFree(new JointState([0.0]), keepout));
        Assert.IsType<TreeFkCollisionChecker>(
            CollisionCheckerFactory.GetOrCreate(robot, tree, tip.Chain, planNames, new double[tree.DriverCount], null, keepout));
    }
}
