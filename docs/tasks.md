# Task data and validation

`TaskInstance` stores a reusable string identity, named world-space frames, and metadata. `TaskPath` preserves task and frame order. These types describe offline planning intent; they do not send robot commands.

`CoreTasks` supplies typed pick, place, walk, and park factories/views. Jaw openings are millimeters (`jawMm`), frame positions are meters, and joint coordinates follow each `JointLimit.Unit`. Factories reject empty object names, invalid walk modes, and non-finite or negative jaw openings. Task frames require finite positions and a nonzero finite quaternion.

`TaskSequenceRules.Validate` checks one held object at a time: pick before place/park, matching object identity, no walk while carrying, and no object left held or parked at the end. It does not prove geometric reachability or structural strength.

```csharp
var tasks = new TaskPath(
    CoreTasks.Pick("part", jawMm: 14, frames: new Dictionary<string, Frame> {
        ["grasp"] = graspFrame
    }),
    CoreTasks.Place("part", jawMm: 40, frames: new Dictionary<string, Frame> {
        ["release"] = releaseFrame
    }));

var sequenceProblems = TaskSequenceRules.Validate(tasks.Tasks);
var geometry = TaskPathValidator.ValidateWithMotion(tasks, robot, ik, checker, scene,
    configurationStep: 0.01);
if (sequenceProblems.Count == 0 && geometry.IsValid) {
    // ReachableStates can become input to an appropriate trajectory planner.
}
```

`Validate` checks each frame's IK solution, joint limits, and endpoint collision. `ValidateWithMotion` also checks sampled joint-space segments between the solutions. It does not assert Cartesian-linear motion or continuous collision freedom between samples. The first frame is the motion start; the seed is an IK hint rather than a validated approach segment. `configurationStep` must be finite and positive and uses each axis's coordinate unit (radians or meters).

Configure the robot, IK solver, and collision checker with matching base/tool frames. The optional frame arguments must match the robot preset: different overrides return an explicit error because the generic solver/checker interfaces cannot reconfigure both kinematics and collision together. Invalid seeds, failed IK, invalid returned states, and colliding endpoints/segments also produce validation errors. Review `IsValid` and `Errors` before consuming `ReachableStates`; failures can leave a partial list.

The optional [bamboo viewer](../src/Motus.Viewer/README.md) maps task data to its own planner-confirmed assembly sequence. Its reach-map prefilter is a necessary reachability check; only the subsequent collision-checked planner confirms a build choice. Its rigid-group rule is a preview model, without a load, stiffness, or physical assembly guarantee.
