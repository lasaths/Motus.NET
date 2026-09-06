# Pick/place programs (0.17.0 development)

A cycle travels to pickup hover, approaches with LIN, closes, attaches, lifts with LIN,
travels to placement hover, approaches with LIN, opens, detaches, and retracts with LIN.
The part stops moving at detach. It remains a scene obstacle for retraction and later cycles.
All distances are meters, serial joints radians, and timestamps seconds.

```csharp
var segments = PickPlaceCycle.ExpandMany(
    grasps, places, objects, 0.08, openState, closedState,
    options: new PickPlaceOptions {
        UseSamplingTransfers = true,
        TouchBodies = new[] { "robotiq_2f85" }
    });
var result = new IndustrialMotionPlanner(robot.Preset, chain).Plan(
    new MotionProgramRequest(robot, start, segments, new PlanningOptions {
        CollisionScene = scene, // Table, all source parts, fixtures, existing placed parts.
        CollisionChecker = CollisionCheckerFactory.Create(robot, chain), // Robot only.
        RetimeTrajectory = true
    }) {
        InitialToolState = openState,
        TransferPlannerFactory = checker => SamplingPlanner.Create(checker,
            new SamplingPlannerOptions {
                PlannerId = SamplingPlannerId.RrtConnect,
                RandomSeed = 42,
                PreferManaged = true
            })
    });
if (result.Success) {
    var export = TrajectoryExport.Export(result.Trajectory!);
    File.WriteAllText("pick-place.json", export.Json);
}
```

`TouchBodies` names must match collision checker identifiers: tool geometry names
(for example `robotiq_2f85`) or explicit `CollisionBodies.RobotLink(index)` identifiers.
The default is empty. Exceptions permit only the named bodies against the current
part during the grasp/close and detach/retract segments. They do not hide objects,
affect arm links that were not named, or persist into later transfers. Account for
intentional part/support contact using explicitly named scene allowed pairs where
needed; do not remove the table or other parts to obtain a successful plan.

`UseSamplingTransfers` defaults to false. When enabled, only travel to pickup hover
and between pickup/placement hovers uses `TransferSegment`. A transfer resolves a
Cartesian goal through IK and plans in joint space; its TCP path is not LIN.
The supplied factory receives the current checker, including the carried volume,
and its request contains the current scene, constraints, and group options. A
missing factory returns `planning.planner_unavailable`; it never silently substitutes
an unchecked straight transfer. Geometry has no dependency on the OMPL package.

Motion programs expect `Options.CollisionChecker` to check the robot only. They add
and remove attached-body checks themselves. If omitted, a checker is built from
the robot and serial chain. Source part names must be unique within ExpandMany.
Attaching an already attached name or detaching an unknown name fails explicitly.

## Timeline and export

`PlanningResult.AttachSpans` and `Trajectory.AttachSpans` share the same timeline.
Each span records the source scene name, geometry, TCP-local pose, start/end times,
and optional `ReleaseWorldPose`. Use `[startSeconds, endSeconds)` when a release
pose exists; at the end time draw the part at its fixed release pose. A null release
pose means the part remains attached through the trajectory end.

All four retimers preserve SET/WAIT duration, stop at unblended program and attachment
boundaries, and map attachment times to the corresponding retimed waypoints. Events
must coincide with trajectory waypoints. `TrajectoryExport.Export` prepares once;
its returned trajectory, JSON, and CSV use the same selected retimer and clock.

JSON adds optional `attachSpans` with body geometry and poses (quaternion `qw/qx/qy/qz`).
CSV adds `attachment_spans_json` when attachments exist: the first data row contains
the complete timeline JSON, later rows leave that cell empty. Use a CSV parser that
supports quoted fields and doubled quotes. Existing no-attachment JSON/CSV retain
their columns/fields. `tool_state_json` is now correctly escaped as a CSV cell.
The additive fields retain PlanBundle contract version 1.0.0.

`Validate` on export still validates against the supplied validation options. A
single static validation scene cannot reproduce a changing pick/place scene;
program collision checks use the scene active at each segment.

## Grasshopper

Motus Pick Place appends `RRT` (default false) and `Touch` (list of collision body
names). Wire the full source/obstacle scene to Motus Program. Program supplies the
RRT-Connect factory and maintains attached objects automatically. Preview uses the
release pose after detach, and Export retains the timeline, including after retiming.
Build against sibling Motus.NET with `-p:UseMotusNetProjectReference=true` until
0.17.0 packages are published.
