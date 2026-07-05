# Data Model

## Robot

- **`RobotManufacturer`** — enum (`UniversalRobots`, `Kuka`, …)
- **`RobotPreset`** — immutable planning defaults from JSON (limits, frames, metadata)
- **`RobotModel`** — runtime wrapper around a preset for planning
- **`ToolFrame`** — static TCP offset from flange (no separate end-effector type)

## Kinematics state

- **`JointState`** — joint angles in **radians**; validates against `JointLimit` list
- **`JointLimit`** — min/max radians; optional max velocity and acceleration
- **`Frame`**, **`BaseFrame`**, **`ToolFrame`** — position (meters) + unit quaternion
- **`CartesianPose`** — TCP frame for Cartesian planning

## Trajectory

- **`TrajectoryPoint`** — `TimeSeconds` + `JointState`
- **`Trajectory`** — ordered points bound to a `RobotModel`; exposes `DurationSeconds`

## Planning

- **`PlanningRequest`** — robot, start, goal joint states, `PlanningOptions`
- **`CartesianPlanningRequest`** — robot, start joints, Cartesian goal, options
- **`PlanningOptions`** — step/timing limits; optional `CollisionScene`, `AttachedBodies`
- **`PlanningContext`** — robot + scene + attach/detach lifecycle (hides scene obstacle on attach)
- **`PlanningResult`** — success flag, trajectory, errors, warnings
- **`ValidationResult`** — validity, errors, warnings
- **`PlanningGroup`** / **`JointIndexMap`** — SRDF group joint subsets

## Attached objects

- **`AttachedBody`** — runtime grasped geometry in TCP-local frame
- **`PlanningContext.Attach(sceneObjectName, geometry, tcpLocalPose)`** — removes matching obstacle from scene
- **`CollisionCheckerFactory.Create(robot, attached: …)`** — attach-aware collision checking (C# path)

## Collision

- **`CollisionObject`** — sphere, box, capsule, or mesh in local/world frame
- **`CollisionScene`** — list of obstacles + `AllowedPairs` (SRDF-lite)
- **`RobotCollisionModel`** — per-link geometry + optional `ToolGeometry` (static gripper)
- **`ICollisionChecker`** — `SphereCollisionChecker`, `RobotMeshCollisionChecker`, `AttachAwareCollisionChecker`, `FclCollisionChecker` (when native FCL built)

## Native (`motus_native`)

- **`Motus.Native`** — unified P/Invoke (`motus_ompl_*`, `motus_fcl_*`)
- **`MotusCapabilities`** (`Motus.OMPL.NET`) — `NativeOmpl`, `NativeFcl`, `Describe()`
- **`OmplPlannerOptions`** — `PlannerId` (`RrtConnect`, `RrtStar`), `MaxPlanTimeSeconds`, motion validation

## Kinematics (Motus.Geometry)

- **`KinematicsProfiles`** — built-in DH chains per preset model name
- **`DhForwardKinematics`** / **`NumericalInverseKinematics`** — FK/IK backends
- **`KinematicsResolver`** — factory from `RobotPreset`

## Export

- **`TrajectoryExport.ToJson`** / **`ToCsv`** — neutral serializations for downstream tools
