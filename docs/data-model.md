# Data Model

## Robot

- **`RobotManufacturer`** — enum (`UniversalRobots`, `Kuka`, …)
- **`RobotPreset`** — immutable planning defaults from JSON (limits, frames, metadata)
- **`RobotModel`** — runtime wrapper around a preset for planning

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
- **`PlanningOptions`** — step/timing limits; optional `CollisionScene`
- **`PlanningResult`** — success flag, trajectory, errors, warnings
- **`ValidationResult`** — validity, errors, warnings

## Collision

- **`CollisionObject`** — sphere or axis-aligned box in world frame
- **`CollisionScene`** — list of obstacles
- **`ICollisionChecker`** — implemented by `SphereCollisionChecker` in Motus.Geometry

## Kinematics (Motus.Geometry)

- **`KinematicsProfiles`** — built-in DH chains per preset model name
- **`DhForwardKinematics`** / **`NumericalInverseKinematics`** — FK/IK backends
- **`KinematicsResolver`** — factory from `RobotPreset`

## Export

- **`TrajectoryExport.ToJson`** / **`ToCsv`** — neutral serializations for downstream tools
