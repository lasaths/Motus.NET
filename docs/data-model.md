# Data Model

## Robot

- **`RobotManufacturer`** — enum (`UniversalRobots`, `Kuka`, …)
- **`RobotPreset`** — immutable planning defaults from JSON (limits, frames, metadata)
- **`RobotModel`** — runtime wrapper around a preset for planning

## Kinematics state

- **`JointState`** — joint angles in **radians**; validates against `JointLimit` list
- **`JointLimit`** — min/max radians; optional max velocity and acceleration
- **`Frame`**, **`BaseFrame`**, **`ToolFrame`** — position (meters) + unit quaternion
- **`CartesianPose`** — frame + metadata placeholder for future Cartesian planning

## Trajectory

- **`TrajectoryPoint`** — `TimeSeconds` + `JointState`
- **`Trajectory`** — ordered points bound to a `RobotModel`; exposes `DurationSeconds`

## Planning

- **`PlanningRequest`** — robot, start, goal, `PlanningOptions`
- **`PlanningOptions`** — `MaxJointStepRadians`, `TimeStepSeconds`
- **`PlanningResult`** — success flag, trajectory, errors, warnings
- **`ValidationResult`** — validity, errors, warnings

## Collision (placeholder)

- **`CollisionScene`**, **`CollisionObject`** — stubs for future collision-aware planners

## Export

- **`TrajectoryExport.ToJson`** / **`ToCsv`** — neutral serializations for downstream tools
