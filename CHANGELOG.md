# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.3.3] - 2026-07-05

### Added

- `RobotModel.JointNames` metadata for URDF chain order and bundled UR presets (`BundledJointNames`)
- `TrajectoryExport` emits `jointNames` and optional per-point named `joints` map for viewer round-trip
- URDF FK cross-check tests (`UrdfFkCrossCheckTests`, `fk_cases.json`) and `viewer_presets.json` home/demo paths
- Dev URDF viewer (`tools/urdf-viewer`) with TCP path, named joint export, and Z-up handling

### Fixed

- `Transforms.Multiply` / `TransformPoint` column-major composition bug affecting FK and collision poses
- `CartesianLinearPathPlanner` LIN trajectories retimed via `TrajectoryRetimer` (seconds, not frame indices)
- `RobotMeshCollisionChecker` accepts optional `SerialJointChain` for URDF kinematics

### Changed

- `PresetLoader` attaches joint names to bundled robot models
- `UrdfRobotLoader.ToModel()` passes through `JointNames`
- Version bumped to **0.3.3**

## [0.3.2] - 2026-07-03

### Fixed

- Publish `Motus.OMPL.Native` on NuGet so `Motus.OMPL.NET` restores cleanly

## [0.3.1] - 2026-07-03

### Added

- UR10e preset `collisionLinks` and `tests/fixtures/ur10e_collision.urdf`

### Fixed

- `RrtConnectPlanner` honors `PlanningOptions.CollisionChecker` (e.g. `RobotMeshCollisionChecker`) instead of always using internal sphere envelopes
- `native-ompl` CI: CMake link against Ubuntu `libompl-dev`, OMPL 1.5 `PathGeometric` API compatibility

## [0.3.0] - 2026-07-02

### Added

- `RobotCollisionModel`, `LinkCollisionGeometry`, and JSON preset `collisionLinks` (capsule/box/mesh)
- `RobotMeshCollisionChecker` — FK-transformed per-link collision with sphere fallback
- URDF `<collision>` import and `SrdfLoader` for allowed collision pairs
- `RetimerAlgorithm.Bottleneck` path-wide retiming (default for `TrajectoryExport` when `retime=true`)
- `Transforms.TransformPoint`, `IFkSolver.ComputeLinkTransforms`
- Optional `native-ompl` CI job (Ubuntu + `libompl-dev`)
- `tests/fixtures/ur5e_collision.urdf`, `docs/urdf-import.md`, Phase 2 regression tests

### Changed

- Denser swept capsule sampling scaled by max joint delta per segment
- UR IK: enumerate analytic branches with FK verify; multi-seed numerical fallback (±10% joint span)
- `PresetLoader.LoadRobotModelByName/FromFile` returns `RobotModel` with optional collision model
- UR5e preset includes approximate per-link collision capsules
- Version bumped to **0.3.0**

## [0.2.0] - 2026-07-02

### Added

- `PlanningOptions.CollisionChecker` and `PlanningCollision` segment validation on planned paths
- `TrajectoryRetimer` — trapezoidal joint-space retiming; `TrajectoryExport` `retime` flag
- `CartesianLinearPathPlanner.PlanToResult` — LIN planning with optional collision validation
- BVH mesh collision enabled in `MeshCollisionChecker`; `CollisionObject.Mesh` for triangle obstacles
- `motus-net.version` semver file for downstream consumers
- Native OMPL implementation (`motus_ompl_ompl.cpp`) behind `MOTUS_USE_OMPL=ON`

### Changed

- `JointLinearPlanner` / `CartesianLinearPlanner` fail loudly when a collision scene is set without a checker
- `NumericalInverseKinematics` prefers continuity (minimum joint delta from seed)
- `RrtConnectPlanner` passes collision checker through segment interpolation; native P/Invoke when OMPL is built
- Version bumped to **0.2.0**
- `UrInverseKinematics` — analytic IK for Universal Robots presets
- SRDF-lite `CollisionScene.AllowedPairs`; capsule link-envelope sampling
- `TrajectoryExport.Export()` with retime + validate; jerk-aware retimer
- URDF prismatic joints in serial chains

## [0.1.0] - 2026-06-28

Initial public release.

### Added

- **Motus.Core** — neutral robot model, joint states, trajectories, `IPlanner`
  interface, `JointLinearPlanner`, trajectory validation (limits, velocity,
  acceleration, optional collision), and JSON/CSV export.
- **Motus.Geometry** — DH forward kinematics, numerical inverse kinematics
  (UR/KUKA profiles), `SphereCollisionChecker`, and `CartesianLinearPlanner`.
- **Motus.OMPL.NET** — pure C# collision-aware RRT-Connect joint-space planner
  and `PathSimplifier` (no native OMPL dependency).
- **Motus.Presets** — JSON preset loader with bundled UR and KUKA defaults
  (approximate public datasheet values for planning/visualization only).

[Unreleased]: https://github.com/lasaths/Motus.NET/compare/v0.3.3...HEAD
[0.3.3]: https://github.com/lasaths/Motus.NET/compare/v0.3.2...v0.3.3
[0.3.2]: https://github.com/lasaths/Motus.NET/compare/v0.3.1...v0.3.2
[0.3.1]: https://github.com/lasaths/Motus.NET/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/lasaths/Motus.NET/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/lasaths/Motus.NET/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/lasaths/Motus.NET/releases/tag/v0.1.0
