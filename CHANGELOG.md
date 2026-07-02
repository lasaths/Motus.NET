# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/lasaths/Motus.NET/compare/v0.3.0...HEAD
[0.3.0]: https://github.com/lasaths/Motus.NET/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/lasaths/Motus.NET/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/lasaths/Motus.NET/releases/tag/v0.1.0
