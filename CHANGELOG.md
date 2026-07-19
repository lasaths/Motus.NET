# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `KinematicTree` / `TreeForwardKinematics` / `ReachSampling` — kinematic tree FK with mimic expansion, tip-chain extract for legacy TCP, capped Halton reach samples
- `UrdfRobotLoader.LoadTree` + URDF `<mimic>` parse; `Load` wraps tip extract and exposes `UrdfRobot.Tree`

## [0.7.0] - 2026-07-19

### Added

- `CollisionShape.Plane` / `CollisionObject.Plane` — infinite half-space (Motus local +X free); O(1) signed-distance tests in managed checkers
- Plane obstacles auto-allow proximal `link:-1..1` pairs (robot + floor at origin)

## [0.6.9] - 2026-07-19

### Fixed

- Plane-goal LIN keeps IK on one joint branch (no random reseed / π-scale flips) so wrist exits stay continuous
- Managed RRT-Connect materializes `JointState` copies when group embedding reuses a scratch buffer — stops aliasing both segment ends / path waypoints onto one array

## [0.6.8] - 2026-07-18

### Changed

- Further mesh-collision / planning hot-path allocation cuts: reuse FK and link-transform buffers, skip unchanged BVH rebuilds, hash allowed pairs, densify post-RRT paths without re-running `JointLinearPlanner` validation
- Root README tightened for quicker install / first-plan guidance

## [0.6.7] - 2026-07-18

### Changed

- **Managed mesh collision hot path** — keep robot link meshes in local space; pose-only transforms with AABB broadphase and conservative link envelopes before triangle SAT (no per-state world-mesh vertex copies)
- **`CollisionObject.ContentHash`** — geometry fingerprint computed once at construction; BVH / session caches use it instead of re-hashing verts
- **`BvhNode.GetPotentialTriangles`** — caller-owned list overload to avoid per-query allocations
- **`RobotMeshCollisionChecker.SegmentCollisionFree`** — reuses joint buffer via `JointState.Wrap`
- **`PlanningCollision.ValidateTrajectory`** — skips redundant `SegmentCollisionFree` when consecutive waypoints already lie within the step size
- BVH caches keyed by content hash (not mesh name) in `MeshCollisionChecker` / `AttachAwareCollisionChecker`

### Added

- `Transforms.TransformPointInto` — allocation-free point transform
- `JointState.Wrap` — adopt an existing joint buffer without copying
- `Example03CollisionRrtPerfTests` — UR10e + ColSphere scenario aligned with Motus.Grasshopper `examples/03_collision_rrt.ghx`

## [0.6.6] - 2026-07-13

### Added

- Plan contract metadata in `TrajectoryExport`: `contractVersion`, `units`, `frameConvention`, optional `diagnostics`, optional `provenance`
- `PlanningMessage`, `PlanningMessageSeverity`, and `PlanningMessageCodes` for machine-readable planner diagnostics
- `PlanBundleContractTests` golden fixtures for joint/cartesian/sampling/motion-program export payloads

### Changed

- `PlanningResult` now carries structured `Messages` alongside existing `Errors`/`Warnings`
- `SamplingPlanner` and collision preflight paths emit typed planning message codes on failures

## [0.6.4] - 2026-07-12

### Added

- **`ToolCollisionPlacement`** — shared tool collision world matrix (FK tip vs TCP placement)
- **`ToolDefinition.GeometryInFlangeFrame` / `GeometryAttachOffset`** — host-supplied tool collision frames
- **`RobotCollisionModel.ToolGeometryInFlangeFrame` / `ToolGeometryAttachOffset`** — planning collision model metadata
- **`UrdfFixedChain.TryTipAttachOffset`** — fixed URDF chain from last actuated link to tip (e.g. wrist_3 → tool0)

### Fixed

- **Robotiq 2F-85 tool collision** — bundled `robotiq_2f85` mesh placed at FK tip with wrist_3→tool0 offset instead of mis-rotated TCP frame (fixes false start-configuration collisions vs visuals)
- **`RobotModel.WithTool`** — propagates tool attach offset into session collision model
- **`RobotMeshCollisionChecker` / `FclCollisionChecker`** — honor tool attach offset during planning collision checks

## [0.6.3] - 2026-07-12

### Added

- **Sampling planner registry** — `SamplingPlannerId`, `SamplingPlannerRegistry`, `SamplingPlanner` façade with managed + native backends
- Registered planners: RRT-Connect (managed fallback), RRT*, AORRTC, LBKPIECE, AIT*, EIT*, BLIT*, ParallelRace meta-planner
- `ManagedRrtConnect`, `NativeOmplPlanner`, `ParallelRacePlanner`, `PlanningPipeline` — shared planning pipeline
- `VampValidationBackend` stub (Phase 5 hook)
- Native C ABI: `motus_ompl_plan`, `motus_ompl_planner_available`, planner IDs 2–6; goal_bias wired for RRT-Connect
- `CollisionMeshCache`, `CollisionCheckerSessionCache`, `CollisionCheckerFactory.GetOrCreate` — mesh BVH + checker reuse
- `scripts/build-native-full.sh/.ps1`; Linux release workflow ships full native (Win/Mac remain stubs)
- `PlanningProfileBenchmarks`, `PlannerComparisonBenchmarks`, `SamplingPlannerRegistryTests`

### Changed

- `RrtConnectPlanner` — obsolete thin wrapper over `SamplingPlanner`
- `RrtConnectOptions` / `OmplPlannerOptions` — alias `SamplingPlannerOptions`
- `RobotMeshCollisionChecker`, `AttachAwareCollisionChecker` — shared mesh BVH cache
- `motus_ompl_ompl.cpp` — planner factory switch with conditional OMPL 2.0 planners (AORRTC, BLIT*)

## [0.6.2] - 2026-07-12

### Added

- `PlanningCollision.ValidateEndpoints` — fast-fail when start or goal is already in collision
- `OmplPlannerOptions.ReportIteration` — optional managed RRT progress callback

### Changed

- `RobotMeshCollisionChecker` — tool geometry checked against scene obstacles; attached-body vs link checks fixed
- `JointLinearPlanner`, `CartesianLinearPathPlanner`, `RrtConnectPlanner` — endpoint validation before planning
- Managed RRT loop reports iteration progress periodically

## [0.6.1] - 2026-07-11

### Added

- **Actuated tool state** — `EndEffectorState`, `ToolCapabilities`, `ToolStateTimeline` annotate trajectory waypoints with gripper parameters
- `MotionProgramValidation.ValidateToolStates` — pre-plan schema check for `SetToolStateSegment` / segment `TargetState`
- `ToolStateCollision.ValidateTrajectory` — per-waypoint collision warnings using width-scaled tool geometry
- `CartesianGoalSolver.TryReachFromStart` — reach Cartesian goals from a known start configuration
- `UrdfUniversalRobotsKinematics` — improved URDF universal-robots IK routing for bundled/tool-offset models
- `PlanningCollisionTests` — LIN validates link envelopes; TCP-on-chord sphere can pass when capsules clear

### Changed

- `ToolDefinition.GeometryForState` — width-scaled gripper collision for actuated tools
- `IndustrialMotionPlanner` — applies tool-state timeline and emits `ToolStateCollision` warnings
- `TrajectoryExport` / `TrajectorySampler` — optional `toolState` per waypoint

## [0.6.0] - 2026-07-10

### Added

- **Session tools** — `ToolDefinition`, `RobotModel.WithTool`, export `SessionToolFrame` when session TCP differs from preset
- `ToolDefinition.FromPreset(RobotModel)` — build tool from bundled preset tool frame + collision
- `HomePoseResolver` + bundled `viewer_presets.json` in `Motus.Presets`
- `TrajectorySampler.AtTime` — joint interpolation along trajectories (shortest-angle joint lerp)
- Public `StlReader.Read` in `Motus.Presets`
- **Xacro (minimal)** — `XacroPreprocessor.Expand`, `UrdfRobotLoader.LoadXacro` (includes, properties, simple macros, `${arg}`; no `$(find)`)
- **Industrial blend execution** — TCP-path truncation at segment corners; exact-stop fallback warning when infeasible
- UR10e preset: Robotiq 2F-85 TCP offset + tool collision box

### Changed

- `IndustrialMotionPlanner` — feasible blend radii truncate segment exits/entries instead of always exact-stop
- `docs/urdf-import.md` — documents in-process xacro tier

## [0.5.1] - 2026-07-10

### Added

- `FastDhFk` — allocation-free DH FK for collision hot paths (link xyz only)
- `PlansMultiGoalSequenceAroundObstacle` — multi-goal RRT regression with timing output

### Changed

- `SphereCollisionChecker` — fast DH/xyz collision path for bundled presets (reused buffers, zero alloc per check)
- `LinkEnvelopeCollision` — xyz-based sphere/box obstacle checks without `Frame` allocations
- `Transforms` — `FromDhInto` / `MultiplyInto` for in-place matrix ops

### Fixed

- Multi-goal obstacle planning perf gate now measures plan time only (`planMs < 500`) for CI stability under parallel load

## [0.5.0] - 2026-07-09

### Added

- Motion-program API for mixed `PTP/LIN/CIRC` (`MotionProgramRequest` + segment types)
- `IndustrialMotionPlanner` for mixed motion programs
- Trajectory export includes motion metadata (`motionType`, `segmentIndex`, `blendRadiusMeters`)
- `Grasshopper01PlaneGoalTests` — UR5e example-01 TCP/LIN regression coverage

### Changed

- `TrajectoryExport` retime default now uses `RetimerAlgorithm.TotgLite`
- `TrajectoryRetimer` supports `RetimerAlgorithm.TotgLite`

### Fixed

- `UrAnalyticInverseKinematics` — Hawkins row-major analytic IK now agrees with `DhForwardKinematics`; fixes IK/LIN from distant seeds (e.g. viewer home → example-01 TCP goal)
- `RrtConnectPlanner` rejects non-positive `StepRadians`
- Collision segment stepping guards against non-positive step values
- `FclCollisionChecker` scene hash now invalidates on obstacle orientation/extent changes

## [0.4.0] - 2026-07-05

### Added

- `PlanningContext`, `AttachedBody`, `PlanningGroup`, `JointIndexMap` — attach at TCP hides scene obstacle; SRDF-style groups
- `Motus.Native` — unified P/Invoke to `motus_native` (OMPL + FCL C ABI); per-RID stubs for Win/Mac/Linux NuGet (managed fallback on desktop)
- `CollisionCheckerFactory`, `AttachAwareCollisionChecker`, `FclCollisionChecker` (FCL when native built; C# mesh fallback)
- `MotusCapabilities.Describe()` — runtime probe for hosts (Grasshopper, CLI)
- Group-aware RRT-Connect — locked joints via `PlanningOptions.GroupMap`; SRDF `LoadGroups` → `PlanningContext.ForGroup`
- Native OMPL: motion validator, `max_plan_time_sec`, `OmplPlannerId` (RRT-Connect, RRT*), native path simplification via `motus_ompl_simplify_path`
- Native FCL: box/sphere/capsule upsert, attach, allowed pairs (`motus_fcl_*`) — Linux CI build; desktop stubs
- JSON preset `toolCollision` field; UR5e/UR10e bundled tool volumes
- URDF tip-link collision → `RobotCollisionModel.ToolGeometry`
- `SrdfLoader.LoadGroups`, `LoadEndEffectors`
- Official `tests/fixtures/ur10e/ur10e.srdf` (MoveIt config, pairs with unmodified `ur10e.urdf`)
- `docs/rhino-host.md` — Rhino 8 Win/Mac deployment guide
- CI matrix: Windows, macOS, Ubuntu; `native-integration` job (Linux OMPL+FCL with `MOTUS_NATIVE_FULL` tests)
- Tests: attach/detach RRT, group lock, SRDF→ForGroup→RRT, native smoke (Linux), URDF tip collision, FK cross-checks, kr210 fixtures
- URDF viewer improvements; `scripts/build-native-stub.sh` / `.ps1`

### Changed

- `RrtConnectPlanner` — group planning space; native validity embeds locked joints
- `CartesianLinearPathPlanner` — uses `CollisionCheckerFactory` with `AttachedBodies`
- `PresetLoader` — resolves bundled robots from plugin/`AppContext.BaseDirectory`
- `RobotMeshCollisionChecker` — tool geometry + attached bodies on mesh path

### Fixed

- `CollisionCheckerFactory` — no double-wrap of `AttachAwareCollisionChecker` on mesh checker
- Native OMPL iteration vs time budget; `setRange` always applied
- URDF mesh loader rejects paths outside the asset directory
- Native runtime tests match stub vs full-native CI profiles (`MOTUS_NATIVE_FULL`)

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

[Unreleased]: https://github.com/lasaths/Motus.NET/compare/v0.7.0...HEAD
[0.7.0]: https://github.com/lasaths/Motus.NET/compare/v0.6.9...v0.7.0
[0.6.9]: https://github.com/lasaths/Motus.NET/compare/v0.6.8...v0.6.9
[0.6.8]: https://github.com/lasaths/Motus.NET/compare/v0.6.7...v0.6.8
[0.6.7]: https://github.com/lasaths/Motus.NET/compare/v0.6.6...v0.6.7
[0.5.0]: https://github.com/lasaths/Motus.NET/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/lasaths/Motus.NET/compare/v0.3.3...v0.4.0
[0.3.3]: https://github.com/lasaths/Motus.NET/compare/v0.3.2...v0.3.3
[0.3.2]: https://github.com/lasaths/Motus.NET/compare/v0.3.1...v0.3.2
[0.3.1]: https://github.com/lasaths/Motus.NET/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/lasaths/Motus.NET/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/lasaths/Motus.NET/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/lasaths/Motus.NET/releases/tag/v0.1.0
