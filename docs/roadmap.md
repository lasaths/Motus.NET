# Roadmap

## Milestone 1 — Motus.NET core

- [x] Motus.Core data model and interfaces
- [x] JSON robot presets (UR + KUKA defaults)
- [x] `JointLinearPlanner`
- [x] Trajectory validation and JSON/CSV export
- [x] Unit tests
- [x] Forward kinematics (DH profiles for all 14 presets)

## Milestone 2 — Motus.NET

- [x] Collision scene primitives (sphere, box, mesh) and checkers
- [x] FK/IK via DH profiles + numerical IK; UR analytic IK for Universal Robots presets
- [x] `CartesianLinearPlanner` and `CartesianLinearPathPlanner` (true LIN)

## Milestone 3 — Motus.NET

- [x] `RrtConnectPlanner` in `Motus.OMPL.NET` (managed; native OMPL when built)
- [x] `PathSimplifier` for shortcut smoothing
- [x] Richer `TrajectoryValidator` (collision segments + acceleration)

## Vision — dependency-light offline planning core

Host-agnostic library (Grasshopper, CLI, services). MoveIt-shaped seams without ROS.

## Milestone 4 — Trust the math · S-M

- [x] Ground-truth FK tests for UR5e/KUKA
- [x] IK round-trip and benchmark tests
- [x] RRT-Connect determinism per seed
- [x] UR analytic IK with numerical fallback

## Milestone 5 — True Cartesian / industrial motion · L

- [x] `LIN` — straight-line TCP with SLERP orientation
- [x] Toolpath through waypoint chains
- [ ] PTP / CIRC / blend radii (future)

## Milestone 6 — Mesh-accurate collision · L-XL

- [x] Mesh obstacles + BVH narrow phase (sphere-link vs mesh SAT)
- [x] Capsule sampling between link origins; SRDF-lite allowed pairs
- [x] Per-link robot meshes (JSON `collisionLinks` + URDF `<collision>`)
- [x] Continuous swept collision (adaptive segment sampling)
- [x] Attached objects (`PlanningContext`, `AttachedBody`, C# attach at TCP)
- [x] Tool collision via `RobotCollisionModel.ToolGeometry` (URDF tip link)

## Milestone 7 — Arbitrary robot import · L

- [x] URDF revolute + prismatic serial chains
- [x] URDF collision geometry + `ur5e_collision.urdf` fixture; `docs/urdf-import.md`
- [ ] xacro preprocessing, tool links, public `ur_description` fixture
- [x] SRDF-lite `disable_collisions` import
- [x] SRDF `group` / `end_effector` metadata (`SrdfLoader.LoadGroups`)

## Milestone 8 — Trajectory parameterization · M

- [x] Trapezoidal retiming with jerk-aware spacing
- [x] `TrajectoryExport.Export` with retime + validate
- [x] Bottleneck path retiming (TOTG-lite default for export)
- [ ] True time-optimal (TOTG) parameterization

## Milestone 9 — Constraints & optimizing planners · L-XL

- [ ] Orientation/path constraints
- [ ] PRM* / CHOMP-style optimization

## Related repos

Rhino / Grasshopper UI lives in **Motus.Grasshopper** and consumes this core via versioned DLLs or NuGet.

## Future optional

- [x] Native OMPL C++ in CI when OMPL is available (`MOTUS_USE_OMPL=ON`)
- NuGet as default integration path for all hosts

## Out of scope (v1)

- Physical robot control
- ROS / MoveIt runtime dependency
- Vendor SDKs
