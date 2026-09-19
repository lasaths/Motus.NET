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
- [x] Industrial blend radii (TCP truncation + exact-stop fallback)
- [x] `CIRC` — circular arc segments

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
- [x] xacro preprocessing, tool links, public `ur_description` fixture
- [x] SRDF-lite `disable_collisions` import
- [x] SRDF `group` / `end_effector` metadata (`SrdfLoader.LoadGroups`)

## Milestone 8 — Trajectory parameterization · M

- [x] Trapezoidal retiming with jerk-aware spacing
- [x] `TrajectoryExport.Export` with retime + validate
- [x] Bottleneck path retiming (TOTG-lite default for export)
- [x] True time-optimal (TOTG) parameterization

## Milestone 9 — Constraints & optimizing planners · L-XL

- [x] Orientation/path constraints
- [x] PRM* / CHOMP-style optimization

## ADR waves — parallel, tree, mobile, legged

- [x] Stewart/Gough (`Family=stewart`) remains a parallel sibling stack with meters-valued leg lengths, collision validation, and managed stroke-space sampling.
- [x] Full-tree / group planning uses `PlanningGroup` + `JointIndexMap` over full driver names; tip-only FK/IK and branch/finger group planning are documented as distinct contracts.
- [x] Holonomic SE(2) managed sampling appends x/y/yaw with explicit local bounds and per-sample base-frame collision hooks.
- [x] Legged full-driver Plan adapter (`ValidateForPlan`) over `TryBuild` output: SSM/collision validation only.
- [x] Legged Motus Plan synthesis (`LeggedGait.PlanBodyPath`): body-path polyline → `TryBuild` + hard SSM `ValidateForPlan` (Walk remains rich UI).

## Milestone 10 — Reliable pick/place · 0.17.0 (ready to cut; not on nuget.org yet)

- [x] Detach at placement before retract; explicit segment-local gripper contact pairs
- [x] Multi-object scene regression, including already placed objects
- [x] Opt-in RRT-Connect transfers via injected planner; LIN approach/retract
- [x] Attachment timeline on Trajectory, preserved by all retimers and JSON/CSV export
- [x] SET/WAIT dwell preservation and single-pass export retiming
- [x] Version metadata aligned across core and Grasshopper

## Related repos

Rhino / Grasshopper UI lives in **Motus.Grasshopper** and consumes this core via versioned DLLs or NuGet.

## Future optional

- [x] Native OMPL C++ in CI when OMPL is available (`MOTUS_USE_OMPL=ON`)
- NuGet as default integration path for all hosts

## Out of scope (v1)

- Physical robot control
- ROS / MoveIt runtime dependency
- Vendor SDKs

## Experimental mobility (post-1.8 / parallel track)

**Not shipped.** Docs-only intent for aerial / humanoid / mammal-quadruped work that stays outside the 2.0 Yak GA surface unless product scope explicitly expands. Physical control, ROS, and vendor SDKs remain out (see above / Path to 2.0 when present).

Honest Motus meanings (planning / preview / export only):

| Track | Verdict | In Motus | Not Motus |
|-------|---------|----------|-----------|
| **Drone (full)** | Conditional — offline SE(3) | Holonomic SE(3) body planning, collision, export; recommended `Family=aerial` | ArduPilot/PX4, battery, MAVLink, live SITL |
| **Humanoid** | Partial experimental | Fixed-base / tree arm groups on a humanoid URDF; optional kinematic biped without balance | Walking balance, loco-manipulation GA |
| **Quadruped** | Partial experimental | Mammal stance/IK on Go2/ANYmal-class URDF; distinct from insectoid `Family=legged` Walk | Spot-class loco product equivalence |

Suggested phasing (does **not** change 1.8 → 1.9 → 2.0 supported promises):

- **1.9 (optional):** Family string helpers / experimental flags + skipped tests only — no GH GA components
- **Post-2.0 parallel track:** SE(3) aerial Motus-full; humanoid upper-body; quadruped stance/IK fixtures

Candidate fixtures ([awesome-robot-descriptions](https://github.com/robot-descriptions/awesome-robot-descriptions)): Crazyflie 2.0 (`cf2_description`), Unitree Go2 (`go2_description`), Unitree G1 or Skydio X2.

Do not claim shipped support in release notes until ADR + METHODS + fixtures land.
