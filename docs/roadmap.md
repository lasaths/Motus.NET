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

## Milestone 10 — Reliable pick/place · 0.17.0 (shipped)

- [x] Detach at placement before retract; explicit segment-local gripper contact pairs
- [x] Multi-object scene regression, including already placed objects
- [x] Opt-in RRT-Connect transfers via injected planner; LIN approach/retract
- [x] Attachment timeline on Trajectory, preserved by all retimers and JSON/CSV export
- [x] SET/WAIT dwell preservation and single-pass export retiming
- [x] Version metadata aligned across core and Grasshopper
- [x] NuGet + GitHub release [v0.17.0](https://github.com/lasaths/Motus.NET/releases/tag/v0.17.0)

## Path to 2.0 — Versioning

Public package SemVer is **aligned** across Motus.NET (NuGet), Motus.Grasshopper, and yak `motus`.

After **0.17.0**, the next public package versions are **1.8.0 → 1.9.0 → 2.0.0**. There is no empty 1.0–1.7 NuGet/Yak upload series; those numbers are not published package lines. Milestone names match the coordinated release tags.

First public Yak Package Manager release is **2.0.0**.

## Milestone 11 — Trust & polish · 1.8.0

Prove 0.17-class behavior is shippable. Few new features — polish and hardening.

- [ ] Drive Motus.Grasshopper regression matrix green on real Rhino (serial, Stewart, SE2, Tool, Walk/Plan legged, Example 10). See sibling Motus.Grasshopper `docs/regression-matrix.md`
- [x] Expand Motus.NET automated coverage for matrix rows that cannot run Rhino GUI in CI (`RegressionMatrixLogicTests`)
- [ ] Fix defects found in that Rhino pass
- [x] Package identity / CHANGELOG prepared for **1.8.0** (this cut); docs reflect SemVer jump after shipped **0.17.0**
- [ ] Coordinated Motus.NET + Motus.Grasshopper **1.8.0** release (NuGet + GitHub tags); Yak not required yet — publish on tag after this metadata lands

## Milestone 12 — Host product · 1.9.0

- [ ] Grasshopper UX / Program / Pick Place / Preview edge cases from 1.8 QA (Touch fail-closed landed; Program Tr-keep / experimental Robot / family handoff in Motus.Grasshopper follow-up)
- [x] Motus.Grasshopper NuGet-default integration path (`UseMotusNetProjectReference` defaults false; pin via `MotusNetPackages.props`). Remaining “all hosts” = other consumers / docs only — see Future optional.
- [ ] Small additive APIs only if needed before the stability promise
- [ ] Coordinated **1.9.0** NuGet + GitHub tags; still no requirement to publish Yak
- [ ] Still no live control / ROS / vendor SDKs

## Milestone 13 — Public product · 2.0.0

- [ ] Align Motus.NET + Motus.Grasshopper (+ yak `motus`) to **2.0.0**
- [ ] First public Yak Package Manager push
- [ ] Declare supported vs advanced surface (e.g. UR pick/place + LIN/RRT supported; stewart/legged advanced unless proven in 1.8)
- [ ] SemVer major = supported product / Yak GA; breaking changes only if banked from 1.8/1.9
- [ ] Release checklist: dual-TFM pack, yak build/push, docs

## Related repos

Rhino / Grasshopper UI lives in **Motus.Grasshopper** and consumes this core via versioned DLLs or NuGet.

## Future optional

- [x] Native OMPL C++ in CI when OMPL is available (`MOTUS_USE_OMPL=ON`)
- [ ] NuGet as default for remaining non-GH hosts *(Motus.Grasshopper done — Milestone 12)*

## Out of scope through 2.0

Unless scope expands, these remain out through the **2.0.0** public product:

- Physical robot control
- ROS / MoveIt runtime dependency
- Vendor SDKs

## Experimental mobility (post-1.8 / parallel track)

**Shipped on Motus.NET tip (1.8.0+ tree; NuGet publish may lag):** HolonomicSE3 + free-flyer hull collision + aerial export honesty (`Family=aerial`), Unitree H2 / Go2 meshless fixtures + FK smoke. **Not** Motus.Grasshopper GA components — GH Motus Robot optional URDF load stays experimental Remark only. Stays outside the 2.0 Yak GA surface unless product scope explicitly expands. Physical control, ROS, and vendor SDKs remain out.

Honest Motus meanings (planning / preview / export only):

| Track | Verdict | In Motus | Not Motus |
|-------|---------|----------|-----------|
| **Drone (full)** | Conditional — offline SE(3) | Holonomic SE(3) body planning, collision, export; recommended `Family=aerial` | ArduPilot/PX4, battery, MAVLink, live SITL |
| **Humanoid** | Partial experimental | Fixed-base / tree arm groups on **Unitree H2** URDF first; optional kinematic biped without balance | Walking balance, loco-manipulation GA |
| **Quadruped** | Partial experimental | Mammal stance/IK on Go2/ANYmal-class URDF; distinct from insectoid `Family=legged` Walk | Spot-class loco product equivalence |

Suggested phasing (does **not** change 1.8 → 1.9 → 2.0 supported promises):

- **1.9 (optional):** Family string helpers / experimental flags + skipped tests only — no GH GA components
- **Post-2.0 parallel track:** SE(3) aerial Motus-full; humanoid upper-body (H2); quadruped stance/IK fixtures

Candidate fixtures ([awesome-robot-descriptions](https://github.com/robot-descriptions/awesome-robot-descriptions) / Unitree):

1. **Crazyflie 2.0** (`cf2_description`) — aerial / SE(3) body proxy
2. **Unitree H2** (`h2_description`) — **first humanoid fixture** ([unitree_ros/robots/h2_description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/h2_description)); prefer over G1
3. **Unitree Go2** (`go2_description`) — mammal quadruped stance/IK
4. Optional visual: Skydio X2; G1 only if H2 meshless strip fails CI

Do not claim shipped support in release notes until ADR + METHODS + fixtures land.
