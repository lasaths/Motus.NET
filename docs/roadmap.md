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

First public Yak Package Manager release is **2.0.0** (identity + pack path ready; Package Manager push still open).

## Milestone 11 — Trust & polish · 1.8.0

Prove 0.17-class behavior is shippable. Few new features — polish and hardening.

- [ ] Drive Motus.Grasshopper regression matrix green on real Rhino (serial, Stewart, SE2, Tool, Walk/Plan legged, Example 10). See sibling Motus.Grasshopper `docs/regression-matrix.md`
- [x] Expand Motus.NET automated coverage for matrix rows that cannot run Rhino GUI in CI (`RegressionMatrixLogicTests`)
- [ ] Fix defects found in that Rhino pass
- [x] Package identity / CHANGELOG prepared for **1.8.0**; docs reflect SemVer jump after shipped **0.17.0**
- [x] Coordinated Motus.NET + Motus.Grasshopper **1.8.0** release (NuGet + GitHub tags [v1.8.0](https://github.com/lasaths/Motus.NET/releases/tag/v1.8.0)); Yak not required

## Milestone 12 — Host product · 1.9.0

- [x] Pick Place empty-`TouchBodies` fail-closed (`PickPlaceCycle`); GH empty **Touch** gate landed in Motus.Grasshopper
- [ ] Remaining GH UX from 1.8 QA: Program Tr-keep / experimental Robot Status / family handoff (Motus.Grasshopper follow-up; not blocking 1.9.0 / 2.0.0 SemVer cuts)
- [x] Motus.Grasshopper NuGet-default integration path (`UseMotusNetProjectReference` defaults false; pin via `MotusNetPackages.props`). Remaining “all hosts” = other consumers / docs only — see Future optional.
- [x] Small additive API: empty-`TouchBodies` throw before stability promise
- [x] Coordinated Motus.NET + Motus.Grasshopper **1.9.0** release (NuGet + GitHub tags [v1.9.0](https://github.com/lasaths/Motus.NET/releases/tag/v1.9.0)); Yak still unpublished (first public Yak **2.0.0**)
- [ ] Still no live control / ROS / vendor SDKs

## Milestone 13 — Public product · 2.0.0

- [x] Align Motus.NET + Motus.Grasshopper package / tag identity to **2.0.0** ([v2.0.0](https://github.com/lasaths/Motus.NET/releases/tag/v2.0.0); sibling GH [v2.0.0](https://github.com/lasaths/Motus.Grasshopper/releases/tag/v2.0.0)); yak `motus` manifest identity **2.0.0** in-repo
- [x] Motus.NET NuGet **2.0.0** flat-container live + Grasshopper NuGet-default restore proven (all six packages)
- [ ] First public Yak Package Manager push (`motus` **2.0.0**)
- [x] Declare supported vs advanced surface — [supported-surface.md](supported-surface.md)
- [x] SemVer major cut = supported-product identity (breaking changes only if banked from 1.8/1.9); Yak GA still gated on Package Manager push + Rhino matrix
- [x] Dual-TFM pack / yak build path + docs for **2.0.0** identity (pack scripts ready; production yak push open)
- [ ] Rhino regression matrix green on host (serial, Stewart, SE2, Tool, Walk/Plan, Example 10) — soft until local Rhino pass

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

## Experimental mobility (tip-shipped; outside Yak GA)

**Shipped on Motus.NET tip / NuGet 2.0.0 (experimental bar):** HolonomicSE3 + free-flyer hull collision + aerial export honesty (`Family=aerial`, RPY fixed-axis XYZ / URDF), Unitree H2 / Go2 meshless fixtures + FK / stance smoke. **Not** Motus.Grasshopper GA components — GH Motus Robot optional URDF load stays experimental Remark only; Waypoints/Export warn aerial ≠ MoveJ. Stays outside the 2.0 Yak GA / Supported surface unless product scope explicitly expands. Physical control, ROS, and vendor SDKs remain out.

Honest Motus meanings (planning / preview / export only):

| Track | Verdict | In Motus (tip) | Not Motus |
|-------|---------|----------------|-----------|
| **Drone (full)** | Conditional — offline SE(3) | Holonomic SE(3) body planning, hull collision, export `bodyPose`; recommended `Family=aerial` | ArduPilot/PX4, battery, MAVLink, live SITL |
| **Humanoid** | Partial experimental | Fixed-base / tree arm groups on **Unitree H2** meshless fixture; no balance claim | Walking balance, loco-manipulation GA |
| **Quadruped** | Partial experimental | Go2 meshless FK / stance smoke; distinct from insectoid `Family=legged` Walk | Mammal foot-IK product, Spot-class loco GA |

Phasing vs Supported / Yak GA (does **not** shrink the 2.0 product goal):

- **Tip-shipped experimental (2.0.0 tree):** SE(3) aerial Motus-honest path; H2 upper-body / arm-group fixture; Go2 stance FK fixture — see [supported-surface.md](supported-surface.md) Experimental
- **Still open for M13 completion:** first Yak Package Manager push; Rhino matrix host pass
- **Post-Yak deepen (optional):** richer aerial fixtures (Crazyflie / Skydio), mammal foot IK, kinematic biped preview — still not balance / flight stack

Fixtures ([awesome-robot-descriptions](https://github.com/robot-descriptions/awesome-robot-descriptions) / Unitree):

1. **Unitree H2** (`tests/fixtures/unitree_h2/`) — **landed** experimental humanoid fixture ([unitree_ros/robots/h2_description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/h2_description))
2. **Unitree Go2** (`tests/fixtures/unitree_go2/`) — **landed** experimental mammal quad FK / stance smoke
3. Free-flyer box (`tests/fixtures/aerial/`) — **landed** SE(3) hull-collision fixture
4. Optional later: Crazyflie 2.0 / Skydio X2 visuals; G1 only if H2 path regresses

Do not promote experimental tracks to Supported / Yak headline in release notes without ADR + METHODS + Rhino matrix proof.
