# Roadmap

## Milestone 1 — Motus.NET core

- [x] Motus.Core data model and interfaces
- [x] JSON robot presets (UR + KUKA defaults)
- [x] `JointLinearPlanner`
- [x] Trajectory validation and JSON/CSV export
- [x] Unit tests
- [x] Forward kinematics (DH profiles for all 14 presets)

## Milestone 2 — Motus.NET

- [x] Collision scene primitives (sphere, box) and `SphereCollisionChecker`
- [x] FK/IK via `DhForwardKinematics` + numerical IK (UR/KUKA profiles)
- [x] `CartesianLinearPlanner` — Cartesian goal via IK, joint-linear path

## Milestone 3 — Motus.NET

- [x] `RrtConnectPlanner` in `Motus.OMPL.NET` (pure C#; native OMPL C++ reserved)
- [x] `PathSimplifier` for shortcut smoothing
- [x] Richer `TrajectoryValidator` (collision + acceleration)

## Vision — "MoveIt for Grasshopper, without ROS"

Milestones 1-3 deliver a vendor-neutral planning/validation/export core with clean
`IPlanner` / `ICollisionChecker` / IK seams. The longer-term goal is a dependency-light,
offline planning stack a Grasshopper fabrication user can actually trust on a real cell —
the capabilities MoveIt provides, minus ROS and minus live robot control.

Today the architecture is MoveIt-shaped but shallow (roughly 10-20% of MoveIt's planning
capability, narrowed to a few UR/KUKA arms). The phases below are ordered by impact for
robotic fabrication. Effort sizing: **S** = days, **M** = 1-2 weeks, **L** = several weeks,
**XL** = months. Phases 5, 6, and 4 are the make-or-break trio.

## Milestone 4 — Trust the math (verified kinematics) · S-M

Prerequisite for everything below; without it the planners can't be relied on.

- [x] Ground-truth FK tests: known UR5e/KUKA TCP poses from published DH, not just finiteness
- [x] Validate IK against an independent analytic UR solver (not only `FK(IK(x))` round-trip)
- [x] RRT-Connect invariants over many seeds: in-limits, collision-free, connects, deterministic per seed
- [x] Statistical planning benchmark (success rate, path length) to catch regressions

## Milestone 5 — True Cartesian / industrial motion · L · make-or-break

The current `CartesianLinearPlanner` interpolates in joint space; the TCP does not follow a
straight line. Fabrication toolpaths need true task-space motion.

- [x] `LIN` — straight-line TCP motion with IK along the path and continuity checks (CartesianLinearPathPlanner with SLERP)
- [ ] PTP and CIRC motion types (future)
- [ ] Blends / zone radii between segments (future)
- [x] Toolpath input (sequence of Cartesian targets)

PONYTAIL: LIN planner implemented but IK robustness incomplete - random restarts added but DLS solver limits effectiveness. Needs analytic UR IK or better seed strategy for production use.

## Milestone 6 — Mesh-accurate collision + scene · XL · make-or-break

Sphere envelopes are too coarse to trust a real cell.

- [x] Mesh geometry for CollisionShape (vertices, indices)
- [x] AABB collision (MeshCollisionChecker - broad phase only)
- [ ] Mesh-accurate collision (managed BVH/GJK or an acceptable native binding) - per-triangle pending
- [ ] Continuous (swept) collision instead of discrete segment sampling
- [ ] Attached objects (grasped tool/part) and an allowed-collision matrix

PONYTAIL: Mesh skeleton in place with AABB broad phase. Upgrade to BVH + per-triangle GJK for full M6. Extension points marked.

## Milestone 7 — Arbitrary robot import · L

Move beyond the baked-in preset list toward a platform.

- [ ] URDF/SRDF (or mesh + DH) import for arbitrary arms and tools
- [ ] Generalized FK/IK for imported chains (numerical IK already exists as a fallback)

## Milestone 8 — Time-optimal trajectory parameterization · M

- [ ] Retiming that respects velocity, acceleration, and jerk limits (TOTG-style)
- [ ] Replace the current step + velocity-cap timing

## Milestone 9 — Constraints & optimizing planners · L-XL

- [ ] Orientation/path constraints (e.g. keep tool normal to surface) and goal regions
- [ ] Optimizing planner (PRM* / CHOMP- or STOMP-style smoothing beyond `PathSimplifier`)

## Related repos

Rhino / Grasshopper UI (`Motus.Grasshopper`, `Motus.Rhino`) lives in a separate repository and consumes this core via project references.

## Future optional

- Native OMPL C++ binding (swap behind `IPlanner`)
- Yak package distribution

## Out of scope (v1)

- Physical robot control
- Dependencies on UR.RTDE.Grasshopper, VirtualRobot, Robots, KUKA|prc, ROS, MoveIt, Tesseract
