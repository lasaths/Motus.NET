# Roadmap

## Milestone 1 — Motus.NET core

- [x] Motus.Core data model and interfaces
- [x] JSON robot presets (UR + KUKA defaults)
- [x] `JointLinearPlanner`
- [x] Trajectory validation and JSON/CSV export
- [x] Unit tests
- [x] Forward kinematics (DH profiles for all 14 presets)
- [ ] Example `.gh` files saved from Rhino

## Milestone 2 — Motus.NET

- [x] Collision scene primitives (sphere, box) and `SphereCollisionChecker`
- [x] FK/IK via `DhForwardKinematics` + numerical IK (UR/KUKA profiles)
- [x] `CartesianLinearPlanner` — Cartesian goal via IK, joint-linear path

## Milestone 3 — Motus.NET

- [x] `RrtConnectPlanner` in `Motus.OMPL.NET` (pure C#; native OMPL C++ reserved)
- [x] `PathSimplifier` for shortcut smoothing
- [x] Richer `TrajectoryValidator` (collision + acceleration)

## Milestone 4 — Motus.Grasshopper / Motus.Rhino

- [x] Improved Rhino preview (FK meshes, tool frame, invalid segments)
- [x] Grasshopper cancellation for long plans (RRT via `OnPingDocument`)
- [x] Optional continuous re-plan toggle (`AutoReplan`)
- [x] Motus.Grasshopper components: preset, plan, validate, preview, export, collision

## Future optional

- Native OMPL C++ binding in `Motus.OMPL.Native` (swap behind `IPlanner`)
- Yak package distribution

## Out of scope (v1)

- Physical robot control
- Dependencies on UR.RTDE.Grasshopper, VirtualRobot, Robots, KUKA|prc, ROS, MoveIt, Tesseract
