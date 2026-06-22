# Roadmap

## Milestone 1 (current)

- [x] Motus.Core data model and interfaces
- [x] JSON robot presets (UR + KUKA defaults)
- [x] `JointLinearPlanner`
- [x] Trajectory validation and JSON/CSV export
- [x] Unit tests
- [x] Motus.Grasshopper components: preset, plan, validate, preview, export
- [ ] Forward kinematics for accurate TCP preview
- [ ] Example `.gh` files saved from Rhino

## Milestone 2

- Collision scene representation and `ICollisionChecker` implementation
- FK/IK interfaces with UR/KUKA analytic or numerical backends
- Cartesian pose goals (linear in joint space fallback)

## Milestone 3

- Custom OMPL binding (`Motus.OMPL.Native` + `Motus.OMPL.NET`)
- `RRTConnect` planner behind `IPlanner`
- Path simplification and richer validation

## Milestone 4

- Improved Rhino preview (meshes, tool frame, invalid segments)
- Grasshopper cancellation for long plans
- Optional continuous re-plan toggle

## Out of scope (v1)

- Physical robot control
- Dependencies on UR.RTDE.Grasshopper, VirtualRobot, Robots, KUKA|prc, ROS, MoveIt, Tesseract
