# Motus.NET

Standalone .NET robotics motion-planning core for the Motus toolkit.

Licensed under [MIT](LICENSE).

## Scope

- Neutral robot model, joint states, trajectories, and planning interfaces
- JSON robot presets (UR and KUKA defaults) with DH kinematics profiles
- `JointLinearPlanner` — deterministic joint-space interpolation
- `CartesianLinearPlanner` — Cartesian goal via IK, joint-linear path
- `RrtConnectPlanner` — collision-aware joint-space RRT-Connect (`Motus.OMPL.NET`)
- Forward/inverse kinematics, sphere-envelope collision checking (`Motus.Geometry`)
- Trajectory validation (limits, velocity, acceleration, optional collision) and JSON/CSV export
- No Rhino, Grasshopper, UR RTDE, VirtualRobot, ROS, MoveIt, Tesseract, or OMPL runtime dependency

## Units

| Quantity | Unit |
|----------|------|
| Joint angles | radians |
| Time | seconds |
| Length / position | **meters** |

Use `Motus.Core.Units` for degree/radian conversion.

## Build & test

```bash
dotnet build Motus.NET.slnx
dotnet test Motus.NET.slnx
```

## Projects

| Project | Purpose |
|---------|---------|
| Motus.Core | Planning data model, planners interface, validation, export |
| Motus.Presets | JSON preset loader |
| Motus.Geometry | DH FK/IK, collision checker, Cartesian planner |
| Motus.OMPL.NET | RRT-Connect planner, path simplification |
| Motus.OMPL.Native | Reserved for future optional OMPL C++ ABI |

## Planners

| Planner | Use when |
|---------|----------|
| `JointLinearPlanner` | Known start/goal joint configs, no obstacles |
| `CartesianLinearPlanner` | Cartesian TCP goal; path is joint-linear after IK |
| `RrtConnectPlanner` | Obstacles in `PlanningOptions.CollisionScene` |

## Safety

Motus is planning, validation, and export only. It does **not** send commands to physical robots. Presets are visualization/planning defaults — verify all limits and calibration before real hardware use.

See [docs/safety.md](docs/safety.md).
