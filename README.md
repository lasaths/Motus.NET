# Motus.NET

Standalone .NET robotics motion-planning core for the Motus toolkit.

## Scope

- Neutral robot model, joint states, trajectories, and planning interfaces
- JSON robot presets (UR and KUKA defaults)
- `JointLinearPlanner` — deterministic joint-space interpolation
- Trajectory validation and JSON/CSV export
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
| Motus.Core | Planning data model, `JointLinearPlanner`, validation, export |
| Motus.Presets | JSON preset loader |
| Motus.Geometry | Placeholder for future geometry helpers |
| Motus.OMPL.Native / Motus.OMPL.NET | Placeholders for future custom OMPL binding |

## Safety

Motus is planning, validation, and export only. It does **not** send commands to physical robots. Presets are visualization/planning defaults — verify all limits and calibration before real hardware use.

See [docs/safety.md](docs/safety.md).
