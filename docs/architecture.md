# Architecture

## Overview

Motus.NET is a standalone planning core consumed by Motus.Grasshopper and future hosts. All vendor-specific and UI concerns stay outside Motus.Core.

- `Motus.Presets` loads JSON/URDF/SRDF data into `RobotModel`.
- `Motus.Core` defines planning contracts, motion segments, validation, and export.
- `Motus.Geometry` and `Motus.OMPL.NET` implement planners/collision (managed + optional native through `Motus.Native`).

See [rhino-host.md](rhino-host.md) for Windows/macOS deployment (managed-first).

## Motus.Core principles

- **No UI dependencies** — pure .NET 8 class library
- **Planners behind `IPlanner`** — joint-linear, Cartesian-linear, LIN, and RRT-Connect
- **Motion-program contracts** — `MotionProgramRequest` with `PTP/LIN/CIRC` segments
- **Presets are data** — JSON files loaded by Motus.Presets, not hardcoded switches
- **PlanningContext** — robot + scene + attach/detach; optional `PlanningGroup` for reduced-DOF planning

## Motion Program Layer (0.6.0)

`IndustrialMotionPlanner` executes mixed `PTP/LIN/CIRC` segments with TCP blend truncation at corners when feasible. Session tools via `ToolDefinition` / `RobotModel.WithTool`. See `README.md` for usage details.

## Key interfaces

| Interface | Role |
|-----------|------|
| `IPlanner` | Produce `PlanningResult` from `PlanningRequest` |
| `ITrajectoryValidator` | Check trajectory against limits and timing |
| `IFkSolver` / `IIkSolver` | FK and IK (`Motus.Geometry`) |
| `ICollisionChecker` | `SphereCollisionChecker`, `RobotMeshCollisionChecker`, `AttachAwareCollisionChecker`, `FclCollisionChecker` |
| `PlanningContext` | Attach geometry at TCP; hide scene obstacles on pick |
| `MotionSegment` | Program segment contracts (`PtpSegment`, `LinSegment`, `CircSegment`) |

## Units

Internal units: **radians** (revolute joints), **meters** (prismatic joints / distance), **seconds** (time). Conversion helpers live in `Units`. `JointLimit` carries `JointCoordinateUnit` (`Radians` or `Meters`); legacy `MinRadians`/`MaxRadians` aliases return the same numeric bounds.

Stewart/Gough platforms (`RobotPreset.Family = "stewart"`) use six prismatic leg lengths in **meters**. See Motus.Grasshopper ADR 0003.

## Engineering bar

Strive for NASA-grade kinematics contracts: explicit units, structured IK/FK failure reasons (no silent NaN), deterministic solvers with documented tolerances, verified FK↔IK round-trips, and docs/ADR before shipping a new mechanism family.

## Native (`motus_native`)

`Motus.Native` loads per-RID stubs from NuGet (`runtimes/{rid}/native/`). Stubs report OMPL/FCL unavailable → managed RRT and C# mesh collision run (default on Rhino Win/Mac). Full native OMPL+FCL optional on Linux CI. See [ompl-port-plan.md](ompl-port-plan.md).
