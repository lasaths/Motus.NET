# Architecture

## Overview

Motus.NET is a standalone planning core consumed by Motus.Grasshopper and future hosts. All vendor-specific and UI concerns stay outside Motus.Core.

```
Motus.Grasshopper (Rhino / GH UI — separate repo)
    ↓ project references / NuGet
Motus.Presets (JSON + URDF/SRDF loaders)
    ↓
Motus.Core (data model, PlanningContext, IPlanner, validation, export)
Motus.Geometry (FK/IK, mesh collision, attach-aware checkers, Cartesian planners)
    ↓
Motus.OMPL.NET (RRT-Connect / RRT*; managed fallback)
Motus.Native (P/Invoke → motus_native: OMPL + FCL)
```

See [rhino-host.md](rhino-host.md) for Windows/macOS deployment (managed-first).

## Motus.Core principles

- **No UI dependencies** — pure .NET 8 class library
- **Planners behind `IPlanner`** — joint-linear, Cartesian-linear, LIN, and RRT-Connect
- **Presets are data** — JSON files loaded by Motus.Presets, not hardcoded switches
- **PlanningContext** — robot + scene + attach/detach; optional `PlanningGroup` for reduced-DOF planning

## Key interfaces

| Interface | Role |
|-----------|------|
| `IPlanner` | Produce `PlanningResult` from `PlanningRequest` |
| `ITrajectoryValidator` | Check trajectory against limits and timing |
| `IFkSolver` / `IIkSolver` | FK and IK (`Motus.Geometry`) |
| `ICollisionChecker` | `SphereCollisionChecker`, `RobotMeshCollisionChecker`, `AttachAwareCollisionChecker`, `FclCollisionChecker` |
| `PlanningContext` | Attach geometry at TCP; hide scene obstacles on pick |

## Units

Internal units: **radians**, **seconds**, **meters**. Conversion helpers live in `Units`.

## Native (`motus_native`)

`Motus.Native` loads per-RID stubs from NuGet (`runtimes/{rid}/native/`). Stubs report OMPL/FCL unavailable → managed RRT and C# mesh collision run (default on Rhino Win/Mac). Full native OMPL+FCL optional on Linux CI. See [ompl-port-plan.md](ompl-port-plan.md).
