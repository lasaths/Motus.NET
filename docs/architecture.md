# Architecture

## Overview

Motus.NET is a standalone planning core consumed by Motus.Grasshopper and future hosts. All vendor-specific and UI concerns stay outside Motus.Core.

```
Motus.Grasshopper (Rhino / GH UI)
    ↓ project references
Motus.Rhino (Frame ↔ Plane, stick preview)
Motus.Presets (JSON loader)
    ↓
Motus.Core (data model, IPlanner, validation, export)
Motus.Geometry (FK/IK, collision, CartesianLinearPlanner)
    ↓
Motus.OMPL.NET (RRT-Connect, path simplification)
```

## Motus.Core principles

- **No UI dependencies** — pure .NET 8 class library
- **Planners behind `IPlanner`** — joint-linear, Cartesian-linear, and RRT-Connect implementations
- **Presets are data** — JSON files loaded by Motus.Presets, not hardcoded switches
- **Deterministic first planner** — joint-space linear interpolation with explicit step and timing options

## Key interfaces

| Interface | Role |
|-----------|------|
| `IPlanner` | Produce `PlanningResult` from `PlanningRequest` |
| `ITrajectoryValidator` | Check trajectory against limits and timing |
| `IForwardKinematics` / `IInverseKinematics` | DH FK and numerical IK (`Motus.Geometry`) |
| `ICollisionChecker` | Sphere-envelope checking (`SphereCollisionChecker`) |

## Units

Internal units: **radians**, **seconds**, **meters**. Conversion helpers live in `Units`.

## OMPL

`Motus.OMPL.NET` is the managed adapter over `Motus.OMPL.Native` (C ABI → OMPL C++). A managed RRT-Connect fallback runs when the native library is not built. See [ompl-port-plan.md](ompl-port-plan.md).
