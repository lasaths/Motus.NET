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
    ↓ future optional
Motus.OMPL.Native → OMPL C++
```

## Motus.Core principles

- **No UI dependencies** — pure .NET 8 class library
- **Planners behind `IPlanner`** — `JointLinearPlanner` today; OMPL adapter later without changing GH workflow
- **Presets are data** — JSON files loaded by Motus.Presets, not hardcoded switches
- **Deterministic first planner** — joint-space linear interpolation with explicit step and timing options

## Key interfaces

| Interface | Role |
|-----------|------|
| `IPlanner` | Produce `PlanningResult` from `PlanningRequest` |
| `ITrajectoryValidator` | Check trajectory against limits and timing |
| `IForwardKinematics` / `IInverseKinematics` | Reserved for future FK/IK backends |
| `ICollisionChecker` | Reserved; first planner skips collision |

## Units

Internal units: **radians**, **seconds**, **meters**. Conversion helpers live in `Units`.

## OMPL (future)

OMPL remains optional. Motus.Core must compile and plan without it. See [ompl-port-plan.md](ompl-port-plan.md).
