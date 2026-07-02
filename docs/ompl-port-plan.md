# OMPL Port Plan

Today `Motus.OMPL.NET` is a pure C# planner. If a native OMPL binding is ever added, Motus would ship a **custom** OMPL .NET binding — not `Ompl.NetStandard.x64`.

## Possible future native layering

```
OMPL C++ library
    ↓
native C ABI wrapper (C ABI, no C++ types exported)
    ↓
Motus.OMPL.NET      (C# P/Invoke)
    ↓
Motus.Core          (IPlanner adapter, internal only)
```

## Initial OMPL subset

Expose only what Motus needs first:

- `RealVectorStateSpace` with joint bounds
- State validity callback / native validity interface
- Start and goal states
- `RRTConnect` planner
- `PathGeometric` extraction
- Planner status
- Path simplification

## Rules

- Do not rewrite OMPL in C#
- Do not expose C++ classes to C#
- OMPL is **optional** — Motus.Core and Grasshopper workflow must work without it
- Grasshopper users keep the same components; planner selection stays internal or becomes an explicit option later

## Current status

`Motus.OMPL.NET` is the C# adapter layer. `Motus.OMPL.Native` exposes a C ABI (`native/include/motus_ompl.h`); the default build is a stub until OMPL C++ is linked with `MOTUS_USE_OMPL=ON`. A managed RRT-Connect fallback remains available for development and CI without native OMPL.
