# OMPL Port Plan

Long-term Motus will ship a **custom** OMPL .NET binding — not `Ompl.NetStandard.x64`.

## Layering

```
OMPL C++ library
    ↓
Motus.OMPL.Native   (C ABI wrapper, no C++ types exported)
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

`Motus.OMPL.NET` ships a **pure C# RRT-Connect** planner behind `IPlanner`, with `PathSimplifier` for shortcut smoothing. `Motus.OMPL.Native` remains reserved for a future optional OMPL C++ ABI swap — no native build required today.
