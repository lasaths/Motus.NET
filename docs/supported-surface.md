# Supported vs advanced surface (toward Motus 2.0)

Product tiers for Motus.NET + Motus.Grasshopper. SemVer **2.0.0** = first public Yak Package Manager release and the stability promise for the **Supported** tier. This document does **not** bump package versions.

Aligned SemVer staircase: **0.17.0 → 1.8.0 → 1.9.0 → 2.0.0** (no empty 1.0–1.7 NuGet/Yak line). See [roadmap.md](roadmap.md) Path to 2.0 and the store release plan.

## Supported (Yak GA at 2.0.0)

Offline **plan / preview / export** only — no live robot I/O.

| Surface | Contract |
|---------|----------|
| Serial UR (+ Robotiq) | Tip-path Plan: LIN, joint-linear, managed RRT-Connect; Tool Cap / Rd / Bd |
| Pick / place | `PickPlaceCycle` + GH Motus Pick Place → Program; non-empty `TouchBodies` / Touch (fail-closed) |
| Waypoints handoff | `Q` → MoveJ **only** for serial `Family` (radians) |
| Export | JSON/CSV with family-honest columns; TotgLite default retimer |

Rhino host checklist: Motus.Grasshopper `docs/regression-matrix.md` (Supported rows). Logic coverage: `RegressionMatrixLogicTests`.

## Advanced (shipped; not Yak headline)

Available in libraries / plugin with honest units and Status — not the Package Manager marketing centerpiece unless the Rhino matrix is proven green for that family.

| Surface | Honesty |
|---------|---------|
| Stewart (`Family=stewart`) | `Q` = leg lengths **meters**; Waypoints/Export warn ≠ MoveJ |
| Legged Walk / `PlanBodyPath` (`Family=legged`) | Full-driver **radians**; body-path ≠ TCP LIN; hard SSM on Plan |
| Holonomic SE(2) | Joint Table `BaseSE2` / `Mobility=HolonomicSE2`; base x/y m, yaw rad |

## Experimental (fixtures / optional GH load)

Motus.NET tests + docs; **no** Motus.Grasshopper GA Aerial / H2 / Go2 components.

| Track | In Motus | Not Motus |
|-------|----------|-----------|
| Aerial / HolonomicSE3 (`Family=aerial`) | SE(3) body plan, hull collision, export `bodyPose` / `waypointsQ=not_ur_movej` | Flight stack, MAVLink, live SITL |
| Unitree H2 | Meshless LoadTree / FK / arm group fixture | Balance, biped Walk GA |
| Unitree Go2 | Meshless FK stance smoke (≠ insectoid legged) | Spot-class loco product |
| Catalog Panda / UR10e+Robotiq minimal | CI smoke fixtures | Claiming every upstream URDF is GA |

See [aerial.md](aerial.md), ADR `docs/adr/0001-holonomic-se3-aerial.md`, and roadmap experimental mobility.

## Out through 2.0

Physical robot control (RTDE / Session / Run), ROS / MoveIt runtime, vendor SDKs — remain out unless product scope expands.

## Host mapping

| Tier | Motus.NET | Motus.Grasshopper |
|------|-----------|-------------------|
| Supported | Core planners + pick/place APIs | Motus tab GA components; Yak `motus` at **2.0.0** |
| Advanced | Stewart / legged / SE2 stacks | Stewart, Walk, Joint Table SE2 — honest warnings |
| Experimental | Fixtures + `Units.IsAerial` | Waypoints/Export aerial warn; optional Robot URDF experimental Status |
