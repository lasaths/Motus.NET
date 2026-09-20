# Aerial / full drone (Motus meaning)

**ADR:** [0001-holonomic-se3-aerial.md](adr/0001-holonomic-se3-aerial.md)  
**API:** `MobilityModel.HolonomicSE3`, `MobilityBoundsSE3`, `Units.AerialFamily`, `FreeFlyerHullCollisionChecker`

## What “full drone” means here

Offline **planning / preview / export** for a free-flying rigid body in **SE(3)** (multirotor as collision hull + body frame). Managed sampling appends **x/y/z/roll/pitch/yaw** to the plan space. Orientation is **RPY** (URDF fixed-axis XYZ) with **singularity Status** near pitch ±π/2.

Collision uses `FreeFlyerHullCollisionChecker` (sphere envelope at the per-sample `BaseFrame`) via `IBaseFrameCollisionChecker`. Export (`TrajectoryExport`) for `Family=aerial` writes **bodyPose** (m + RPY rad) and marks `waypointsQ=not_ur_movej` — never `jointsRadians` / MoveJ.

## What it does *not* mean

- Not PX4 / ArduPilot / Betaflight / MAVLink
- Not battery, wind, GPS, live SITL, or propulsion mixing
- Not a claim that Motus missions are hardware-flyable

Fixture: `tests/fixtures/aerial/free_flyer_box.urdf` (meshless box hull; circumscribed sphere ≈ 0.146 m).

## Pass-off with a serial arm (2.1)

**ADR:** [0002-aerial-arm-pass-off.md](adr/0002-aerial-arm-pass-off.md)

GH ships **drone-only** first (`11_aerial_hover`) until HolonomicSE3 Plan → Preview is trusted. Sequential dual-program (no multi-robot state space) remains the Motus.NET contract:

1. Plan the free-flyer with `HolonomicSE3` + `FreeFlyerHullCollisionChecker` (or `.WithAttached` for payload); prefer `MobilityBoundsSE3.HoverHandoff` for approach/hover; optional `AerialStationHold.AppendHold`.
2. Carry the brick as a **base-local** `AttachedBody` (Phase B). Scene-brick relocate remains valid for hosts that only hide/show obstacles.
3. Run serial `PickPlaceCycle` (example 10: Touch + SET open/close) from the handoff grasp to finish the tower.
4. Export aerial `bodyPose` and serial joints separately — never MoveJ aerial `Q`.
5. **One Play** (GH, later): Preview list = aerial `Tr` then arm `Tr` (shared scrub; hold outside each window).
