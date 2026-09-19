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
