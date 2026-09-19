# ADR 0001: Holonomic SE(3) free-flyer mobility (aerial)

## Status

Accepted (foundation slice). Experimental / parallel to industrial 1.8→2.0.

## Context

Motus already plans holonomic **SE(2)** mobile bases (`MobilityModel.HolonomicSE2`: x/y/yaw, Z fixed). Multirotor / free-flyer offline planning needs **SE(3)** body poses with collision and honest export — without becoming a flight computer (no PX4, ArduPilot, MAVLink, battery, or rate loops).

## Decision

1. **`MobilityModel.HolonomicSE3`** — position (m) + fixed-axis XYZ **RPY** (rad), same ZYX convention as URDF / `Transforms.FromRpy`.
2. **Ponytail orientation = RPY** with named **singularity Status** near pitch ±π/2 (`cos(pitch)≈0`). Quaternion *sampling* is deferred; `Frame` storage remains quaternion.
3. **`MobilityBoundsSE3`** — six `JointLimit` dims appended after joint space (`+6`), separate from SE(2) `MobilityBounds` so serial/Stewart/SE2 contracts stay unchanged.
4. **`RobotPreset.Family = "aerial"`** (`Units.IsAerial`) — Waypoints/export must not treat body SE(3) as UR MoveJ radians. `Q` may be empty (0 actuated joints) or unused props.
5. **Managed sampling only** — same path as SE2 (native OMPL bypassed when `Mobility` is set). Per-sample `BaseFrame` overrides reuse `IBaseFrameCollisionChecker`.
6. **Out of scope for this ADR:** autopilots, dynamics as a product, minimum-snap corridors, GH aerial components.

## Defaults

| Parameter | Default |
|-----------|---------|
| XYZ bounds | X/Y ±2 m; Z ∈ [0, 3] m |
| Roll / yaw | ±π rad |
| Pitch | ±1.4 rad (strictly inside (−π/2, π/2)) |
| Pitch singularity | `|cos(pitch)| < 1e-3` → Status |

## Consequences

- SE(2) API unchanged; planners accept SE2 **or** SE3, not mixed.
- Serial / Stewart / legged planning paths untouched when `Mobility` is null.
- Full “Motus drone” exit = SE3 plan + base-frame collision (`FreeFlyerHullCollisionChecker` / `IBaseFrameCollisionChecker`) + honest Waypoints/export (`Family=aerial`, bodyPose not MoveJ) + METHODS (this ADR). Autopilot remains forever out of Motus.
- Collision clear/blocked RRT smoke and aerial JSON/CSV export honesty are part of the deepen slice after the foundation.