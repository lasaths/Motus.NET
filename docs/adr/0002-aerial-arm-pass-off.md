# ADR 0002: Aerial ↔ serial arm pass-off (Motus 2.1)

## Status

Accepted (foundation for 2.1). Experimental / parallel to Yak GA until Supported surface expands.

## Context

Motus 2.0 ships HolonomicSE3 free-flyer planning (`Family=aerial`, ADR 0001) and reliable serial pick/place (`PickPlaceCycle`, example 10). Product wants a **drone brings a brick → arm finishes the tower** demo without a flight computer or multi-robot state space.

Gaps today:

- No dual-agent planner; `PlanningGroup` is one robot’s joints.
- `AttachedBody` is TCP-local; free-flyer collision rides `BaseFrameOverride` via `IBaseFrameCollisionChecker` and does not carry attach payloads.
- No first-class aerial “station-hold” primitive (arm has `WaitSegment`).

## Decision

1. **Sequential dual-program, shared scene** — Plan the drone with `SamplingPlanner` + `HolonomicSE3` + `FreeFlyerHullCollisionChecker` against a shared `CollisionScene`. Plan the arm with `PickPlaceCycle` → `IndustrialMotionPlanner` / Motus Program (example 10 contracts: SET open/close, `TouchBodies`, Attach/Detach). Do **not** invent a MultiRobot planner for 2.1.
2. **Pass-off seam** — Agree a world handoff pose `W`. Drone SE3 goal ≈ hover at `W` with payload via **base-frame attach** (`FreeFlyerHullCollisionChecker.WithAttached` / `BaseFrameAttachCollisionChecker`; `AttachedBody.TcpLocalPose` = base-local). After drone plan + optional `AerialStationHold.AppendHold`, brick rests at grasp-ready world pose; arm `Expand` from that grasp → tower places. **One Play** in GH: Preview list = aerial `Tr` then arm `Tr` (shared scrub; hold outside each window). Export streams stay separate: aerial `bodyPose` vs serial jointsRadians / Waypoints `Q`.
3. **Hover stability (offline)** — Prefer `MobilityBoundsSE3.HoverHandoff` (tighter roll/pitch) for approach + station poses; optional CHOMP-lite smooth on the SE3 path when the seed is clear. “Hold” = `AerialStationHold.AppendHold` (arm may `WaitSegment` while the host treats the drone as parked). Named Status on pitch singularity / OOB / collision — no silent NaN.
4. **Phase B (landed)** — Base-frame attach: `AttachedBody` parent = base + `IBaseFrameCollisionChecker` that transforms payload with `BaseFrameOverride`. SamplingPlanner auto-wraps when `Mobility` + `AttachedBodies` + `IBaseFrameCollisionChecker`.
5. **Grasshopper** — Thin wiring + Cassis-authored example derived from `10_pick_place` (sibling ADR). No Motus Aerial GA component required for the first cut if Motus Robot URDF + Plan/Program + Export warnings already cover the path under UseLocal / tip.

## Out of scope

PX4 / ArduPilot / MAVLink, wind, battery, live RTDE, synchronized multi-robot RRT in one state vector.

## Consequences

- Example 10 serial tower path stays green; pass-off is additive.
- Docs / METHODS cite ADR 0001 + this ADR; regression adds a Motus.NET logic test for handoff → PickPlace Touch/SET contract.
- Yak Supported surface unchanged until product explicitly promotes aerial + dual-agent demo.
