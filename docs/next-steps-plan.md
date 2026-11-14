# Next steps plan (Milestone 15, after PR #42)

This file is a hand-off plan. An agent or a person can continue from here without other context.
Status on 2026-11-11 (Milestone 16 added 2026-11-13): PR #42 (branch `cursor/arch-deepening`) is stacked on PR #41 (`cursor/gridshell-3d-rigidity`). Neither PR is merged. Do the steps in order. Make one commit for each step.

## Rules for every step

- Make the smallest change that solves the step. Do not change defaults. New behaviour is opt-in.
- Keep the collision check exact at 2 mm. Do not loosen the tolerance. Do not use spheres as the final check.
- Motus.Core and Motus.Geometry stay RhinoCommon-free and bamboo-free.
- Do not change which canes are fixed or grippable.
- Stock comes from one fixed stock spot.
- Run one heavy run at a time.
- Do not merge, tag or publish packages without the owner's OK. Do not force-push.

## Gates

Run all gates before you commit a code step. If a gate fails, stop and report. Do not commit.

1. Full core suite (expect 570/570 or more):
   `dotnet test tests/Motus.Core.Tests -c Release`
2. Warren 7/7 and the shipped-plan check are in that suite:
   `BuildSearch_DerivesTheShippedSequence_EveryChoicePlannerConfirmed` and `ShippedDefaultPlan_IsIdenticalToLivePlanning` (tests/Motus.Core.Tests/BambooTrussTests.cs).
3. Dome gate (slow, about 1.2–1.4 h on 8 cores):
   `MOTUS_DOME_BUILD=1 dotnet test tests/Motus.Core.Tests -c Release --filter Dome3x3_Example_BuildsComplete_ReplaysAndIsClear`
   Expect 24/24, 65 confirmed, 40 rejected, stop `complete`. Last pass: 871e6a8, 4279 s.

## Step 1 – Move 3: certified segments into core

Goal: a generic segment certifier that any robot task can use. Bamboo becomes one caller.

Today: `src/Motus.Construction/Bamboo/BambooSegmentCertifier.cs` (220 lines). Callers: `BambooTrussScan.cs`, `BambooTrussPlanner.cs`, `MotionConstraints.cs`, test `tests/Motus.Core.Tests/BambooTrussTests.Certify.cs`.

Do:
1. Find the generic part: the bisection loop (midpoint clear of obstacles grown by ε·(b−a)/2), the motion bound from joint deltas times reach from each joint axis, and the grow quantum (1e-4·2^k).
2. Put it in Motus.Core (for example `SegmentCertifier.cs`) behind a small interface: the robot gives its sweep bound and an "is the pose clear of obstacles grown by r" check. Do not reference bamboo types.
3. Keep `BambooSegmentCertifier` as a thin adapter (the bamboo body spheres, jaw strokes, held cane, `BambooTrussCollision.Inflate`). Keep its public API.
4. Add one Motus.Core test with a plain 2-link arm and one box: a segment that passes through the box is rejected, and a clear segment is certified.
5. Update docs/METHODS.md (the certified-segments row) and docs/architecture.md in ASD-STE100 style.

Done when: all three gates pass, with the same counts as before.

## Step 2 – Tower: brace a node early (opt-in)

Problem: the 1x2, 3-story tower (60 canes) places 6/60 with one stock spot. The cause is a missing foothold, not the stock position. No anchor has a Motus-clear grasp of `anchor-N002`. A placed post that is fixed at one end only cannot be clamped.

Inputs are in `tests/fixtures/tower1x2/` (`tower-input.json`, and `harness/` is the runner that made the outT3 result). Build the harness with `dotnet build -c Release tests/fixtures/tower1x2/harness`. The outT3 run settings were:
`BAY=0.28 STORY=0.30 SITELEN=0.22 STARTHEAD=0 STOCK=-0.25,0.05,45 MAXREJ=250 LANDPRE=1 CONSTRAINED=1`, then
`dotnet tests/fixtures/tower1x2/harness/bin/Release/net8.0/tower-harness.dll tests/fixtures/bamboo_icd/bamboo_icd.urdf 1 plain <outdir>`.
Baseline outT3: 6/60, 9 confirmed, 250 rejected, search 3187 s. Placed: N000N100, N010N100, N011N111, N011N110, N001N100, N011N112.

Do:
1. Add an opt-in option (off by default) that ranks first the cane that fixes a node at both ends, so a placed post becomes clampable. That gives the robot a foothold into the second bay.
2. Run gates 1 and 2 with the option off.
3. Run the tower with the option on. Compare with the outT3 baseline. Check a fresh replay of the result and the 2 mm scan.
4. Report placed count, confirmed and rejected counts, the stop reason and the time.

Done when: gates pass with the option off, and the tower result is reported. Commit the option even if the tower gets better but not complete.

Result (2026-11-11, f7a8648): the option was `Options.BraceFirst` (harness: `BRACE=1`). It did not change the result, so it was removed again in the next commit. The core suite passed 571/571 with the option off.
The tower run outT4 (outT3 settings + `BRACE=1`) placed 6/60, with 9 confirmed and 250 rejected, stop `rejected confirmation limit (250)`, search 1960 s.
The search path is the same as outT3. A fresh replay built 6 members, and the 2 mm scan is collision-free.
The option ranked `N001N100` first, but the default search already placed it 5th. That cane fixes `N000N100`.
New stall: after the six canes, the search used its rejections on relays of the second-story canes
(`N100N200`, `N110N200`, `N111N210`, `N111N211`, `N110N210`). The planner found no landing for each relay park.
The remaining first-story canes had no Motus-clear R grasp, or the planner found no landing for them.
Next: give the robot a foothold for the relay park on the second story (or a stock spot nearer the second story).

Diagnosis (2026-11-12): the relays are not the real stall. After the six canes, none of the 14 remaining first-story canes can be built, directly or after one landing.
The cause is the stock pick. From every stand the robot can reach, the R arm collides when it grasps the one stock (`θ4 ↔ anchor-N001` or `θ4 ↔ L-grip rail`), or it has no IK.
A second-story foothold does not help. Relays used 164 of the 250 rejections.

## Step 3 – Relay precheck (opt-in)

Done: `Options.RelayPrecheck` (harness: `RELAYPRE=1`), off by default.
With the option on, the search skips a relay before the planner confirms it. It skips the relay when, by IK only, no one-step landing (or the stand) reaches the stock and the park, or no one-step landing (or the stand) reaches the place.
A skip is not a rejection. The log shows `relay <id>: skipped — <reason>`.
The option only prunes. It does not make a new path.

Result (2026-11-13): outT5 (outT4 settings + `RELAYPRE=1`) placed 6/60, 20 confirmed, 250 rejected, by the same path as outT4. The precheck skipped 123 relays.
outT6 (outT5 settings + `FOOTHOLDS=1`, which sets `Options.PlacedFootholds` and so `FixedMemberRule = false`) placed 7/60, 9 confirmed, 250 rejected, search 3274 s. Two canes went to story 2 by relay. Fresh replay and the 2 mm scan are clean; the 3 mm margin fails on the base tube against `anchor-N001`.
Most late rejections fail at the same stock grasp, beside `anchor-N001`. The next tower step is a stock-spot change (owner's decision), see Milestone 16, step G11.

## Milestone 16 – Toward a general planner (plan, 2026-11-13)

Goal: one planner that serves these setups: a 6-axis arm, a 6-axis arm on a linear track, a 6-axis arm with a 1- or 2-axis positioner (DKP), and the bamboo mobile robot. Later: 7-axis arms, a gantry, two arms, a mobile manipulator, and "robot holds the part".
Source: a code review with small runs on 2026-11-13 (CET), on PR #42 `74ad079`. Runs: a scratch console, not in this repo. Evaluation Page: "Motus robot setups × development state".

### What the review found

- Numerical IK does not converge. It succeeds only when the seed is already the answer. Seed 0.05 or 0.2 rad off the answer: rail + UR10e, Panda and KR 210 URDF 0/40 each (`SingularJacobian` at iteration 1). KR 10 and iiwa 7 presets 0/40 each (`NoConvergence` after 400 iterations). UR analytic IK: 40/40 (UR10e preset), 3/3 (bundled UR10e + Robotiq URDF). The same code is on master.
- The existing IK tests seed at the answer (`CatalogRobotSmokeTests.cs:97-99`, `PoEParityTests.cs`), so CI does not find the failure.
- UR analytic IK fails (0/3) on the `ur10e.urdf` and `ur10e_minimal.urdf` fixtures. Cause not known.
- `TreeFkCollisionChecker` tests robot links against scene objects only. It does not test links against each other. Arm against positioner, and arm against arm in one tree, are not checked.
- `UrdfRobotLoader` gives a prismatic joint a radians limit (`UrdfRobotLoader.cs:78`). Serial export writes `jointAngles = radians`, `jointsRadians` and `joint_N_rad` for every joint (`TrajectoryExport.cs:269,317,344`).
- Serial export drops `BaseFrameOverride`, so SE(2) base poses are lost. Only the aerial export writes a base pose.
- For AllDrivers, Cartesian goals and LIN use tip-chain IK. Side branches stay at the start value (GH `PlanExecutor.cs:375`). `EmbedSideBranches` also drops the LIN motion type and segment index.
- The search kernel interfaces (`IPlanningState`, `ICandidateGenerator`, `IConfirmer`, `ITaskRunner`) are `internal` to Motus.Construction.
- Numerical IK has no secondary objective. It keeps the solution with the smallest joint change from 13 seeds.
- No remote-TCP mode. No URScript, KRL or RAPID writer.
- Bamboo uses its own grip IK (`BambooGripKinematics`). The IK fault does not change the build search.

### Rules for Milestone 16

The rules and gates at the top of this file apply. Also:

- One step = one PR (or one commit on a stacked branch). Steps G0 and G1 are bug fixes. They can change results for non-UR robots. Say so in the CHANGELOG.
- Every new option is off by default. Do not change a default without the owner's OK.
- Keep the exact 2 mm collision check. A new collision check can only add contacts. It never removes one.
- Warren 7/7 and `ShippedDefaultPlan_IsIdenticalToLivePlanning` must stay identical after every step.
- Run the dome gate for a step that touches the search, the bamboo planner, collision used by bamboo, or IK used by bamboo.
- Sizes are estimates: S = up to 1 day, M = 2–4 days, L = 1–2 weeks, XL = more than 2 weeks.
- "Owner's OK" = Lassie's explicit OK before the step starts or before its result is merged.

### Order

| Step | Item | Needs | Size (estimate) | Owner's OK |
|------|------|-------|-----------------|------------|
| G0 | Fix numerical IK | – | S–M | Merge |
| G1 | Link-to-link collision in TreeFK | – | M | Merge |
| G2 | Prismatic units in loader and export | – | S | Merge (export shape) |
| G3 | Track: LIN and coordinated axes | G0, G2 | M | Merge |
| G4 | Positioner: coordinated motion, export motion type | G0, G1 | M–L | Merge |
| G5 | Mobile base: export the base pose | – | S | Merge (export shape) |
| G6 | L2: general AssemblyPlanner (UR pick-place tower) | G0 | L | API choice, ADR |
| G7 | Redundancy: secondary objective (opt-in) | G0 | M | Merge |
| G8 | Remote TCP (robot holds the part) | G0 | S–M | Merge |
| G9 | Gantry and two-arm fixtures | G0, G1, G3 | M | Merge |
| G10 | Robot-code writers | – | open | Scope decision |
| G11 | Bamboo tower | stock spot | M | Stock spot, heavy runs |
| G12 | Merge #41, #42, #43 | – | S | Merge |
| G13 | Rhino matrix pass | deferred | M | Start |

### G0 – Fix numerical IK (P0)

Goal: `NumericalInverseKinematics` converges from a seed that is not the answer, for URDF (PoE) and DH (finite-difference) chains. UR analytic IK stays as it is.

Files: `src/Motus.Geometry/NumericalInverseKinematics.cs`, `src/Motus.Geometry/PoEJacobian.cs`, tests in `tests/Motus.Core.Tests/` (new `NumericalIkConvergenceTests.cs`), `docs/METHODS.md` (PoE FK/IK row), `CHANGELOG.md`.

Do:
1. Singularity check (`PoEJacobian.cs:75-91`, used at `NumericalInverseKinematics.cs:199-205`). The Gershgorin lower bound for the smallest eigenvalue of J·Jᵀ is 0 for normal poses, so the estimate is infinity. Replace it with a real value: the smallest eigenvalue of the 6×6 J·Jᵀ (Jacobi), or the smallest singular value. Keep the 1e10 limit.
2. Step sign (`NumericalInverseKinematics.cs:209-223`). The finite-difference Jacobian is d(error)/dq with error = target − current, and the code adds the DLS step. Check the sign with a one-joint test. Then fix it: use the TCP Jacobian, or subtract the step.
3. Off-answer tests. For each chain: random goal poses inside the limits, seed 0.05 rad and 0.2 rad off the goal joints. Chains: rail + UR10e URDF (fixture made in the test), Panda, KR 210 URDF, KR 10 DH preset, iiwa 7 DH preset, UR10e URDF as a generic chain. Expect at least 95% success at 0.05 rad. Also test a goal out of reach: it must fail with a named reason.
4. Plain UR10e URDF: find why UR analytic IK fails on `tests/fixtures/ur10e/ur10e.urdf` and `ur10e_minimal.urdf` (0/3; the bundled UR10e + Robotiq URDF passes). Check the base Rz(π) and the tool0 frame in `UrdfUniversalRobotsKinematics`. Fix it only if the cause is a frame offset. Add a test.

Proof: new tests pass. Full core suite passes. Warren 7/7 identical. UR analytic results unchanged (`RegressionMatrixLogicTests`, `CartesianGoalSolverTests`). The dome gate is not needed: bamboo uses only the FK from `KinematicsResolver` (`BambooIcdCell.cs:101`), not this IK.

Risk: plans for non-UR robots change, because IK starts to succeed. That is the fix, but say it in the CHANGELOG.

### G1 – Link-to-link collision in TreeFK (P0)

Goal: `TreeFkCollisionChecker` also tests robot links against each other: across branches (arm against positioner or workpiece link, arm against arm) and inside a chain, with the same SRDF / adjacent-link exemptions as the serial checkers.

Files: `src/Motus.Geometry/TreeFkCollisionChecker.cs`, maybe `src/Motus.Geometry/RobotMeshCollisionChecker.cs` (share the pair filter), tests (`TreeFkCollisionCheckerTests.cs`), `docs/METHODS.md`.

Do:
1. Build the link-pair list once: skip parent–child pairs, pairs the SRDF disables, and pairs with no geometry. Use exact narrow phase, as for scene objects.
2. Make it the default for trees, because a missed contact is a safety bug. If the owner wants the old behaviour, add an opt-out option. Do not make it opt-in.
3. Tests: arm link inside the turntable box is a contact (the review case). Arm clear of the turntable is free. Two arms that cross is a contact.

Proof: new tests pass. Full core suite. Warren 7/7 identical. The dome gate is not needed: Motus.Construction does not use `TreeFkCollisionChecker` (checked 2026-11-13).

Owner's OK: needed to merge, because it changes results (some plans that passed will now fail). The 2 mm check is not loosened.

### G2 – Prismatic units in loader and export

Goal: a prismatic joint is metres everywhere.

Files: `src/Motus.Presets/UrdfRobotLoader.cs` (line 78: set `JointCoordinateUnit.Meters` for prismatic), `src/Motus.Core/TrajectoryExport.cs` (per-joint unit), export golden fixtures in `tests/fixtures/exports`, `docs/data-model.md`.

Do:
1. Loader: set the unit from the URDF joint type (GH already does this in Serial Chain, Joint Table and `PlanDofComposer`).
2. Export: add a per-joint unit list (`units.joints = ["m", "rad", …]`) and CSV headers `joint_N_m` for prismatic joints. Keep `jointsRadians` and `jointAngles` for serial arms with only revolute joints, so the PlanBundle golden file does not change. For mixed chains, decide with the owner: rename the field, or keep it and add the unit list.

Proof: golden export test unchanged for UR. New test: rail chain exports metres for the rail. Full core suite. Warren 7/7.

Owner's OK: the export contract for mixed chains.

### G3 – Track: LIN and coordinated axes

Goal: Cartesian goals and TCP LIN work on a 6-axis arm on a rail. The rail moves with the arm.

Files: new fixture `tests/fixtures/ur10e_track/` (UR10e on a 4 m prismatic rail), `src/Motus.Geometry/CartesianLinearPathPlanner.cs` (only if needed), tests, GH example (Motus.Grasshopper, separate PR).

Do:
1. Tests: IK reach to a goal that needs the rail; LIN of 2 m along the rail; joint-goal RRT.
2. Optional, opt-in: a rail preference (weight per joint in the DLS step) so the arm moves before the rail, or the other way. Default: no weights.
3. GH: an example with the track URDF and a plane goal.

Proof: tests pass. Full core suite. Warren 7/7. LIN keeps the 2 mm collision check.

### G4 – Positioner: coordinated motion and export motion type

Goal: a 1- or 2-axis positioner moves with the arm on Cartesian goals and LIN. The target can be in the workpiece frame.

Files: `src/Motus.Geometry/KinematicTree.cs` (or a new `TreeInverseKinematics.cs`), `src/Motus.Geometry/CartesianLinearPathPlanner.cs`, new fixture `tests/fixtures/ur10e_dkp2/` (2-axis tilt–turn), GH `src/Motus.GH/Planning/PlanExecutor.cs` (`EmbedSideBranches`: keep `MotionType`, `SegmentIndex`, `BlendRadiusMeters`, `ToolState`).

Do:
1. Relative IK: solve arm + positioner joints so the TCP meets a target given in the positioner flange frame. The positioner joints are IK variables.
2. Opt-in secondary rule for the positioner (for example: keep the tool axis near vertical, or keep the positioner still), from G7.
3. Default stays as today (positioner held for Cartesian moves). The new mode is opt-in.
4. GH: keep motion metadata in `EmbedSideBranches` (bug fix, small).

Proof: tests with the 1-axis (example 06 geometry) and 2-axis fixtures; LIN with a moving workpiece. Collision uses G1. Full core suite. Warren 7/7.

### G5 – Mobile base: export the base pose

Goal: serial export writes the base x, y, yaw for each point when `BaseFrameOverride` is set.

Files: `src/Motus.Core/TrajectoryExport.cs` (serial path, `:305-323`; CSV), tests, `docs/data-model.md`.

Do: add an optional `baseFrame` per point in JSON and `base_x_m, base_y_m, base_yaw_rad` columns in CSV only when a point has an override. Fixed-base exports do not change.

Proof: golden export unchanged for fixed-base. New SE(2) export test. Full core suite.

### G6 – L2: a general AssemblyPlanner (UR pick-place tower)

Goal: the build search runs for a robot that is not bamboo. Proof: a UR10e + Robotiq pick-place tower through the same kernel.

Files: `src/Motus.Construction/Search/*` (kernel), a new adapter (for example `src/Motus.Construction/PickPlace/`), `src/Motus.Construction/AssemblyPlanner.cs`, tests, a new Motus.Construction ADR, `docs/METHODS.md`.

Decision for the owner (ADR):
- Option A: a second adapter inside Motus.Construction. The interfaces stay `internal`. Small API change.
- Option B: make `IPlanningState`, `ICandidateGenerator`, `IConfirmer` and `ITaskRunner` public. Larger API surface and a stability promise.

Do: write the ADR with both options. After the choice: the adapter (state = placed parts; candidates = next part; confirmer = `PickPlaceCycle` + collision), and a 5×4 tower test (example 10 geometry).

Proof: UR tower test complete and replay-identical. Warren 7/7 identical. Dome gate (kernel change). Full core suite.

### G7 – Redundancy: secondary objective (opt-in)

Goal: for 7-axis arms, tracks, gantries and positioners, the IK can prefer a solution by a rule, not only by the nearest seed.

Files: `src/Motus.Geometry/NumericalInverseKinematics.cs`, `src/Motus.Geometry/NumericalIkOptions.cs`, tests.

Do: an opt-in null-space term in the DLS step: joint-limit distance, joint weights, or a rest pose. Default: off (today's result).

Proof: with the option off, every IK test result is identical. With it on, a Panda LIN keeps the elbow continuous. Full core suite.

### G8 – Remote TCP (robot holds the part)

Goal: the robot carries the part along a fixed world tool (spindle, glue nozzle).

Files: `src/Motus.Geometry/CartesianLinearPathPlanner.cs`, `src/Motus.Geometry/IndustrialMotionPlanner.cs` (LIN, CIRC), `src/Motus.Core/CartesianPlanning.cs`, tests, a GH option on Motus Plan (separate PR).

Do: an opt-in remote-TCP mode: a fixed world tool frame, and targets given in the part frame. Each target becomes TCP = tool · target⁻¹ before IK.

Proof: a LIN along a part edge past a fixed tool; attached-body collision stays exact. Full core suite.

### G9 – Gantry and two-arm fixtures

Goal: fixtures and tests for a 2- or 3-axis gantry with an arm, and for two arms in one tree.

Files: new fixtures under `tests/fixtures/`, tests.

Do: gantry: reuse the G3 tests. Two arms: joint-goal RRT on both arms, arm-against-arm contact (G1), Cartesian goal on one tip. Synchronised two-arm motion stays out of scope (Motus.NET ADR 0002) unless the owner changes the scope.

Proof: tests pass. Full core suite.

### G10 – Robot-code writers (open decision)

Today: export is JSON/CSV and Waypoints `Q` → MoveJ. No URScript, KRL or RAPID writer. The roadmap puts physical control, ROS and vendor SDKs out of scope. It does not say if offline robot-code files are in scope.

Decision for the owner: in scope (offline files only, no live I/O) or out of scope. If in scope: start with URScript (`movej` / `movel` from the joint trajectory), as a separate package, with a golden-file test. Size: M for URScript, M for each other dialect (estimate).

### G11 – Bamboo tower

Today: 7/60 (outT6, `FOOTHOLDS=1`). The stock grasp beside `anchor-N001` blocks most candidates.

Decision for the owner: move or raise the one shared stock spot away from `anchor-N001` (it stays one fixed spot for every cane), add a second approach direction for the stock pick, or raise the 250-rejection limit. `FOOTHOLDS=1` (all placed canes clampable) is approximate for pin-jointed canes, and needs the owner's OK for each run.

Proof: a tower run (heavy, one at a time), fresh replay, 2 mm scan. Defaults unchanged; Warren 7/7; dome gate if the search changes.

### G12 – Merge #41, #42, #43

Owner's OK. In this order: #41, then #42, then #43 (Tandem's ranker logging, based on #42). No tags, no package publish without the owner's OK.

### G13 – Rhino matrix pass (deferred)

The owner deferred Rhino: "solve the issues we have before going to rhino". After G0–G5: run the Motus.Grasshopper `docs/regression-matrix.md` rows in Rhino and tick them. A setup becomes Supported only with an ADR, a METHODS entry and a Rhino matrix proof.

## Later (not planned in detail)

- Tower story 3: relays, or a stock height for each stock spot.
- Relax assumptions one at a time: vertical anchors, straight-up approach, fixed stock spots.
- More seeds and platforms for dome and tower (only seed 11 and Linux x64 were tested).
- Reach map and scene check into core, as its own PR.
- Planning session (about 99 references in 26 files), in small steps.
- Do not generalise stability or 3D rigidity until a second structure type needs them.
- Rhino and Grasshopper are not checked for #41 and #42.
