# Next steps plan (Milestone 15, after PR #42)

This file is a hand-off plan. An agent or a person can continue from here without other context.
Status on 2026-11-11: PR #42 (branch `cursor/arch-deepening`) is stacked on PR #41 (`cursor/gridshell-3d-rigidity`). Neither PR is merged. Do the steps in order. Make one commit for each step.

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
The option only prunes. It does not make a new path. Next: scout one shared stock spot away from the anchor columns, then re-run the tower with `RELAYPRE=1` (heavy, needs the owner's go-ahead).

## Later (not planned in detail)

- Tower story 3: relays, or a stock height for each stock spot.
- Relax assumptions one at a time: vertical anchors, straight-up approach, fixed stock spots.
- More seeds and platforms for dome and tower (only seed 11 and Linux x64 were tested).
- Reach map and scene check into core, as its own PR.
- Planning session (about 99 references in 26 files), in small steps.
- Do not generalise stability or 3D rigidity until a second structure type needs them.
- Rhino and Grasshopper are not checked for #41 and #42.
