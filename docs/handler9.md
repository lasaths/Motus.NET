# Bamboo handler `handler9` (Experimental)

Status: **Experimental**. This robot is not in the Supported list. Promotion needs an ADR, a METHODS entry and a Rhino matrix proof.

Sources:

- Lassie's slide "9 Degrees of Freedom".
- Lassie's CAD "Bambino Kinematic Schematics.step" (Autodesk export, mm, Y-up). The CAD is the authority for geometry.

Files:

- URDF: `tests/fixtures/handler9/handler9.urdf`. Do not edit it by hand.
- Meshes: `tests/fixtures/handler9/meshes/*.stl` (one per CAD part, in its link frame, CAD units) and `meshes/boxes.json` (collision box per part).
- Generator: `tools/handler9/gen_handler9_urdf.py`. All dimensions are in `DIMS`. `LU` sets the scale.
- Mesh export: `tools/handler9/export_meshes.py model.step` (needs `cadquery-ocp` and `trimesh`).
- Kinematics: `src/Motus.Construction/Bamboo/Handler9Kinematics.cs`.
- Tests: `tests/Motus.Core.Tests/Handler9KinematicsTests.cs`.

This robot is not `bamboo_icd`. `bamboo_icd` is a 5R arm that hangs under a pole and walks with two grips. This robot is a fixed beam with three stations.

## CAD parts and joints

The CAD has 13 solids. Each moving part maps to one link.

| CAD part | Link | Moves with |
|----------|------|------------|
| Backbone (unnamed solid) | `base_link` | fixed |
| HandlingGripper1 / HG_Plate+ExtentionAxis (round plate + vertical shaft) | `handler_1_carriage` | `handler_1_yaw`, `handler_1_lift` |
| HandlingGripper1 / StaticGripper | `handler_1_clamp` | the carriage (fixed joint) |
| HandlingGripper1 / ActiveGriper - 95 LegthUnits Extention | `handler_1_active` | `handler_1_feed` |
| HandlingGripper1(Mirror) / … | `handler_2_*` | the same joints of Handler_2 |
| CenterPlate / CP_Plate+ExtentionAxis | `main_clamp_carriage` | `main_clamp_yaw`, `main_clamp_lift` |
| CP_Active+Rotational Gripper(Mirror) / Linear Translation(Mirror) | `main_clamp_blue_sled` | `main_clamp_blue_slide` |
| CP_Active+Rotational Gripper(Mirror) / Axial Rotation_+-40Deg(Mirror) | `main_clamp_blue` | `main_clamp_blue_roll` |
| CP_Active+Rotational Gripper / Linear Translation, Axial Rotation_+-40Deg | `main_clamp_yellow_sled`, `main_clamp_yellow` | `main_clamp_yellow_slide`, `main_clamp_yellow_roll` |
| Bamboo / 60LU_Dia | not a robot part | — |

Handler_1 is the non-mirror handler at −x. Blue is the MainClamp gripper at −x (as on the slide).

## DOF interpretation (changed by the CAD)

The CAD and the slide labels show 12 joints: 3 + 6 + 3. The slide title says 9.
The URDF has the 12 CAD joints. 10 joints are planned DOF. Two joints are passive.

Single roll drive (Lassie, from photos of the MainClamp):

- One roll gripper is driven (active). The other roll gripper is passive. It follows the cane.
- Both grippers hold one rigid cane on one axis. Thus the passive roll angle equals the active roll angle.
- `main_clamp_yellow_roll` is a URDF `<mimic>` of `main_clamp_blue_roll` (multiplier 1, offset 0). It is not a planned DOF.
- ASSUMPTION: blue is the active roll. The CAD does not show which roll has the drive: the two grippers are exact mirror parts (same volume, no motor or drive feature).

Coupled MainClamp slides (Lassie: one motor drives both):

- `main_clamp_yellow_slide` is a URDF `<mimic>` of `main_clamp_blue_slide` (multiplier −1, offset 0). It is not a planned DOF.
- Both slide axes are +x. Blue limits are 0..+110 LU. Yellow limits are −110..0 LU. Thus yellow = −blue moves both grippers toward the centre, or both away from it.

Planned DOF: 10 (handlers 3 + 3, MainClamp 4: yaw, lift, blue slide, blue roll).

| Station | Slide label | Joint | Type, axis | Range |
|---------|-------------|-------|------------|-------|
| Handler_1, Handler_2 | Rotation about the vertical axis | `handler_N_yaw` | revolute, z | −180°..+180° (slide) |
| Handler_1, Handler_2 | Axial linear vertical movement | `handler_N_lift` | prismatic, z | 0..90 LU (inferred) |
| Handler_1, Handler_2 | Linear Moving Clamp | `handler_N_feed` | prismatic, along the cane | 0..95 LU away from the static clamp (part name) |
| MainClamp | Rotation about the vertical axis | `main_clamp_yaw` | revolute, z | −180°..+180° (slide) |
| MainClamp | Axial linear vertical movement | `main_clamp_lift` | prismatic, z | 0..95 LU (inferred) |
| MainClamp | Linear Movement / Linear Moving Clamp (blue, active) | `main_clamp_blue_slide` | prismatic, along the cane | 0..110 LU toward the centre (inferred) |
| MainClamp | Linear Movement (yellow, passive) | `main_clamp_yellow_slide` | mimic of `main_clamp_blue_slide` (multiplier −1), not planned | −110..0 LU |
| MainClamp | Rotational Freedom (blue, active) | `main_clamp_blue_roll` | revolute, about the cane axis | ±40° (part name) |
| MainClamp | Rotational Freedom (yellow, passive) | `main_clamp_yellow_roll` | mimic of `main_clamp_blue_roll`, not planned | ±40° |

Changes from the earlier model:

- The "Linear Moving Clamp" is a joint. In the CAD the active gripper slides along the cane, away from the static gripper. It does not close across the cane. Earlier it was gripper state.
- The MainClamp has two slides (one per gripper). One motor drives both. The yellow slide follows the blue slide with the opposite sign.
- The roll range is ±40°, not ±180°.
- The yellow roll is passive. It follows the blue roll.
- Clamp open/close is not in the CAD. It stays gripper state.

Possible reading of "9": 12 joints − 3 linear moving clamps. Lassie must confirm. With the two passive joints, the model plans 10.

## Frames

- World: z up. The beam runs along x. The beam bottom is at z = 0. The beam centre is at x = y = 0.
- CAD to Motus: (x, y, z) CAD → (x, −z, y) Motus.
- Each yaw joint is on the beam top, on the station axis (x = −375, 0, +375 LU).
- Each lift puts its carriage frame on the cane centreline (z = 242 LU at lift 0).
- Clamp frames: `handler_N_clamp` (static), `handler_N_active`, `main_clamp_blue`, `main_clamp_yellow`. Each origin is on the cane centreline, at the gripper centre. Each +x is along the cane.
- The handler grippers are off the yaw axis: the static gripper 50.4 LU toward the beam centre, the active gripper 50.4 LU toward the beam end.
- The roll joints turn a gripper about its own +x. A roll does not move the cane axis.

## Kinematics

FK uses the existing `TreeForwardKinematics`.
IK is closed form, because each chain is yaw + lift (+ one slide along the cane):

- yaw = heading of the cane axis. The solver tries both headings. It uses the heading nearest to the seed first.
- lift = cane height − clamp height at lift 0.
- feed or slide = distance along the cane from the clamp at 0.
- The blue roll keeps the seed value. The cane line does not set a roll. The yellow roll follows the blue roll in FK. The blue slide is the slide drive. The yellow slide follows as −blue.

`TrySolveClamp` puts one clamp on a given point and axis. A solve of the yellow MainClamp gripper sets the blue slide drive, because the two slides are coupled.
`TrySolveCane` puts all six clamp frames on one straight cane line. Feeds and the blue MainClamp slide keep their seed values. The yellow slide follows as −blue.
A failure returns `false` and a reason. The seed is not changed.

Limits of this model:

- The clamps have no pitch joint. Thus the cane axis must be horizontal.
- At yaw 0, every clamp origin is above the beam centre line. Thus a cane that all three stations hold must be above that line, along x.

## Two outer clamps hold a rod

Handler_1 and Handler_2 can hold a rod without the MainClamp. `TrySolveRod` solves this case. Feeds and MainClamp joints keep their seed values.

- Plane: the vertical plane through the two yaw axes (the beam plane, y = 0).
- In-plane motion: the two lifts set the rod height and the rod tilt. The rod can also move along its own axis.
- Yaw: each clamp +x points along the rod heading. In this plane the heading is 0 or 180°.
- Each static clamp origin is on the rod axis.

Closed-chain constraint:

- The beam fixes the horizontal distance `d` between the two static clamps (649.2 LU).
- The distance between the clamps along the rod is `d / cos(tilt)`. It changes when the tilt changes.
- Thus the rod must slide in at least one clamp when the tilt changes.
- A tilted rod also needs a passive pitch at each clamp. The URDF has no pitch joint.
- The active gripper shares the lift of its static gripper. With a tilt, it misses the rod by tan(tilt) × 100.8 LU.
- The tilt is limited by the lift range: about ±8° at 90 LU of lift.

## Dimensions

All values are from the CAD, in CAD length units (LU). `LU = 0.001` m (the STEP header says mm).
The CAD part names say "LengthUnits". If 1 LU is not 1 mm, change `LU` only.

| Name | Value (LU) | Source |
|------|------------|--------|
| `cane_d` | 60 | Bamboo `60LU_Dia`; gripper cradle radius 30 |
| `beam` | 1000 × 220 × 80 | Backbone |
| `station_x` | ±375 | plate and shaft centres |
| `cane_z` | 242 | cane centreline (beam top 80 + 162) |
| `static_dx`, `active_dx` | 50.4 | gripper centres from the handler axis |
| `feed` | 95 | ActiveGriper "95 LegthUnits Extention" |
| `main_dx` | ±170 | MainClamp gripper centres |
| `main_slide` | 110 | plate half-length 230 − gripper width 120 (inferred) |
| `lift` | 90 (handlers), 95 (MainClamp) | shaft length below the beam (inferred) |
| `roll_deg` | ±40° | Axial Rotation_+-40Deg |

Geometry notes:

- Visuals are the CAD meshes. Collisions are the part bounding boxes in the link frame.
- A plate box includes its shaft, so it overlaps the beam at q = 0 (adjacent links).

## Open questions

- Is the robot 9 DOF? The model plans 10 DOF. Which 1 more joint does the title leave out?
- Which roll gripper has the drive: blue or yellow? The model assumes blue.
- The two MainClamp slides are one coupled drive (Lassie, 2026-11-18). The yellow slide mimics the blue slide with multiplier −1.
- The cane diameter comes from the CAD rod: 60 mm (Lassie, 2026-11-18). The 150 mm figure is dropped.
- What are the real lift, feed and MainClamp slide ranges and directions?
- How does a gripper close on the cane? The CAD shows open cradles only.
- Can a clamp tilt (pitch)? If not, the two-clamp rod cannot tilt.
