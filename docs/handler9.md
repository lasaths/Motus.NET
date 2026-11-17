# 9-DOF bamboo handler (Experimental)

Status: **Experimental**. This robot is not in the Supported list. Promotion needs an ADR, a METHODS entry and a Rhino matrix proof.

Source: Lassie's slide "9 Degrees of Freedom".
Files:

- URDF: `tests/fixtures/handler9/handler9.urdf`. Do not edit it by hand.
- Generator: `tools/handler9/gen_handler9_urdf.py`. All dimensions are in `DIMS` at the top.
- Kinematics: `src/Motus.Construction/Bamboo/Handler9Kinematics.cs`.
- Tests: `tests/Motus.Core.Tests/Handler9KinematicsTests.cs`.

Three clamp stations hold one cane: Handler_1 and Handler_2 at the ends, MainClamp in the middle. The clamps take round profiles.

This robot is not `bamboo_icd`. `bamboo_icd` is a 5R arm that hangs under a pole and walks with two grips. This robot is a fixed beam with three stations.

## DOF interpretation

The slide shows 12 labels. The title says 9 DOF.
We count a clamp open/close as gripper state, not as a joint.
Each station has one clamp open/close (the Linear Moving Clamp against the Stationary Clamp).
Thus 12 labels − 3 clamp open/close = 9 joints.

| Station | Slide label | Model | Joint | Limits |
|---------|-------------|-------|-------|--------|
| Handler_1 | Rotation about the vertical axis | revolute, z | `handler_1_yaw` | −180°..+180° |
| Handler_1 | Axial linear vertical movement | prismatic, z | `handler_1_lift` | placeholder |
| Handler_1 | Gripper (moving + stationary clamp) | gripper state | none | — |
| MainClamp | Rotation about the vertical axis | revolute, z | `main_clamp_yaw` | −180°..+180° |
| MainClamp | Axial linear vertical movement | prismatic, z | `main_clamp_lift` | placeholder |
| MainClamp | Linear Movement | prismatic, along the cane (local x) | `main_clamp_slide` | placeholder |
| MainClamp | Rotational Freedom (blue clamp) | revolute, about the cane axis | `main_clamp_blue_roll` | ±180° (placeholder) |
| MainClamp | Rotational Freedom (yellow clamp) | revolute, about the cane axis | `main_clamp_yellow_roll` | ±180° (placeholder) |
| MainClamp | Linear Moving Clamp | gripper state | none | — |
| Handler_2 | Same as Handler_1 | | `handler_2_yaw`, `handler_2_lift` | as Handler_1 |

Joint chain per station: beam → yaw → lift → clamp.
The MainClamp chain is: beam → yaw → lift → slide → two roll clamps (two branches).

## Frames

- World: z up. The beam runs along x. The beam bottom is at z = 0.
- Each yaw axis is on the beam top, at the station x position.
- Clamp frames: `handler_1_clamp`, `main_clamp_blue`, `main_clamp_yellow`, `handler_2_clamp`.
- A clamp frame origin is on the cane centreline. Its +x is along the cane.
- The cane diameter does not move the clamp frames. It only sizes the jaws.
- At yaw = 0, the clamp +x is world +x (along the beam).
- The roll joints turn the clamp about its own +x. A roll does not move the cane axis.

## Kinematics

FK uses the existing `TreeForwardKinematics`.
IK is closed form, because each chain is yaw + lift (+ slide):

- yaw = heading of the cane axis. The solver tries both headings (the cane has no direction). It uses the heading nearest to the seed first.
- lift = cane height − clamp height at lift 0.
- slide (MainClamp only) = distance along the cane from the clamp at slide 0.
- Rolls keep the seed value. The cane line does not set a roll.

`TrySolveClamp` puts one clamp on a given point and axis.
`TrySolveCane` puts all four clamp frames on one straight cane line. The grip position along the cane is free. The MainClamp slide keeps its seed value.
A failure returns `false` and a reason. The seed is not changed.

Limits of this model:

- The clamps have no pitch joint. Thus the cane axis must be horizontal.
- At yaw 0, every clamp origin is above the beam centre line. Thus a cane that all three stations hold must be above that line, along x.

## Placeholder dimensions

All values are estimates from the slide proportions. They are approximate. Change them in `DIMS`, then run the generator.

| Name | Value | Meaning |
|------|-------|---------|
| `cane_d` | 0.150 m | Cane diameter parameter. The default is a placeholder: the clamps take round profiles up to about 150 mm, and the exact size is not known. The jaws close on this diameter. |
| `beam` | 4.0 × 0.30 × 0.30 m | Base beam length, width, height |
| `station_x` | −1.80, 0, +1.80 m | Handler_1, MainClamp, Handler_2 positions along the beam |
| `column_h` | 0.60 m | Beam top to cane axis at lift 0 |
| `lift` | 0 .. 0.40 m | Vertical movement range |
| `slide` | −0.25 .. +0.25 m | MainClamp movement along the cane |
| `clamp_dx` | 0.15 m | Blue and yellow clamp offset from the MainClamp centre |
| `jaw` | 0.10 × 0.03 × 0.20 m | Jaw plate size |

## Open questions

- Is the MainClamp "Linear Movement" along the cane or across the beam?
- Do the blue and yellow clamps turn independently? What are their limits?
- Can a clamp tilt (pitch)? If not, the cane must stay horizontal.
- Is a clamp centre on its yaw axis, or offset from it?
- What are the real dimensions and stroke lengths?
- What is the real cane diameter range (minimum and maximum)?
