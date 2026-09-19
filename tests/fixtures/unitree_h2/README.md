# Unitree H2 (experimental Motus fixture)

Meshless kinematic tree for **Unitree H2** — not H1-2 (`h1_2_description`).

| File | Purpose |
|------|---------|
| `h2_minimal.urdf` | Pelvis-rooted joints/links only (~31 revolute). CI / LoadTree smoke. |
| `LICENSE` | Upstream BSD-3-Clause from [unitree_ros](https://github.com/unitreerobotics/unitree_ros). |

## Source

- Official: [`unitree_ros/robots/h2_description/H2.urdf`](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/h2_description/H2.urdf)
- Product: [unitree.com/H2](https://www.unitree.com/H2)

## Motus adaptations

- Visual / collision / inertial / material / mujoco stripped.
- Floating base (`floating_base_joint` world→pelvis) omitted — Motus rejects `type="floating"`. Pelvis is the tree root (fixed-base equivalent).

## Experimental honesty

**In scope:** `LoadTree` + Tree FK scrub; optional one-arm `PlanningGroup` (legs/waist locked at start).

**Out of scope:** biped walk, balance, ZMP/WBC, Unitree SDK locomotion, `Family=legged` Walk, GA `Family=humanoid`.

Prefer `H2.urdf` over `H2_loop` for Motus (loop rods need MuJoCo equality, not URDF).
