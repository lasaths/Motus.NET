# Unitree Go2 (experimental Motus fixture)

Meshless kinematic tree for **Unitree Go2** mammal quadruped — distinct from insectoid `Family=legged` / `QuadSmoke` Walk.

| File | Purpose |
|------|---------|
| `go2_minimal.urdf` | Base-rooted 12R + foot frames. CI / LoadTree + FK stance smoke. |
| `LICENSE` | Upstream BSD-3-Clause from [unitree_ros](https://github.com/unitreerobotics/unitree_ros). |

## Source

- Official: [`unitree_ros/robots/go2_description`](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2_description) (`urdf/go2_description.urdf`)
- Product: [unitree.com/go2](https://www.unitree.com/go2)

## Motus adaptations

- Visual / collision / inertial / material stripped.
- Sensors, rotors, and head geometry links dropped; foot frames kept for stance FK.
- Upstream URDF already uses `base` as root (no `floating` joint) — fixed-base tree FK; no float→fixed rewrite required for this description.

## Experimental honesty

**In scope:** `LoadTree` + Tree FK scrub; standing/stance joint pose with foot heights roughly coplanar.

**Out of scope:** Motus Walk / `Family=legged` insectoid gait, Bretl–Lall wrench LP, Spot/ANYmal equivalence, Unitree SDK locomotion, GA `Family=quadruped`.
