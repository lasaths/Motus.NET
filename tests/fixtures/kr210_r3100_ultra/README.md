# KR 210 R3100 ultra (Quantec)

Kinematics in `kr210_r3100_ultra_minimal.urdf` are taken from KUKA’s **kr210_r3100_ultra** xacro in the official ROS 2 robot descriptions repo.

## Official source

- Repository: [kroshu/kuka_robot_descriptions](https://github.com/kroshu/kuka_robot_descriptions)
- Package: `kuka_quantec_support`
- Macro: `urdf/kr210_r3100_ultra_macro.xacro`
- License: Apache-2.0

KUKA ship **xacro**, not a flat URDF. To generate the full model with meshes (RViz, Gazebo, MoveIt):

```bash
git clone https://github.com/kroshu/kuka_robot_descriptions.git
cd kuka_robot_descriptions
# ROS 2 + xacro required
xacro kuka_quantec_support/urdf/kr210_r3100_ultra.urdf.xacro prefix:="" mode:=mock > kr210_r3100_ultra.urdf
```

Or visualize:

```bash
ros2 launch kuka_quantec_support view_robot.launch.py robot_model:=kr210_r3100_ultra robot_family:=quantec
```

## Files in this folder

| File | Purpose |
|------|---------|
| `kr210_r3100_ultra.urdf` | Full model with meshes from [kuka_robot_descriptions](https://github.com/kroshu/kuka_robot_descriptions) |
| `kr210_r3100_ultra_minimal.urdf` | Kinematics-only chain (Motus import / CI) |
| `README.md` | This file |

## Motus usage

**Grasshopper:** wire the path into **Motus Load URDF** (`BaseLink` = `base_link`, `TipLink` = `tool0`).

**.NET API (full model with meshes):**

```csharp
var urdf = UrdfRobotLoader.Load("kr210_r3100_ultra.urdf", new UrdfLoadOptions
{
    BaseLink = "base_link",
    TipLink = "tool0",
    ModelName = "KR 210 R3100 ultra"
});
```

Run `npm run fetch-assets` in `Motus.NET/tools/urdf-viewer` to download visual meshes (~9 MB).

**Lightweight import (no meshes):** use `kr210_r3100_ultra_minimal.urdf`.
