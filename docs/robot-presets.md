# Robot Presets

Presets live under `resources/robots/` as JSON files:

```
resources/robots/UR/     UR3e, UR5e, UR10e, UR16e, UR20, UR30
resources/robots/KUKA/   KR 6 R900, KR 10 R1100, … LBR iiwa 7/14
```

## JSON schema (summary)

| Field | Type | Notes |
|-------|------|-------|
| `manufacturer` | string | `UniversalRobots` or `Kuka` |
| `modelName` | string | Lookup key |
| `family` | string | e.g. `e-series` |
| `axisCount` | int | Must match `jointLimits` length |
| `jointLimits[]` | array | `minRadians`, `maxRadians`, optional velocity/acceleration |
| `reachMeters` | number? | Approximate reach |
| `payloadKg` | number? | Nominal payload |
| `baseFrame` | object | x,y,z + quaternion (qw,qx,qy,qz) in meters |
| `toolFrame` | object | Same + optional `name` |
| `collisionLinks[]` | array? | Per-link capsule/box/sphere/mesh for `RobotMeshCollisionChecker` |
| `toolCollision` | object? | Static gripper volume in TCP-local frame (`shape`, `halfX`/`radius`/etc.) |
| `notes` | string? | Human notes |
| `sourceNote` | string? | Verification reference |
| `disclaimer` | string? | Defaults to planning-only disclaimer |

## Loading

```csharp
var robot = PresetLoader.LoadRobotModelByName("UR5e");
var preset = robot.Preset;
```

Custom paths: `PresetLoader.LoadRobotModelFromFile(path)`.

## Important

**Presets are planning and visualization defaults only.** They do not guarantee physical robot compatibility. Before real use, verify joint limits, TCP, base frame, payload, safety settings, mastering/calibration, controller configuration, and cell-specific constraints.
