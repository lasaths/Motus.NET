# URDF import

Motus.NET loads **serial revolute and prismatic** chains from URDF without ROS or xacro at runtime.

## Workflow

1. Expand xacro offline (if needed):

```bash
xacro robot.urdf.xacro > robot.urdf
```

2. Load in C#:

```csharp
var robot = UrdfRobotLoader.Load("robot.urdf", new UrdfLoadOptions
{
    BaseLink = "base_link",
    TipLink = "tool0",
    ModelName = "my_arm"
});
var model = robot.ToModel(); // includes CollisionModel when URDF defines <collision>
```

## Collision geometry

Per-link `<collision>` elements are imported into `RobotCollisionModel`:

- `box`, `cylinder` (as capsule), `sphere`, `mesh` (STL)

Mesh paths are resolved relative to the URDF file directory.

## SRDF allowed pairs

Use `SrdfLoader` to merge disable-collision pairs into a planning scene:

```csharp
var pairs = SrdfLoader.LoadAllowedPairs("cell.srdf");
var scene = SrdfLoader.MergeAllowedPairs(obstacleScene, pairs, linkNameToIndex);
```

Pair names can be robot link names or `CollisionBodies.RobotLink(index)` when mapped.

## Limits

- Serial chains only (no closed loops)
- No mimic joints
- xacro is not evaluated in-process — preprocess first
