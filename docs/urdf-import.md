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

## Visual verification

Use the dev viewer ([urdf-loaders](https://github.com/gkjohnson/urdf-loaders)) to inspect Motus URDF fixtures and scrub joint angles:

```bash
cd tools/urdf-viewer
npm install
npm run dev
```

Open `http://localhost:5173`, pick a fixture, and adjust joint sliders. Each fixture loads a **home pose** and **bundled demo path** from [`tests/fixtures/viewer_presets.json`](../tests/fixtures/viewer_presets.json); the TCP trace is drawn in blue. Press **Play trajectory** to animate the demo, or **Home pose** to return to the default.

Drop a Motus trajectory JSON export (`TrajectoryExport.ToJson`) onto the panel to replace the bundled path.

Automated FK cross-check against urdf-loader runs in CI via `UrdfFkCrossCheckTests` (requires Node 22+ and `npm ci` in `tools/urdf-viewer`).
