# Motus.Viewer — Blazor WebAssembly Bamboo Arm Viewer

**This is the sole Motus bamboo viewer.** The standalone HTML preview under
`/workspace/bamboo-viewer` (and any sibling Grasshopper PR HTML assets) is
**retired as a product viewer** — kept only as a visual/UX reference. Do not
treat that folder as the shipping UI.

A Blazor WebAssembly app that runs Motus.NET kinematics, collision checking, and
task validation in the browser. The `/bamboo` page ports the look and UX of the
old ICD/LIS HTML preview (pole hang, parallel jaws, θ axes, task cards, scrub)
while Motus.NET remains the source of truth for joint state, tasks, and collision.

## Architecture

```
Browser (WebAssembly)
├── Blazor UI (BambooViewer.razor)     # sliders, tasks, play/scrub
├── Motus.NET C# (WASM)
│   ├── UrdfRobotLoader / joint limits
│   ├── RobotMeshCollisionChecker      # collision decisions
│   └── BambooIcdMotion (viewer)       # ICD planar IK + motion keys in C#
└── Three.js (js/viewer.js)            # draw only: pole, strut, jaws, axes
```

**Key principle:** Three.js only draws. Joint angles, gripper openings, strut
hold, task frames, and collision status are computed/owned in C# and pushed via
`setBambooPose` / `setBambooFrames`.

## Frame convention

| Layer | Up axis | Notes |
|-------|---------|-------|
| Three.js viewer | **Y-up** | Matches retired HTML preview (pole at `y = 0.40`, segments along local −Y) |
| Motus.NET URDF | **Z-up** | ROS convention; shared θ vector with the viewer (degrees in UI, radians in Motus) |

Conversion for task plane positions stored on `TaskInstance`: viewer `(x, y, z)` →
Motus-ish `(x, −z, y)`. Documented in `Services/BambooIcdMotion.cs`.

## Features (ported from HTML)

- Hang from horizontal bamboo pole; arm chain; strut pick / stand upright
- Linear parallel-jaw grippers (L/R mm); left jaw closed on pole during task
- Visible joint rotation axes + labels θ1–θ5; jaw travel axes
- Per-joint sliders in degrees within published limits; play / pause / reset; motion scrub
- Tasks: string identity pick/place, editable plane `yMm` nudges, jawMm, duplicate/delete
  (cannot delete the last pick or last place)
- Collision: Motus.NET `RobotMeshCollisionChecker` decides; play stops on hit; Three.js tints

## Run

```bash
cd src/Motus.Viewer
dotnet run --urls http://127.0.0.1:5268
```

Open `http://127.0.0.1:5268/bamboo`.

## What still differs from the HTML preview

- Motus URDF mesh envelopes are a floor-serial approximation of the ICD lengths;
  visual meshes match the HTML preview. Collision part names are coarse when Motus
  reports a hit (no Motus API for named OBB pairs yet).
- ICD planar IK for play lives in `BambooIcdMotion` (C#), not Motus.Geometry’s
  generic numerical IK — Motus still owns collision and `TaskInstance` / `TaskPath`.
- Blazor template Counter/Weather pages were removed; Bootstrap assets may remain unused.

## References

- Retired visual reference: `/workspace/bamboo-viewer/next.html`
- Motus.NET, Three.js r128, Blazor WebAssembly
