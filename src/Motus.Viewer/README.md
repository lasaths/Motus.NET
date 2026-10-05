# Motus.Viewer — Blazor WebAssembly Bamboo Arm Viewer

A Blazor WebAssembly application that runs Motus.NET kinematics, collision checking, and task validation **entirely in the browser**. Demonstrates the Bamboo ICD 5-DoF robotic arm with real-time visualization via Three.js and C#-powered motion planning.

## Features

- **C# kinematics in WASM**: Forward kinematics computed by Motus.NET running in the browser
- **Collision detection**: Real-time collision checking using `MeshCollisionChecker` and `SphereCollisionChecker`
- **Task validation**: Pick-and-place task path validation with `TaskPathValidator`
- **Three.js rendering**: 3D visualization with interactive joint controls
- **No JavaScript reimplementation**: All robot logic (FK, IK, collision) comes from the C# library

## Architecture

```
Browser (WebAssembly)
├── Blazor UI (BambooViewer.razor)
├── Motus.NET C# Libraries
│   ├── Motus.Core (models, validation, tasks)
│   ├── Motus.Geometry (FK/IK, collision)
│   └── Motus.Presets (URDF loading)
└── Three.js (rendering only)
    └── JS interop: updateRobot(linkPoses)
```

**Key principle**: Three.js **only renders**. Robot state (joint angles, link poses, collision status) is computed by Motus.NET C# and passed to JavaScript for display.

## Running Locally

### Prerequisites

- .NET 8 SDK or later
- Modern browser with WebAssembly support

### Build and run

```bash
cd src/Motus.Viewer
dotnet run
```

Then open `http://localhost:5000` (or the port shown in the terminal).

### Development server with hot reload

```bash
dotnet watch
```

## Project Structure

```
src/Motus.Viewer/
├── Pages/
│   ├── BambooViewer.razor       # Main viewer component
│   └── Home.razor               # Redirect to /bamboo
├── wwwroot/
│   ├── css/
│   │   └── viewer.css           # Dark-mode UI styling
│   ├── js/
│   │   └── viewer.js            # Three.js scene + JS interop
│   ├── fixtures/
│   │   └── bamboo_icd/
│   │       └── bamboo_icd.urdf  # 5-DoF arm URDF model
│   └── index.html               # Loads Three.js + Blazor
├── Motus.Viewer.csproj          # References Motus.Core, Geometry, Presets
└── README.md                    # This file
```

## Bamboo ICD 5-DoF Arm

The Bamboo arm is a 5-degree-of-freedom serial manipulator based on RA-L 2022 specifications:

- **θ1**: wrist Z rotation [-170°, 170°]
- **θ2**: elbow X rotation [-90°, 90°]
- **θ3**: shoulder X rotation [-135°, 135°]
- **θ4**: elbow X rotation [-90°, 90°]
- **θ5**: wrist Z rotation [-170°, 170°]

The URDF model is loaded at runtime and forward kinematics is computed via `UrdfRobotLoader` and `SerialForwardKinematics`.

## Task Validation

The viewer demonstrates task-based planning:

1. **Pick**: Approach and grasp a strut from the ground
2. **Place**: Stand the strut upright and release

Tasks are validated using `TaskPathValidator` with:
- **IK reachability**: Can the arm reach each frame?
- **Collision checking**: Are all poses collision-free?
- **Joint limits**: Do solutions respect the URDF limits?

Failed validations report specific errors (e.g., "frame unreachable", "collides with ground").

## Controls

- **Joint sliders**: Manually adjust each joint angle
- **Reset**: Return to home position (all zeros)
- **Play Task**: Execute the pick-and-place path (if IK succeeds)
- **Collision status**: Real-time indicator shows green (free) or red (colliding)

## Testing

Bamboo arm tests validate that Motus.NET handles the arm correctly:

```bash
cd /workspace
dotnet test --filter "FullyQualifiedName~BambooArm"
```

Tests cover:
- URDF loading and joint limit parsing
- Forward kinematics at home position
- Collision checking (no self-collision)
- Task path validation (reachable frames pass, colliding frames fail)
- Mesh collision checker integration

## Deployment

To publish as static files for hosting:

```bash
dotnet publish -c Release -o publish
```

Output is in `publish/wwwroot/`. Host via any static file server (GitHub Pages, Netlify, etc.).

## Why Blazor WASM?

The user (Lassie) explicitly chose Blazor WebAssembly to ensure the viewer **tests Motus.NET itself**, not a JavaScript reimplementation. By running the C# library in the browser, we guarantee:

1. The viewer uses the same kinematics as the .NET library
2. Collision detection matches what .NET tests use
3. Task validation logic is identical to server-side planning
4. No risk of JS/C# drift

This makes the viewer a **live integration test** of Motus.NET, not just a visualization tool.

## References

- [Motus.NET GitHub](https://github.com/lasaths/Motus.NET)
- RA-L 2022: Bamboo ICD 5-DoF arm specifications
- [Three.js](https://threejs.org/)
- [Blazor WebAssembly](https://dotnet.microsoft.com/apps/aspnet/web-apps/blazor)
