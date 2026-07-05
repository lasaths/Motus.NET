# Rhino 8 / Grasshopper host guide

Motus.NET targets **Rhino 8 on Windows and macOS** with no Linux-only runtime requirements. Planning, attach, collision, and URDF import run in **managed .NET**; native OMPL/FCL are optional accelerators used only when a full `motus_native` is present and reports available.

## Requirements

| Host | .NET | Notes |
|------|------|--------|
| Rhino 8.20+ Win/Mac | `net8.0` | Matches Motus package TFM |
| Rhino 8.10–8.19 | `net7.0` | Multi-target your `.gha` or upgrade Rhino |

Reference packages: `Motus.Core`, `Motus.Geometry`, `Motus.OMPL.NET`, `Motus.Presets` (and transitively `Motus.Native`).

## What works without native OMPL/FCL

- `JointLinearPlanner`, `CartesianLinearPathPlanner`
- `RrtConnectPlanner` — **managed RRT-Connect** (default on desktop)
- `RobotMeshCollisionChecker`, attach at TCP, `PlanningContext`
- Bundled JSON presets (`resources/robots` copied to output via `Motus.Presets`)
- URDF load via `UrdfRobotLoader`

Probe at runtime:

```csharp
MotusCapabilities.Describe();
// e.g. "managed RRT-Connect, C# mesh collision, attach supported"
```

## Native library (`motus_native`)

The `Motus.Native` NuGet ships a **per-RID stub** (`runtimes/{rid}/native/`) so P/Invoke resolves on Win/Mac without `LD_LIBRARY_PATH`. Stubs return *unavailable* for OMPL/FCL → managed code paths activate automatically.

`buildTransitive/Motus.Native.targets` copies the stub into your plugin output folder (`runtimes/win-x64/native/`, `osx-arm64/native/`, etc.).

Build stubs locally:

```powershell
.\scripts\build-native-stub.ps1          # Windows
./scripts/build-native-stub.sh         # macOS / Linux
```

Full OMPL+FCL native builds are **CI/Linux optional** today; not required for Rhino.

## Grasshopper packaging checklist

1. Target `net8.0` (or `net8.0-windows` + `net8.0` multi-target per McNeel templates).
2. Reference Motus NuGet packages; ensure `resources/robots` lands beside your `.gha` (automatic with `Motus.Presets` package).
3. Deploy all Motus DLLs + `runtimes/**/native/motus_native.*` from build output into the Yak package.
4. Do **not** set `LD_LIBRARY_PATH` or ship Linux `.so` files on Windows/Mac.

## Preset / resource paths

`PresetLoader` resolves bundled robots from:

1. Directory containing `Motus.Presets.dll` with `resources/robots`
2. `AppContext.BaseDirectory` (Rhino plugin folder)

Copy custom URDF/meshes next to the `.gha` or pass absolute paths to `UrdfRobotLoader`.
