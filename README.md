# Motus.NET

Standalone .NET 8 robotics motion-planning library for the Motus toolkit. Planning, validation, and export only — no UI, no vendor runtime, and no connection to physical robots.

`Motus.OMPL.NET` is the **OMPL adapter layer**: it targets a native [OMPL](https://ompl.kavrakilab.org/) C++ binding via `Motus.OMPL.Native` (see [docs/ompl-port-plan.md](docs/ompl-port-plan.md)). Until the native library is built with `MOTUS_USE_OMPL=ON`, a managed RRT-Connect fallback is used.

Arbitrary serial manipulators can be loaded from **URDF** (`UrdfRobotLoader`) or bundled JSON presets. FK/IK use a generic numerical solver on the joint chain — no per-vendor IK branches.

Licensed under [MIT](LICENSE).

## Install

```bash
dotnet add package Motus.Core
dotnet add package Motus.Geometry
dotnet add package Motus.OMPL.NET
dotnet add package Motus.OMPL.Native
dotnet add package Motus.Presets
```

| Package | Purpose |
|---------|---------|
| `Motus.Core` | Data model, `IPlanner`, validation, JSON/CSV export |
| `Motus.Geometry` | Serial-chain FK, numerical IK, sphere collision, Cartesian planners |
| `Motus.OMPL.NET` | RRT-Connect (native OMPL when built, managed fallback) |
| `Motus.Presets` | JSON presets + `UrdfRobotLoader` |

## Load a robot

```csharp
// Bundled preset
var preset = PresetLoader.LoadByModelName("UR5e");

// URDF (revolute serial chain)
var robot = UrdfRobotLoader.Load("path/to/robot.urdf", new UrdfLoadOptions {
    BaseLink = "base_link",
    TipLink = "tool0"
});
var fk = KinematicsResolver.CreateFkSolver(robot.Preset, robot.Chain);
```

## Planners

| Planner | Use when | Package |
|---------|----------|---------|
| `JointLinearPlanner` | Known start/goal joint configs, no obstacles | `Motus.Core` |
| `CartesianLinearPlanner` | Cartesian TCP goal; joint-linear path after IK | `Motus.Geometry` |
| `CartesianLinearPathPlanner` | True TCP-linear (LIN) motion | `Motus.Geometry` |
| `RrtConnectPlanner` | Obstacles in `PlanningOptions.CollisionScene` | `Motus.OMPL.NET` |

## Units

Internal units are **radians**, **seconds**, and **meters**. Use `Motus.Core.Units` for degree/radian conversion.

## Build & test

```bash
dotnet build Motus.NET.slnx
dotnet test Motus.NET.slnx
```

Native OMPL (optional):

```bash
cmake -S native -B native/build -DMOTUS_USE_OMPL=ON
cmake --build native/build
```

Requires the .NET 9 SDK or newer to read the `.slnx` solution; the libraries target `net8.0`.

## Releases

Push a version tag (`v0.3.1`) to run [`.github/workflows/release.yml`](.github/workflows/release.yml): build, test, pack, publish to [nuget.org](https://www.nuget.org/profiles/lasaths) via [trusted publishing](https://learn.microsoft.com/en-us/nuget/nuget-org/trusted-publishing), and create a GitHub Release with `.nupkg` assets.

Configure the nuget.org trusted publisher policy (one-time):

1. [nuget.org](https://www.nuget.org) → **Account** → **Trusted Publishing** → **Add**
2. Provider: **GitHub**, owner `lasaths`, repository `Motus.NET`, workflow filename **`release.yml`** (exact match)
3. Re-run the failed **Release** workflow on the tag after the policy is saved (or push the next tag)

## Safety

Motus does **not** send commands to physical robots. Robot presets and URDF imports are planning/visualization defaults — verify all limits and calibration before any hardware use. See [docs/safety.md](docs/safety.md).

## AI-assistance disclaimer

This library and its bundled robot presets were developed with AI assistance. All values are approximate and must be independently verified against official manufacturer datasheets and the real controller before any physical use.

## Attribution

- Managed RRT-Connect fallback implements the RRT-Connect algorithm (Kuffner & LaValle, ICRA 2000). Native planning uses [OMPL](https://ompl.kavrakilab.org/) (BSD-3-Clause) when built.
- Robot presets are approximate values from public datasheets, for planning/visualization only.
