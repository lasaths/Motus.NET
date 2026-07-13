# Motus.NET

Standalone .NET 8 robotics motion-planning library for the Motus toolkit. Planning, validation, and export only — no UI, no vendor runtime, and no connection to physical robots.

`Motus.OMPL.NET` is the **OMPL adapter layer**: it targets unified native `motus_native` (OMPL + optional FCL) via `Motus.Native` (see [docs/ompl-port-plan.md](docs/ompl-port-plan.md)). Until the native library is built, managed RRT-Connect and C# mesh collision (including attach) are used.

Arbitrary serial manipulators can be loaded from **URDF** (`UrdfRobotLoader`) or bundled JSON presets. FK/IK use DH profiles on bundled presets (analytic IK for Universal Robots) and numerical IK on generic URDF chains.

Licensed under [MIT](LICENSE).

## Rhino 8 / Grasshopper

Windows and macOS hosts use **managed planning and collision by default** — no Linux `.so`, no `LD_LIBRARY_PATH`. See [docs/rhino-host.md](docs/rhino-host.md).

## Install

```bash
dotnet add package Motus.Core
dotnet add package Motus.Geometry
dotnet add package Motus.OMPL.NET
dotnet add package Motus.Native
dotnet add package Motus.OMPL.Native
dotnet add package Motus.Presets
```

| Package | Purpose |
|---------|---------|
| `Motus.Core` | Data model, `IPlanner`, validation, `PlanningContext`, JSON/CSV export |
| `Motus.Geometry` | FK/IK, collision, Cartesian planners, attach-aware checking |
| `Motus.OMPL.NET` | RRT-Connect / RRT* (native OMPL when built, managed fallback) |
| `Motus.Native` | P/Invoke to `motus_native` (OMPL + FCL C ABI) |
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
| `IndustrialMotionPlanner` | Mixed `PTP/LIN/CIRC` motion programs | `Motus.Geometry` |

## Motion Programs (0.6.0)

```csharp
var session = robot.WithTool(ToolDefinition.FromPreset(robot));
var planner = new IndustrialMotionPlanner(session.Preset);
var request = new MotionProgramRequest(
    session,
    start,
    new MotionSegment[]
    {
        new PtpSegment(ptpGoal, blendRadiusMeters: 0.004),
        new LinSegment(linGoal, stepMeters: 0.005, blendRadiusMeters: 0.003),
        new CircSegment(circVia, circGoal, arcSamples: 12)
    });

var result = planner.Plan(request);
```

- Segment model: `PtpSegment`, `LinSegment`, `CircSegment`
- Blend radii truncate TCP paths at corners when feasible; infeasible blends fall back to exact-stop with a warning
- Export includes motion metadata (`motionType`, `segmentIndex`, `blendRadiusMeters`) and optional `toolFrame` for session tools

## Xacro import (0.6.0)

```csharp
var robot = UrdfRobotLoader.LoadXacro("robot.urdf.xacro", new UrdfLoadOptions {
    BaseLink = "base_link",
    TipLink = "tool0"
});
```

Minimal in-process xacro: includes, properties, simple macros. No `$(find)` — use `XacroOptions.SearchPaths` or preprocess offline.

## Attach (pick-style)

```csharp
var ctx = PlanningContext.Create(robot, scene)
    .Attach("workpiece", workpieceBox, tcpLocalFrame);
var checker = CollisionCheckerFactory.Create(robot, attached: ctx.Attached);
var result = new RrtConnectPlanner(checker).Plan(
    new PlanningRequest(robot, start, goal, ctx.ToPlanningOptions()));
```

`MotusCapabilities.Describe()` reports native OMPL/FCL availability for hosts.

## Planning group (SRDF-style)

```csharp
// From SRDF (official MoveIt config, pairs with unmodified URDF)
var groups = SrdfLoader.LoadGroups("robot.srdf");
var ctx = PlanningContext.Create(robot, scene).ForGroup(groups[0]);
var result = new RrtConnectPlanner(checker).Plan(
    new PlanningRequest(robot, start, goal, ctx.ToPlanningOptions()));

// Or manual joint list (partial-DOF lock)
var group = new PlanningGroup("arm", "base_link", "tool0",
    ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2"]);
var ctx2 = PlanningContext.Create(robot, scene).ForGroup(group);
```

Non-group joints stay locked at the start configuration during planning.

**Native OMPL + FCL:** optional on Linux when `motus_native` is built with OMPL/FCL. Win/Mac NuGet ships stubs — managed RRT and C# mesh collision run by default (see [docs/rhino-host.md](docs/rhino-host.md)).

## Units

Internal units are **radians**, **seconds**, and **meters**. Use `Motus.Core.Units` for degree/radian conversion.

## PlanBundle contract

`TrajectoryExport.ToJson(...)` emits the stable planner contract consumed by adapter layers (for example `Motus.Grasshopper`) and downstream control plugins.

- `contractVersion` tracks payload schema compatibility (current `1.0.0`)
- `exportVersion` tracks serializer implementation version (current `1`)
- `units` and `frameConvention` lock radians/seconds/meters and joint ordering semantics
- `diagnostics` carries machine-readable planner messages (`code`, `severity`, `message`)
- `provenance` optionally carries reproducibility metadata (`plannerId`, `randomSeed`, `settingsHash`, `retimeAlgorithm`)

Compatibility policy:

- Patch/minor updates preserve backward compatibility for existing required fields
- New optional fields are additive and safe
- Any removal/rename/type change of existing fields requires a `contractVersion` major bump

## 0.5.0 Migration Notes

See `CHANGELOG.md` (`Unreleased`) for upgrade details.

## Build & test

```bash
dotnet build Motus.NET.slnx
dotnet test Motus.NET.slnx
```

Native `motus_native` (optional):

```bash
cmake -S native -B native/build -DMOTUS_USE_OMPL=ON -DMOTUS_USE_FCL=ON
cmake --build native/build
# Linux: export LD_LIBRARY_PATH=native/build
```

Requires the .NET 9 SDK or newer to read the `.slnx` solution; the libraries target `net8.0`.

## Releases

Push a version tag (`v0.5.0`) to run [`.github/workflows/release.yml`](.github/workflows/release.yml): build, test, pack, publish to [nuget.org](https://www.nuget.org/profiles/lasaths) via [trusted publishing](https://learn.microsoft.com/en-us/nuget/nuget-org/trusted-publishing), and create a GitHub Release with `.nupkg` assets.

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
