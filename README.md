# Motus.NET

.NET 8 motion-planning library for serial robots: plan, validate, export. No UI, no vendor runtime, no live robot I/O.

Use it from [Motus.Grasshopper](https://github.com/lasaths/Motus.Grasshopper) or any .NET host. Bundled presets (UR family) and **URDF / xacro** loaders; analytic IK for Universal Robots, numerical IK for generic chains.

Licensed under [MIT](LICENSE).

## Packages

```bash
dotnet add package Motus.Core
dotnet add package Motus.Geometry
dotnet add package Motus.OMPL.NET
dotnet add package Motus.Presets
```

| Package | Role |
|---------|------|
| `Motus.Core` | Models, `IPlanner`, validation, `PlanningContext`, JSON/CSV export |
| `Motus.Geometry` | FK/IK, mesh/sphere collision, LIN / industrial motion programs |
| `Motus.OMPL.NET` | Sampling planners (managed RRT-Connect by default; native OMPL when built) |
| `Motus.Presets` | JSON presets + `UrdfRobotLoader` / xacro |
| `Motus.Native` | Optional P/Invoke to `motus_native` (OMPL + FCL) |

**Rhino / desktop:** managed planning and C# mesh collision are the default — no Linux `.so` required. See [docs/rhino-host.md](docs/rhino-host.md).

## Quick start

```csharp
var robot = PresetLoader.LoadRobotModelByName("UR10e");
var start = HomePoseResolver.HomeOrZeros(robot);
var goal = new JointState(/* radians */);

var scene = new CollisionScene(new[] {
    CollisionObject.Sphere("obstacle", Frame.Identity, 0.1)
});
var checker = CollisionCheckerFactory.Create(robot);
var opts = new PlanningOptions {
    CollisionScene = scene,
    CollisionChecker = checker
};

var result = SamplingPlanner.Create(checker, new SamplingPlannerOptions {
    PlannerId = SamplingPlannerId.RrtConnect,
    MaxIterations = 4000
}).Plan(new PlanningRequest(robot, start, goal, opts));

if (result.Success)
    File.WriteAllText("plan.json", TrajectoryExport.ToJson(result.Trajectory!));
```

URDF:

```csharp
var robot = UrdfRobotLoader.Load("robot.urdf", new UrdfLoadOptions {
    BaseLink = "base_link",
    TipLink = "tool0"
});
var fk = KinematicsResolver.CreateFkSolver(robot.Preset, robot.Chain);
```

## Planners

| Planner | When to use |
|---------|-------------|
| `JointLinearPlanner` | Free-space joint interpolation |
| `CartesianLinearPathPlanner` | True TCP-linear (LIN) after IK |
| `SamplingPlanner` | Obstacles — RRT-Connect / RRT* / … via registry |
| `IndustrialMotionPlanner` | Mixed PTP / LIN / CIRC programs |

Prefer `SamplingPlanner` + `CollisionCheckerFactory.Create` (or `GetOrCreate` for session reuse) over the obsolete `RrtConnectPlanner` wrapper.

```csharp
// Motion program
var planner = new IndustrialMotionPlanner(robot.Preset);
var result = planner.Plan(new MotionProgramRequest(robot, start, new MotionSegment[] {
    new PtpSegment(ptpGoal, blendRadiusMeters: 0.004),
    new LinSegment(linGoal, stepMeters: 0.005),
    new CircSegment(via, circGoal, arcSamples: 12)
}));
```

## Context: attach, groups, SRDF

```csharp
var ctx = PlanningContext.Create(robot, scene)
    .Attach("workpiece", workpieceBox, tcpLocalFrame)
    .ForGroup(SrdfLoader.LoadGroups("robot.srdf")[0]);

var checker = CollisionCheckerFactory.Create(robot, attached: ctx.Attached);
var result = SamplingPlanner.Create(checker).Plan(
    new PlanningRequest(robot, start, goal, ctx.ToPlanningOptions()));
```

Non-group joints stay locked at the start configuration. `MotusCapabilities.Describe()` reports managed vs native OMPL/FCL.

## Units & export

Internal units: **radians**, **seconds**, **meters** (`Motus.Core.Units` for °↔rad).

`TrajectoryExport.ToJson` emits the **PlanBundle** contract for Grasshopper and control adapters:

- `contractVersion` / `exportVersion` — schema compatibility
- `units`, `frameConvention` — semantics
- `diagnostics` — machine-readable planner messages
- `provenance` — optional `plannerId`, seed, settings hash

Additive optional fields are safe; removing/renaming fields requires a major `contractVersion` bump.

## Build & test

Requires .NET 9+ SDK to open the `.slnx`; libraries target `net8.0`.

```bash
dotnet build Motus.NET.slnx
dotnet test Motus.NET.slnx
```

Optional native library:

```bash
cmake -S native -B native/build -DMOTUS_USE_OMPL=ON -DMOTUS_USE_FCL=ON
cmake --build native/build
# Linux: export LD_LIBRARY_PATH=native/build
```

Benchmarks: `benchmarks/Motus.Benchmarks` (BenchmarkDotNet).

## Releases

Tag `vX.Y.Z` → [release workflow](.github/workflows/release.yml) builds, tests, packs, publishes to [nuget.org](https://www.nuget.org/profiles/lasaths), and creates a GitHub Release.

One-time nuget.org trusted publisher: GitHub owner `lasaths`, repo `Motus.NET`, workflow `release.yml`.

Changelog: [CHANGELOG.md](CHANGELOG.md).

## Safety

Motus does **not** command physical robots. Presets and URDF imports are planning defaults — verify limits and calibration before hardware use. See [docs/safety.md](docs/safety.md).

This library and bundled presets were developed with AI assistance; values are approximate and must be checked against manufacturer data.

## Attribution

- Managed RRT-Connect implements Kuffner & LaValle (ICRA 2000). Native path uses [OMPL](https://ompl.kavrakilab.org/) (BSD-3-Clause) when built.
- Robot presets are approximate public-datasheet values for planning/visualization only.
