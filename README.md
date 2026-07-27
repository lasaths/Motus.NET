# Motus.NET

.NET 8 **host-agnostic** motion-planning core: models, FK/IK, collision, sampling/Cartesian planners, retiming, and export.

| Motus.NET | [Motus.Grasshopper](https://github.com/lasaths/Motus.Grasshopper) |
|-----------|-------------------------------------------------------------------|
| Libraries on [nuget.org](https://www.nuget.org/profiles/lasaths) (`0.13.0`) | Rhino/GH UI that calls these APIs |
| Owns math, units, Status contracts, method DOIs | Thin components + examples |

No UI, no vendor runtime, no live robot I/O. Algorithm catalog: [docs/METHODS.md](docs/METHODS.md) · citations: [docs/REFERENCES.bib](docs/REFERENCES.bib).

Bundled presets (UR family) and **URDF / xacro** loaders; analytic IK for Universal Robots, numerical IK for generic chains. Parallel **Stewart** (`Family=stewart`, leg lengths in meters) and **legged** gait preview (`Family=legged`, radians) are first-class siblings of serial tip chains.

Licensed under [MIT](LICENSE).

## Contents

- [Packages](#packages)
- [Quick start](#quick-start)
- [Planners](#planners)
- [Algorithms and references](#algorithms-and-references)
- [Context: attach, groups, SRDF](#context-attach-groups-srdf)
- [Units and export](#units--export)
- [Build and test](#build--test)
- [Releases](#releases)
- [Safety](#safety)
- [Attribution](#attribution)

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

**Rhino / desktop:** managed planning and C# mesh collision are the default — no Linux `.so` required. See [docs/rhino-host.md](docs/rhino-host.md). Stub/NuGet builds often expose only `RrtConnect` in the planner registry; extra OMPL IDs need a full native build. Check `MotusCapabilities.Describe()`.

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
| `SamplingPlanner` | Obstacles — RRT-Connect / PRM* / … via registry |
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

## Algorithms and references

Canonical detail (API, units, failure codes): [docs/METHODS.md](docs/METHODS.md). BibTeX: [docs/REFERENCES.bib](docs/REFERENCES.bib).

<details>
<summary><strong>Method catalog</strong> — area, behavior, citations (click to expand)</summary>

| Area | What Motus does | References |
|------|-----------------|------------|
| URDF / xacro | In-process xacro subset → kinematic tree (`LoadTreeXacro`) | [docs/urdf-import.md](docs/urdf-import.md) |
| Joint LIN | Free-space joint interpolation + optional collision | Deterministic managed |
| Cartesian LIN | TCP-linear path, IK per sample | Deterministic managed |
| PoE FK / numerical IK | Modern Robotics screws + body Jacobian for URDF serial | Lynch & Park, *Modern Robotics* |
| TOTG retime | Managed TOPP-RA-style retiming (`RetimerAlgorithm.Totg`) | Pham & Pham 2018, DOI [10.1109/TRO.2018.2819195](https://doi.org/10.1109/TRO.2018.2819195) |
| Path constraints | MoveIt-shaped position/orientation checks | Sucan et al. 2012, DOI [10.1109/MRA.2012.2205651](https://doi.org/10.1109/MRA.2012.2205651) |
| RRT-Connect | Default sampling (managed; native OMPL optional) | Kuffner & LaValle, ICRA 2000; [OMPL](https://ompl.kavrakilab.org/) |
| PRM* | Managed roadmap sampling | Karaman & Frazzoli 2011, DOI [10.1177/0278364911406761](https://doi.org/10.1177/0278364911406761) |
| CHOMP-lite | Post-process smoother | Zucker et al. 2013, DOI [10.1177/0278364913488805](https://doi.org/10.1177/0278364913488805) |
| Stewart / Gough | Leg-length IK/FK, stroke-space collision/RRT (`Family=stewart`, **meters**) | Merlet; Dasgupta & Mruthyunjaya — see METHODS |
| Group / tree planning | `PlanningGroup` / `GroupMap` over named drivers | Kinematic-tree ADRs / METHODS |
| Holonomic SE(2) | Base x/y/yaw appended to sampling | LaValle, *Planning Algorithms* (2006) |
| Legged gait | Duty-cycle gait + SSM validation (`Family=legged`, **radians**) | Lynch & Park; Song & Waldron; McGhee & Frank — see METHODS |

</details>

## Context: attach, groups, SRDF

```csharp
var ctx = PlanningContext.Create(robot, scene)
    .Attach("workpiece", workpieceBox, tcpLocalFrame)
    .ForGroup(SrdfLoader.LoadGroups("robot.srdf")[0]);

var checker = CollisionCheckerFactory.Create(robot, attached: ctx.Attached);
var result = SamplingPlanner.Create(checker).Plan(
    new PlanningRequest(robot, start, goal, ctx.ToPlanningOptions()));
```

Non-group joints stay locked at the start configuration.

## Units & export

Internal units: **radians**, **seconds**, **meters** (`Motus.Core.Units` for °↔rad). Stewart configurations are **meters** (leg stroke); legged joints are **radians**.

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

Changelog: [CHANGELOG.md](CHANGELOG.md). Current pin for Grasshopper: **0.13.0**.

## Safety

Motus does **not** command physical robots. Presets and URDF imports are planning defaults — verify limits and calibration before hardware use. See [docs/safety.md](docs/safety.md).

This library and bundled presets were developed with AI assistance; values are approximate and must be checked against manufacturer data.

## Attribution

- Managed RRT-Connect implements Kuffner & LaValle (ICRA 2000). Native path uses [OMPL](https://ompl.kavrakilab.org/) (BSD-3-Clause) when built.
- Robot presets are approximate public-datasheet values for planning/visualization only.
