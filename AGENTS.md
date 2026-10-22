# AGENTS.md

Notes for AI agents and maintainers working on Motus.NET. Mirrors the conventions in
[Motus.Grasshopper AGENTS.md](https://github.com/lasaths/Motus.Grasshopper/blob/master/AGENTS.md).

## Engineering bar

- **Clear contracts**: explicit units (rad vs m, `RobotPreset.Family`), and Status / error strings that name the real failure.
- **No silent failures**: reject NaN/Inf and out-of-range input at load and API boundaries; return structured reasons, never garbage poses.
- **Deterministic**: documented tolerances, iteration caps, seeded randomness. Methods and DOIs are listed in [docs/METHODS.md](docs/METHODS.md).
- Prefer correctness and reviewability over clever shortcuts.

## Boundaries

- **Planning, preview and export only.** No live robot control: no RTDE, sockets to controllers, or motion commands. Execution lives in downstream control plugins.
- Experimental tracks stay outside the supported surface ([docs/supported-surface.md](docs/supported-surface.md)) until they have an ADR, METHODS entry and tests.

## Collision rule (exact)

- The final collision verdict comes from **exact shapes** (segment/triangle/OBB distances, SAT). Bounding volumes (AABB, BVH, spheres) are used only as a **provably conservative broad phase**: they may cull pairs that cannot touch, and never decide "clear" on their own.
- Bamboo planning adds `MotionConstraints.ClearanceMarginM` (2 mm) to every obstacle. Only grasped/clamped contacts are exempt.
- A speed-up must not change a verdict. `ShippedDefaultPlan_IsIdenticalToLivePlanning` (Linux x64) and the shipped-plan replay tests (every platform) must stay green. If a change alters the shipped Warren plan, treat that as a regression unless it is deliberate and documented.

## Sanitization

- Names written to URDF/XML go through `UrdfName.Sanitize`. Mesh file names go through the writer's sanitizer.
- External robot/mechanism descriptions are validated: limits, schema versions, size caps.
- No machine-local paths (`/Users/...`, `/workspace/...`, `C:\...`) in code, tests or docs. Tests find fixtures relative to the repo (`tests/fixtures/...`) via `AppContext.BaseDirectory`.
- Never commit credentials or tokens.

## Build and test

CI (`.github/workflows/ci.yml`) runs on ubuntu/macos/windows plus a native-integration job:

```bash
dotnet restore Motus.NET.slnx
./scripts/build-native-stub.sh
dotnet build Motus.NET.slnx -c Release --no-restore
dotnet test  Motus.NET.slnx -c Release --no-build
```

The `.slnx` needs SDK 9+. With an SDK 8 toolchain, build and test each project instead:

```bash
dotnet test tests/Motus.Core.Tests/Motus.Core.Tests.csproj -c Release      # ~15 min, includes the bamboo truss plan tests
dotnet test tests/Motus.OMPL.Tests/Motus.OMPL.Tests.csproj -c Release
dotnet test tests/Motus.Presets.Tests/Motus.Presets.Tests.csproj -c Release
dotnet build src/Motus.Viewer/Motus.Viewer.csproj -c Release
```

The full native build (OMPL/FCL) is in `native/` (CMake). See the native-integration job.

## Layout

```
src/Motus.Core          models, tasks, validation, retiming, export
src/Motus.Geometry      FK/IK, collision (CollisionGeometry, ExactNarrowPhase), LIN
src/Motus.Construction  bamboo work cell, truss planner, build search, AssemblyPlanner
src/Motus.OMPL.NET      sampling planners (+ Motus.OMPL.Native, Motus.Native P/Invoke)
src/Motus.Presets       URDF / xacro / SRDF loaders and writer
src/Motus.Viewer        Blazor WASM preview (experimental)
tests/                  xUnit suites + fixtures
```
