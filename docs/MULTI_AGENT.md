# Multi-agent operating model (Close Open Developments)

Standing roster for Motus.NET + Motus.Grasshopper feature phases. Coordinator merges only after NASA exit + review trio + citation gate.

## Roster

| Agent | Owns | Must not |
|-------|------|----------|
| **Coordinator** | Kickoff, DAG, merge order, NuGet/pin, todos | Implement both repos alone |
| **NetCore** | Motus.NET APIs, solvers, `*MethodRefs` DOI consts | GH kinematics; skipping reason codes |
| **NetTest** | Tests, fixtures, determinism | API design without NetCore |
| **GhWire** | Thin pins, Family Status, examples, pin | Reimplementing algorithms in GH |
| **LitCite** | DOI resolve, SOTA alt scan, BibTeX, why-not lines | Implementing solvers |
| **DocsAdr** | METHODS.md, architecture/ADR/roadmap/CHANGELOG, GH component one-liners | Silent contracts; uncited methods |
| **Review_Architecture** | Seam reuse, no second stack | Style nits |
| **Review_Testing_NASA** | Round-trips, seeds, caps | Ignoring Family/unit tests |
| **Review_Trust_Units** | Units, NaN/Inf, handoff honesty, citation present | Approving silent fails or uncited solvers |

## Kickoff skeleton

```
Phase: P{n} — {title}
Repo primary: Motus.NET | Motus.Grasshopper
NASA bar: units, Family, reason codes, seeds, no silent fail
Reuse: {existing types}
Primary algorithm + DOI: {from docs/METHODS.md}
SOTA alts to document (not implement): {from METHODS.md}
Forbidden: new Mac C++, GH kinematics, AxisCount==6 gating, uncited solvers
Deliver: code + tests + MethodRefs + METHODS.md + bib; exit checklist
Handoff: NetCore → NetTest; LitCite → DocsAdr; GhWire parallel; then Review×3
```

## Hot-file freeze

| File | Owner when contended |
|------|----------------------|
| `SamplingPlannerRegistry.cs` / `SamplingPlannerId.cs` | PRM/CHOMP before Stewart |
| `PlanExecutor.cs` (GH) | Stewart → tree → SE2/legged |
| `TrajectoryRetimer.cs` | Totg phase |
| `Planning.cs` / `PlanningOptions` | Constraint hook first |
| MotusNetPackages.props (GH) | Coordinator at phase end |

See also [METHODS.md](METHODS.md) and [roadmap.md](roadmap.md).
