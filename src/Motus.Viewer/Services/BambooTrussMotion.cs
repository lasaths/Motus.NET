using Motus.Core;
namespace Motus.Viewer.Services;

/// <summary>
/// Motus string-identity tasks and motion keys for /bamboo-truss: each member is a pick (single store
/// feed) → place (cane on its vertical-plane truss edge) pair. Spatial IK via
/// <see cref="BambooIcdMotion.IkSolveAxis"/> with multi-tilt approach (jaw tilt need not equal strut pitch).
/// </summary>
public static class BambooTrussMotion
{
    public sealed class CycleBound
    {
        public required string MemberId { get; init; }
        public double T0 { get; init; }
        public double T1 { get; init; }
    }

    public sealed class TrussBuild
    {
        public string Error { get; set; } = "";
        public List<BambooIcdMotion.MotionKey> Keys { get; set; } = new();
        public List<CycleBound> Cycles { get; set; } = new();
        public List<BambooIcdMotion.TaskEdit> Tasks { get; set; } = new();
    }

    /// <summary>Jaw-axis alignment tolerance for place grasps on inclined vertical-plane edges (degrees).</summary>
    public const double PlaceAxisErrDeg = 10;

    /// <summary>
    /// Target joint step (deg) for Motus samples while linking. Coarser than the viewer 0.5° scan,
    /// but fine enough that Swing vias which clear the planner also clear BambooTrussScan when Motus
    /// actually runs (WASM must UseCell — disk LoadFile is unavailable in the browser).
    /// </summary>
    const double MotusPlanStepDeg = 5;
    /// <summary>Hard cap on Motus samples per carrying segment (keeps WASM Build bounded).</summary>
    const int MotusPlanStepsMax = 12;
    /// <summary>Max IK candidates kept on Motus-checked stages (Swing/Hold/Release/Depart).</summary>
    const int MotusCandMax = 6;
    /// <summary>Beam width: after each stage, keep only the cheapest predecessors.</summary>
    const int LinkBeamWidth = 5;

    static readonly object CellGate = new();
    static BambooIcdCell? MotusCell;
    static List<BambooIcdCell.StrutPlacement> Placed = new();
    static BambooIcdCell.StrutPlacement? CurrentRelease;

    /// <summary>
    /// Bind the Motus collision cell used while planning. Required under Blazor WASM (no disk URDF);
    /// the /bamboo-truss page passes the same cell the viewer scan uses so planner Motus == Play Motus.
    /// </summary>
    public static void UseCell(BambooIcdCell cell)
    {
        ArgumentNullException.ThrowIfNull(cell);
        lock (CellGate)
        {
            MotusCell = cell;
            _planTrack = null;
        }
    }

    static bool ReleaseOverlapsPlaced()
    {
        if (CurrentRelease is not { } r || Placed.Count == 0) return false;
        foreach (var p in Placed)
        {
            var dx = p.X - r.X; var dy = p.Y - r.Y; var dz = p.Z - r.Z;
            if (dx * dx + dy * dy + dz * dz < 0.012 * 0.012) return true;
        }
        return false;
    }

    static BambooIcdCell.StrutTrack? _planTrack;
    static BambooIcdCell.StrutTrack PlanningTrack()
    {
        if (_planTrack is not null) return _planTrack;
        // Capture HeldOffset as if gripping the store feed (same offset used all carry cycle).
        var rest = BambooTrussLayout.Store;
        var sols = BambooIcdMotion.IkSolveAxis(rest.X, rest.Y, rest.Z, rest.PitchDeg, rest.Axis, BambooIcdMotion.HangPose.Pole, 2);
        var q = sols.FirstOrDefault() ?? BambooIcdMotion.HomeDeg.ToArray();
        var keys = new List<BambooIcdMotion.MotionKey>
        {
            new() { T = 0, Label = "Close", Q = q, GR = BambooTrussLayout.JawClosedMm, Hold = 1 },
            new() { T = 1, Label = "Open", Q = q, GR = BambooTrussLayout.JawOpenMm, Hold = 2 }
        };
        var cell = TryCell() ?? throw new InvalidOperationException("Motus cell required for PlanningTrack");
        _planTrack = cell.TrackStrut(keys, rest);
        if (_planTrack.Miss is not null || _planTrack.HeldOffset is null)
        {
            // Fallback: strut mesh at jaw origin along tool Z.
            _planTrack = new BambooIcdCell.StrutTrack
            {
                Rest = BambooIcdCell.PlacementWorld(rest),
                HeldOffset = Frame.Identity,
                Released = BambooIcdCell.PlacementWorld(rest)
            };
        }
        return _planTrack;
    }

    static BambooIcdCell? TryCell()
    {
        if (MotusCell is not null) return MotusCell;
        lock (CellGate)
        {
            if (MotusCell is not null) return MotusCell;
            try
            {
                var urdf = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..",
                    "tests", "fixtures", "bamboo_icd", "bamboo_icd.urdf"));
                if (!File.Exists(urdf))
                    urdf = Path.GetFullPath(Path.Combine("tests", "fixtures", "bamboo_icd", "bamboo_icd.urdf"));
                if (!File.Exists(urdf)) return null;
                var cell = BambooIcdCell.LoadFile(urdf, BambooTrussLayout.StrutLength);
                cell.IncludePole = false;
                cell.SetAnchorPads(BambooTrussLayout.AnchorPads
                    .Select(a => (a.X, a.Z, BambooTrussLayout.AnchorPadHalf)).ToList());
                MotusCell = cell;
                return MotusCell;
            }
            catch
            {
                // WASM / missing fixture: caller must UseCell. SegCheck falls back to geometry.
                return null;
            }
        }
    }

    static BambooIcdCell Cell =>
        TryCell() ?? throw new InvalidOperationException(
            "BambooTrussMotion Motus cell unset — call UseCell with the viewer BambooIcdCell.");

    public static List<BambooIcdMotion.TaskEdit> DefaultTasks(IReadOnlyList<BambooTrussLayout.MemberDef>? members = null)
    {
        var tasks = new List<BambooIcdMotion.TaskEdit>();
        foreach (var m in members ?? BambooTrussLayout.Members)
        {
            tasks.Add(new BambooIcdMotion.TaskEdit
            {
                Identity = "pick",
                Object = m.Id,
                JawMm = BambooTrussLayout.JawClosedMm,
                Planes =
                [
                    Plane("approach", m.Store, BambooTrussLayout.StoreApproachDy),
                    Plane("grasp", m.Store, 0)
                ]
            });
            tasks.Add(new BambooIcdMotion.TaskEdit
            {
                Identity = "place",
                Object = m.Id,
                JawMm = BambooTrussLayout.JawOpenMm,
                Planes =
                [
                    Plane("hold", m.Place, BambooTrussLayout.PlaceHoldDy),
                    Plane("release", m.Place, 0)
                ]
            });
        }
        return tasks;
    }

    static BambooIcdMotion.PlaneEdit Plane(string name, BambooIcdCell.StrutPlacement p, double dy) => new()
    {
        Name = name,
        X = p.X,
        Y0 = p.Y + dy,
        Z = p.Z,
        PitchDeg = p.PitchDeg,
        YawDeg = p.YawDeg,
        YMm = 0
    };

    public static List<(BambooIcdMotion.TaskEdit Pick, BambooIcdMotion.TaskEdit Place, string MemberId)> Pairs(
        IReadOnlyList<BambooIcdMotion.TaskEdit> tasks)
    {
        var pairs = new List<(BambooIcdMotion.TaskEdit, BambooIcdMotion.TaskEdit, string)>();
        BambooIcdMotion.TaskEdit? pendingPick = null;
        string? pendingId = null;
        foreach (var t in tasks)
        {
            if (t.Identity == "pick")
            {
                pendingPick = t;
                pendingId = t.Object;
            }
            else if (t.Identity == "place" && pendingPick is not null &&
                     string.Equals(pendingId, t.Object, StringComparison.Ordinal))
            {
                pairs.Add((pendingPick, t, t.Object));
                pendingPick = null;
                pendingId = null;
            }
        }
        return pairs;
    }

    public static TrussBuild Build(IReadOnlyList<BambooIcdMotion.TaskEdit>? tasks = null)
    {
        tasks ??= DefaultTasks();
        var pairs = Pairs(tasks);
        if (pairs.Count == 0)
            return new TrussBuild { Error = "No pick/place pairs (object-matched).", Tasks = tasks.ToList() };
        if (pairs.Count > 1 && TryCell() is null)
            return new TrussBuild
            {
                Error = "Motus cell required for multi-member path (call UseCell with the viewer BambooIcdCell).",
                Tasks = tasks.ToList(),
                Keys = HomeKeys()
            };

        var allKeys = new List<BambooIcdMotion.MotionKey>();
        var cycles = new List<CycleBound>();
        var n = pairs.Count;
        for (var i = 0; i < n; i++)
        {
            var (pick, place, memberId) = pairs[i];
            Placed = pairs.Take(i)
                .Select(p => BambooTrussLayout.Members.First(m => m.Id == p.MemberId).Place)
                .ToList();
            var one = BuildOne(pick, place, memberId, afterPlaced: i > 0);
            if (!string.IsNullOrEmpty(one.Error))
                return new TrussBuild { Error = $"{memberId}: {one.Error}", Tasks = tasks.ToList(), Keys = HomeKeys() };

            var t0 = i / (double)n;
            var t1 = (i + 1) / (double)n;
            cycles.Add(new CycleBound { MemberId = memberId, T0 = t0, T1 = t1 });
            foreach (var k in one.Keys)
            {
                allKeys.Add(new BambooIcdMotion.MotionKey
                {
                    T = t0 + k.T * (t1 - t0),
                    Label = $"{memberId}:{k.Label}",
                    Q = k.Q.ToArray(),
                    GR = k.GR,
                    Hold = k.Hold
                });
            }
        }

        for (var i = 1; i < allKeys.Count; i++)
            if (allKeys[i].T <= allKeys[i - 1].T)
                allKeys[i].T = allKeys[i - 1].T + 1e-6;
        allKeys[^1].T = 1;

        return new TrussBuild { Error = "", Keys = allKeys, Cycles = cycles, Tasks = tasks.ToList() };
    }

    /// <summary>
    /// Same as <see cref="Build"/> but awaits <paramref name="onMember"/> between members so Blazor WASM can paint.
    /// </summary>
    public static async Task<TrussBuild> BuildAsync(
        IReadOnlyList<BambooIcdMotion.TaskEdit>? tasks = null,
        Func<string, Task>? onMember = null)
    {
        tasks ??= DefaultTasks();
        var pairs = Pairs(tasks);
        if (pairs.Count == 0)
            return new TrussBuild { Error = "No pick/place pairs (object-matched).", Tasks = tasks.ToList() };
        if (pairs.Count > 1 && TryCell() is null)
            return new TrussBuild
            {
                Error = "Motus cell required for multi-member path (call UseCell with the viewer BambooIcdCell).",
                Tasks = tasks.ToList(),
                Keys = HomeKeys()
            };

        var allKeys = new List<BambooIcdMotion.MotionKey>();
        var cycles = new List<CycleBound>();
        var n = pairs.Count;
        for (var i = 0; i < n; i++)
        {
            if (onMember is not null) await onMember(pairs[i].MemberId);
            else await Task.Yield();
            var (pick, place, memberId) = pairs[i];
            Placed = pairs.Take(i)
                .Select(p => BambooTrussLayout.Members.First(m => m.Id == p.MemberId).Place)
                .ToList();
            var one = BuildOne(pick, place, memberId, afterPlaced: i > 0);
            if (!string.IsNullOrEmpty(one.Error))
                return new TrussBuild { Error = $"{memberId}: {one.Error}", Tasks = tasks.ToList(), Keys = HomeKeys() };

            var t0 = i / (double)n;
            var t1 = (i + 1) / (double)n;
            cycles.Add(new CycleBound { MemberId = memberId, T0 = t0, T1 = t1 });
            foreach (var k in one.Keys)
            {
                allKeys.Add(new BambooIcdMotion.MotionKey
                {
                    T = t0 + k.T * (t1 - t0),
                    Label = $"{memberId}:{k.Label}",
                    Q = k.Q.ToArray(),
                    GR = k.GR,
                    Hold = k.Hold
                });
            }
        }
        if (allKeys.Count > 0)
        {
            for (var i = 1; i < allKeys.Count; i++)
                if (allKeys[i].T <= allKeys[i - 1].T)
                    allKeys[i].T = allKeys[i - 1].T + 1e-6;
            allKeys[^1].T = 1;
        }
        return new TrussBuild { Error = "", Keys = allKeys, Cycles = cycles, Tasks = tasks.ToList() };
    }

    static List<BambooIcdMotion.MotionKey> HomeKeys() =>
    [
        new() { T = 0, Label = "Home", Q = BambooIcdMotion.HomeDeg.ToArray(), GR = BambooTrussLayout.JawOpenMm, Hold = 0 },
        new() { T = 1, Label = "Home", Q = BambooIcdMotion.HomeDeg.ToArray(), GR = BambooTrussLayout.JawOpenMm, Hold = 0 }
    ];

    readonly record struct Pose(double X, double Y, double Z, double Pitch, double Yaw)
    {
        public (double X, double Y, double Z) Axis => BambooIcdMotion.StrutAxis(Pitch, Yaw);
        public Pose At(double x, double y, double z) => this with { X = x, Y = y, Z = z };
    }

    static Pose? PlaneWorld(BambooIcdMotion.TaskEdit task, string name)
    {
        var pl = task.Planes.FirstOrDefault(p => p.Name == name);
        if (pl is null) return null;
        return new Pose(pl.X, pl.Y0 + pl.YMm / 1000.0, pl.Z, pl.PitchDeg, pl.YawDeg);
    }

    static List<double[]> Solve(Pose p, double axisErrDeg)
    {
        var axis = p.Axis;
        var tilts = new[] { p.Pitch, -Math.Abs(p.Pitch), Math.Abs(p.Pitch), 0.0, -20, -40, -60, -80, -90, 20, 40 };
        var seen = new HashSet<int>();
        var sols = new List<double[]>();
        foreach (var tilt in tilts)
        {
            if (!seen.Add((int)Math.Round(tilt))) continue;
            sols.AddRange(BambooIcdMotion.IkSolveAxis(p.X, p.Y, p.Z, tilt, axis, BambooIcdMotion.HangPose.Pole, axisErrDeg));
        }
        return sols;
    }

    static List<double[]> SolvePick(Pose p)
    {
        var sols = BambooIcdMotion.IkSolveAxis(p.X, p.Y, p.Z, p.Pitch, p.Axis, BambooIcdMotion.HangPose.Pole, 2);
        var near = sols.Where(q => Math.Abs(Wrap180(q[0] + 90)) < 50)
            .OrderBy(q => Math.Abs(Wrap180(q[1] - BambooIcdMotion.HomeDeg[1])))
            .ToList();
        return near.Count > 0 ? near : sols;
    }

    static List<double[]> Dedup(List<double[]> sols, int max)
    {
        if (sols.Count <= max) return sols;
        var step = sols.Count / (double)max;
        return Enumerable.Range(0, max).Select(k => sols[(int)(k * step)]).ToList();
    }

    static List<double[]> AxisIk(double x, double y, double z, (double X, double Y, double Z) axis, params double[] tilts)
    {
        var sols = new List<double[]>();
        foreach (var t in tilts)
            sols.AddRange(BambooIcdMotion.IkSolveAxis(x, y, z, t, axis, BambooIcdMotion.HangPose.Pole, 15));
        return Dedup(sols, MotusCandMax);
    }

    /// <summary>
    /// Pick store horizontally, lift, reorient hang-side of the vertical plane, swing out above the
    /// member, lower, retreat while holding, release/open, Home.
    /// </summary>
    static BambooIcdMotion.MotionBuild BuildOne(BambooIcdMotion.TaskEdit pick, BambooIcdMotion.TaskEdit place, string memberId, bool afterPlaced)
    {
        var approach = PlaneWorld(pick, "approach");
        var grasp = PlaneWorld(pick, "grasp");
        var hold = PlaneWorld(place, "hold");
        var release = PlaneWorld(place, "release");
        if (approach is null || grasp is null || hold is null || release is null)
            return new BambooIcdMotion.MotionBuild { Error = "plane missing", Keys = HomeKeys() };

        var sa = SolvePick(approach.Value);
        var sg = SolvePick(grasp.Value);
        if (afterPlaced)
        {
            var saClear = sa.Where(q => MotusPoseClear(q, false)).ToList();
            if (saClear.Count > 0) sa = saClear;
            var sgClear = sg.Where(q => MotusPoseClear(q, false)).ToList();
            if (sgClear.Count > 0) sg = sgClear;
        }
        var sr = Solve(release.Value, PlaceAxisErrDeg);
        var sh = Solve(hold.Value, PlaceAxisErrDeg);
        if (sh.Count == 0 && sr.Count > 0)
        {
            foreach (var dy in new[] { 0.008, 0.015, 0.02, 0.004 })
            {
                sh = Solve(release.Value.At(release.Value.X, release.Value.Y + dy, release.Value.Z), PlaceAxisErrDeg);
                if (sh.Count > 0) break;
            }
            if (sh.Count == 0) sh = sr;
        }
        // Stacked edges (baseR/diagR share XZ): approach Hold from hang-side (−Z) / outside so
        // Swing→Hold→Lower does not drop the held cane through the neighbour below.
        if (afterPlaced)
        {
            var ox = Math.Sign(release.Value.X) * 0.015;
            var holdAlts = new List<double[]>();
            foreach (var (x, y, z) in new (double, double, double)[]
                     {
                         (release.Value.X + ox, release.Value.Y + 0.025, release.Value.Z - 0.03),
                         (release.Value.X + ox, release.Value.Y + 0.02, release.Value.Z - 0.02),
                         (release.Value.X, release.Value.Y + 0.03, release.Value.Z - 0.035),
                         (release.Value.X + ox, hold.Value.Y, Math.Max(0.06, release.Value.Z - 0.04))
                     })
            {
                holdAlts.AddRange(Solve(new Pose(x, y, z, release.Value.Pitch, release.Value.Yaw), PlaceAxisErrDeg));
                holdAlts.AddRange(AxisIk(x, y, z, release.Value.Axis, release.Value.Pitch, 0, -40));
            }
            holdAlts = Dedup(holdAlts, MotusCandMax);
            if (holdAlts.Count > 0)
                sh = Dedup(holdAlts.Concat(sh).ToList(), MotusCandMax);
        }
        // Motus-checked stages explode as O(|prev|·|next|·steps); keep place sols bounded.
        sh = Dedup(sh, MotusCandMax);
        sr = Dedup(sr, MotusCandMax);

        var bad = new List<string>();
        if (sa.Count == 0) bad.Add("pick:approach");
        if (sg.Count == 0) bad.Add("pick:grasp");
        if (sh.Count == 0) bad.Add("place:hold");
        if (sr.Count == 0) bad.Add("place:release");
        if (bad.Count > 0)
            return new BambooIcdMotion.MotionBuild { Error = string.Join(", ", bad) + " unreachable", Keys = HomeKeys() };

        var G = grasp.Value;
        var H = hold.Value;
        var R = release.Value;
        CurrentRelease = new BambooIcdCell.StrutPlacement(R.X, R.Y, R.Z, R.Pitch, R.Yaw);
        var placeAxis = R.Axis;
        var stages = new List<Stage>();
        var G0 = grasp.Value;

        if (afterPlaced)
        {
            // Hang-side / +X arc so empty-hand return clears planted baseL (R-grip rail).
            var via = new List<double[]>();
            foreach (var (x, y, z) in new (double, double, double)[]
                     {
                         (0.10, 0.22, -0.04),
                         (0.14, 0.20, -0.02),
                         (0.08, 0.18, G0.Z),
                         (0.05, 0.16, G0.Z),
                         (G0.X + 0.04, 0.14, G0.Z),
                         (0.0, 0.20, -0.05)
                     })
            {
                via.AddRange(SolvePick(new Pose(x, y, z, G0.Pitch, G0.Yaw)));
                via.AddRange(Solve(new Pose(x, y, z, 0, G0.Yaw), 15));
            }
            via = Dedup(via, MotusCandMax);
            var viaClear = via.Where(q => MotusPoseClear(q, false)).ToList();
            if (viaClear.Count > 0) via = viaClear;
            if (via.Count > 0)
                stages.Add(new Stage { Name = "Reenter", Sols = via });
        }

        stages.Add(new Stage { Name = "pick:approach", Sols = sa });
        stages.Add(new Stage { Name = "pick:grasp", Sols = sg });

        List<double[]>? lift = null;
        // After baseL is planted, lift while pulling hang-side (−Z) so the 0.5 m held feed clears it.
        var liftTargets = afterPlaced
            ? new (double dy, double dz)[] { (TransitDy, -0.04), (0.06, -0.05), (0.05, -0.03), (0.04, -0.02), (TransitDy, 0) }
            : new (double dy, double dz)[] { (TransitDy, 0), (0.04, 0), (0.05, 0), (0.03, 0) };
        foreach (var (dy, dz) in liftTargets)
        {
            var cand = SolvePick(G.At(G.X, G.Y + dy, G.Z + dz));
            if (cand.Count > 0) { lift = cand; break; }
        }
        if (lift is null || lift.Count == 0)
            return new BambooIcdMotion.MotionBuild { Error = "transit toward place:hold unreachable (Lift)", Keys = HomeKeys() };
        stages.Add(new Stage { Name = "Lift", Sols = lift });

        // Hang-side bay (Z below PlaneZ): pull horizontal, then pitch onto the member axis.
        var bayZ = Math.Min(G.Z, 0.02);
        var bayY = Math.Max(0.14, Math.Max(G.Y, R.Y) + 0.06);
        {
            var pull = SolvePick(new Pose(0, bayY, bayZ, G.Pitch, G.Yaw));
            if (pull.Count == 0) pull = Solve(new Pose(0, bayY, bayZ, 0, G.Yaw), 15);
            if (pull.Count > 0) stages.Add(new Stage { Name = "Pitch", Sols = pull });
        }

        var pitched = false;
        foreach (var (x, y, z) in new (double, double, double)[]
                 {
                     (0, bayY, bayZ),
                     (R.X * 0.4, bayY, bayZ),
                     (0, bayY + 0.02, bayZ),
                     (R.X * 0.5, Math.Max(bayY, 0.14), Math.Max(0.05, R.Z - 0.05)),
                     (H.X, Math.Max(H.Y + 0.04, 0.13), Math.Max(0.06, H.Z - 0.04)),
                     (H.X, Math.Max(H.Y + 0.03, 0.12), H.Z)
                 })
        {
            var ps = AxisIk(x, y, z, placeAxis, R.Pitch, -Math.Abs(R.Pitch), Math.Abs(R.Pitch), 0, -40, -60, -80, 20);
            if (ps.Count > 0)
            {
                stages.Add(new Stage { Name = "Pitch", Sols = ps });
                pitched = true;
                break;
            }
        }
        if (!pitched)
            return new BambooIcdMotion.MotionBuild { Error = "transit toward place:hold unreachable (Pitch)", Keys = HomeKeys() };

        // Outside / hang-side Z-step into the plane so Swing never sweeps across the opposite
        // triangle (diagR:Swing θ5↔placed-diagL when leaping through x≈0).
        var swingY = Math.Max(Math.Max(H.Y, R.Y) + 0.04, 0.125);
        var side = Math.Sign(R.X);
        if (side == 0) side = 1;
        var outX = R.X + 0.03 * side;
        // Step Z from bay toward PlaneZ at outX, place orientation.
        foreach (var z in new[] { bayZ, bayZ + 0.02, bayZ + 0.035, Math.Max(bayZ + 0.05, R.Z - 0.02), R.Z })
        {
            var ps = new List<double[]>();
            ps.AddRange(Solve(new Pose(outX, swingY, z, R.Pitch, R.Yaw), 15));
            ps.AddRange(AxisIk(outX, swingY, z, placeAxis, R.Pitch, 0, -Math.Abs(R.Pitch), -40));
            // Keep jaw on the member's side of the midplane when placing off-centre.
            if (Math.Abs(R.X) > 0.02)
                ps = ps.Where(q =>
                {
                    var f = BambooIcdMotion.FkJaw(q);
                    return Math.Sign(f.Pos[0]) == 0 || Math.Sign(f.Pos[0]) == side || Math.Abs(f.Pos[0]) < 0.02;
                }).ToList();
            ps = Dedup(ps, MotusCandMax);
            if (ps.Count > 0)
                stages.Add(new Stage { Name = z >= R.Z - 0.005 ? "Swing" : "Pitch", Sols = ps });
        }
        // Final swing above the true member centre.
        {
            var swingRaw = new List<double[]>();
            foreach (var (x, y, z) in new (double, double, double)[]
                     {
                         (outX, swingY, R.Z),
                         (R.X, swingY, R.Z),
                         (H.X, swingY, H.Z)
                     })
            {
                swingRaw.AddRange(Solve(new Pose(x, y, z, R.Pitch, R.Yaw), 15));
                swingRaw.AddRange(AxisIk(x, y, z, placeAxis, R.Pitch, 0, -Math.Abs(R.Pitch), -40));
            }
            swingRaw = Dedup(swingRaw, MotusCandMax * 2);
            var anchors = sh.Concat(sr).ToList();
            var swing = new List<double[]>();
            foreach (var maxDeg in new[] { 55.0, 85.0, 120.0 })
            {
                swing.Clear();
                foreach (var q in swingRaw)
                {
                    foreach (var a in anchors)
                    {
                        var ok = true;
                        for (var k = 0; k < 5; k++)
                            if (Math.Abs(Wrap180(q[k] - a[k])) > maxDeg) { ok = false; break; }
                        if (ok) { swing.Add(q); break; }
                    }
                }
                if (swing.Count > 0) break;
            }
            if (swing.Count == 0) swing = swingRaw.Count > 0 ? swingRaw : sh;
            swing = Dedup(swing.Concat(sh).ToList(), MotusCandMax);
            stages.Add(new Stage { Name = "Swing", Sols = swing });
        }

        stages.Add(new Stage { Name = "place:hold", Sols = sh });
        // Hang-side via before the final seat so Lower does not drop through a stacked neighbour.
        if (afterPlaced && !ReleaseOverlapsPlaced())
        {
            var via = new List<double[]>();
            var ox = Math.Sign(R.X) * 0.01;
            foreach (var (x, y, z) in new (double, double, double)[]
                     {
                         (R.X + ox, R.Y + 0.015, R.Z - 0.02),
                         (R.X + ox, R.Y + 0.01, R.Z - 0.015),
                         (R.X, R.Y + 0.012, R.Z - 0.02)
                     })
            {
                via.AddRange(Solve(new Pose(x, y, z, R.Pitch, R.Yaw), PlaceAxisErrDeg));
                via.AddRange(AxisIk(x, y, z, placeAxis, R.Pitch, 0, -40));
            }
            via = Dedup(via, MotusCandMax);
            if (via.Count > 0)
                stages.Add(new Stage { Name = "Swing", Sols = via }); // Motus-checked like Swing
        }
        stages.Add(new Stage { Name = "place:release", Sols = sr });

        // Depart to the store-side bay (away from PlaneZ and from the x=0 upright) so Open
        // does not leave R-grip jaws on placed-sep / neighbours.
        {
            var store = BambooTrussLayout.Store;
            var away = Math.Sign(R.X) == 0 ? 0.04 : 0.035 * Math.Sign(R.X);
            var depRaw = new List<double[]>();
            foreach (var (x, y, z) in new (double, double, double)[]
                     {
                         (store.X, Math.Max(0.15, R.Y + 0.06), store.Z),
                         (store.X, 0.16, store.Z),
                         (R.X + away, Math.Max(0.14, R.Y + 0.05), store.Z),
                         (Math.Sign(R.X) * 0.08, 0.15, 0.05),
                         (store.X + 0.02, 0.17, store.Z),
                         (R.X + away, R.Y + 0.05, Math.Max(0.05, R.Z - 0.05))
                     })
            {
                depRaw.AddRange(SolvePick(new Pose(x, y, z, 0, 90)));
                depRaw.AddRange(Solve(new Pose(x, y, z, 0, 90), 15));
                depRaw.AddRange(Solve(new Pose(x, y, z, R.Pitch, R.Yaw), 15));
                depRaw.AddRange(AxisIk(x, y, z, placeAxis, R.Pitch, 0, -40));
            }
            depRaw = Dedup(depRaw, MotusCandMax * 2);
            var dep = depRaw.Where(q =>
            {
                var f = BambooIcdMotion.FkJaw(q);
                var dx = f.Pos[0] - R.X; var dy = f.Pos[1] - R.Y; var dz = f.Pos[2] - R.Z;
                // Far from release and not near the shared upright column.
                return Math.Sqrt(dx * dx + dy * dy + dz * dz) >= 0.035 && Math.Abs(f.Pos[0]) + Math.Abs(f.Pos[2] - BambooTrussLayout.PlaneZ) > 0.04;
            }).ToList();
            if (dep.Count == 0) dep = depRaw;
            if (dep.Count > 0)
                stages.Add(new Stage { Name = "Depart", Sols = Dedup(dep, MotusCandMax) });
            else
                stages.Add(new Stage { Name = "Depart", Sols = [BambooIcdMotion.HomeDeg.ToArray()] });
        }

        stages.Add(new Stage { Name = "Retract", Sols = [BambooIcdMotion.HomeDeg.ToArray()] });

        var chained = LinkStages(stages);
        if (!chained.Ok && chained.Next == "Depart")
        {
            stages.RemoveAll(s => s.Name == "Depart");
            chained = LinkStages(stages);
        }
        if (!chained.Ok)
            return new BambooIcdMotion.MotionBuild
            {
                Error = string.IsNullOrEmpty(chained.Next) ? "path unlinkable" : chained.Next + " unreachable",
                Keys = HomeKeys()
            };

        return new BambooIcdMotion.MotionBuild { Error = "", Keys = Stamp(ChainItems(chained.Seq, place.JawMm, pick.JawMm)) };
    }

    const double TransitDy = 0.055;
    const double DepartDy = 0.03;

    sealed class Stage { public string Name = ""; public List<double[]> Sols = new(); }

    static double Wrap180(double a)
    {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    static int MotusStepsFor(double[] a, double[] b)
    {
        var jump = 0.0;
        for (var k = 0; k < 5; k++)
            jump = Math.Max(jump, Math.Abs(Wrap180(b[k] - a[k])));
        var steps = (int)Math.Ceiling(jump / MotusPlanStepDeg);
        return Math.Clamp(steps, 4, MotusPlanStepsMax);
    }

    /// <summary>
    /// Conservative capsule clearance: held cane vs already-placed edge canes (Motus-true enough to
    /// reject mid-span leaps through the opposite triangle when Motus IsClear is unavailable).
    /// </summary>
    static bool MotusPoseClear(double[] q, bool carrying)
    {
        if (Placed.Count == 0 || TryCell() is not { } cell) return true;
        try
        {
            var extras = Placed.Select((p, i) => ($"placed-{i}", p)).ToList();
            var track = PlanningTrack();
            var hold = carrying ? 1 : 0;
            var gR = carrying ? BambooTrussLayout.JawClosedMm : BambooTrussLayout.JawOpenMm;
            return cell.IsClear(q, BambooTrussLayout.JawClosedMm, gR, hold, carrying ? "Lower" : "Approach", track, extras);
        }
        catch { return false; }
    }

    static bool GeoClearVsPlaced(double[] q)
    {
        if (Placed.Count == 0) return true;
        var f = BambooIcdMotion.FkJaw(q);
        var ax = f.Z;
        var half = BambooTrussLayout.StrutLength / 2;
        var r = BambooIcdCell.StrutRadius;
        // Inflate slightly so geo rejects before Motus θ-housing grazes a placed cane.
        var minDist = 2 * r + 0.006;
        var a0 = (f.Pos[0] - ax[0] * half, f.Pos[1] - ax[1] * half, f.Pos[2] - ax[2] * half);
        var a1 = (f.Pos[0] + ax[0] * half, f.Pos[1] + ax[1] * half, f.Pos[2] + ax[2] * half);
        // Also keep the jaw frame centre off placed cane axes (θ5 housing proxy).
        var jaw = (f.Pos[0], f.Pos[1], f.Pos[2]);
        foreach (var p in Placed)
        {
            var bx = p.Axis;
            var b0 = (p.X - bx.X * half, p.Y - bx.Y * half, p.Z - bx.Z * half);
            var b1 = (p.X + bx.X * half, p.Y + bx.Y * half, p.Z + bx.Z * half);
            if (SegSegDist(a0, a1, b0, b1) < minDist) return false;
            if (PointSegDist(jaw, b0, b1) < r + 0.018) return false;
        }
        return true;
    }

    static double PointSegDist((double X, double Y, double Z) p, (double X, double Y, double Z) a, (double X, double Y, double Z) b)
    {
        var dx = b.X - a.X; var dy = b.Y - a.Y; var dz = b.Z - a.Z;
        var len2 = dx * dx + dy * dy + dz * dz;
        var t = len2 < 1e-18 ? 0 : Math.Clamp(((p.X - a.X) * dx + (p.Y - a.Y) * dy + (p.Z - a.Z) * dz) / len2, 0, 1);
        var qx = a.X + dx * t - p.X; var qy = a.Y + dy * t - p.Y; var qz = a.Z + dz * t - p.Z;
        return Math.Sqrt(qx * qx + qy * qy + qz * qz);
    }

    static double SegSegDist(
        (double X, double Y, double Z) a0, (double X, double Y, double Z) a1,
        (double X, double Y, double Z) b0, (double X, double Y, double Z) b1)
    {
        var best = double.MaxValue;
        for (var i = 0; i <= 12; i++)
        {
            var u = i / 12.0;
            var p = (a0.X + (a1.X - a0.X) * u, a0.Y + (a1.Y - a0.Y) * u, a0.Z + (a1.Z - a0.Z) * u);
            best = Math.Min(best, PointSegDist(p, b0, b1));
        }
        return best;
    }

    static (bool Ok, double Penalty) SegCheck(double[] a, double[] b, bool carrying, bool motusVsPlaced = false)
    {
        var floorBad = false;
        var steps = motusVsPlaced && Placed.Count > 0 ? MotusStepsFor(a, b) : 12;
        // Geometric placed clearance only when Motus cell is missing (WASM without UseCell).
        // With Motus available, IsClear is the source of truth — geo alone over-rejects at shared nodes.
        var geoVsPlaced = carrying && motusVsPlaced && Placed.Count > 0 && TryCell() is null;
        for (var s = 0; s <= steps; s++)
        {
            var u = s / (double)steps;
            var q = a.Select((v, k) => v + (b[k] - v) * u).ToArray();
            for (var i = 0; i < 5; i++)
                if (q[i] < BambooIcdMotion.LimitsDeg[i].Min - 0.05 || q[i] > BambooIcdMotion.LimitsDeg[i].Max + 0.05)
                    return (false, 0);
            var f = BambooIcdMotion.FkJaw(q);
            if (!floorBad)
                foreach (var pt in f.Pts)
                    if (pt[1] < -0.002) floorBad = true;
            if (carrying)
            {
                var ay = f.Z[1];
                var bot = f.Pos[1] - BambooTrussLayout.StrutLength / 2 * Math.Abs(ay) - BambooIcdCell.StrutRadius * Math.Sqrt(Math.Max(0, 1 - ay * ay));
                if (bot < -0.006) return (false, 0);
                if (bot < -0.002) floorBad = true;
                if (geoVsPlaced && !GeoClearVsPlaced(q))
                    return (false, 0);
            }
            // Motus vs placed: carrying (held cane) or empty-hand approach after members exist.
            if (motusVsPlaced && Placed.Count > 0 && TryCell() is { } cell)
            {
                try
                {
                    var extras = Placed.Select((p, i) => ($"placed-{i}", p)).ToList();
                    var track = PlanningTrack();
                    var hold = carrying ? 1 : 0;
                    var gR = carrying ? BambooTrussLayout.JawClosedMm : BambooTrussLayout.JawOpenMm;
                    if (!cell.IsClear(q, BambooTrussLayout.JawClosedMm, gR, hold, carrying ? "Lower" : "Approach", track, extras))
                        return (false, 0);
                }
                catch
                {
                    if (carrying && !GeoClearVsPlaced(q)) return (false, 0);
                    if (!carrying) return (false, 0);
                }
            }
        }
        return (true, floorBad ? BambooIcdMotion.FloorPenalty : 0);
    }

    static (bool Ok, double Cost, List<(string Name, double[] Q)> Seq, int Far, string Next) LinkStages(List<Stage> stages)
    {
        var n = stages.Count;
        var cost = stages.Select(st => st.Sols.Select(_ => 1e18).ToArray()).ToArray();
        var prev = stages.Select(st => st.Sols.Select(_ => -1).ToArray()).ToArray();
        var motus0 = Placed.Count > 0 && stages[0].Name is "pick:approach" or "Reenter";
        for (var i = 0; i < stages[0].Sols.Count; i++)
        {
            var (ok0, pen0) = SegCheck(BambooIcdMotion.HomeDeg, stages[0].Sols[i], false, motus0);
            if (ok0) cost[0][i] = pen0;
        }
        for (var s = 1; s < n; s++)
        {
            var leave = stages[s].Name is "Retract" or "Clear";
            var name = stages[s].Name;
            // Carrying after Close: Lift onward (not empty-hand Reenter/approach/grasp).
            var carry = name is "Lift" or "Pitch" or "Swing" or "place:hold" or "place:release" or "Depart";
            var overlap = ReleaseOverlapsPlaced();
            var motus = false;
            if (carry && !overlap && name is "Lift" or "Swing" or "place:hold" or "place:release" or "Depart")
                motus = true;
            if (!carry && Placed.Count > 0 && name is "pick:approach" or "Reenter")
                motus = true;
            // Only expand from the cheapest predecessors (beam) — Motus checks dominate WASM startup.
            var prevAlive = new List<int>();
            for (var i = 0; i < stages[s - 1].Sols.Count; i++)
                if (cost[s - 1][i] < 1e17) prevAlive.Add(i);
            if (prevAlive.Count > LinkBeamWidth)
            {
                prevAlive.Sort((a, b) => cost[s - 1][a].CompareTo(cost[s - 1][b]));
                prevAlive.RemoveRange(LinkBeamWidth, prevAlive.Count - LinkBeamWidth);
            }
            for (var j = 0; j < stages[s].Sols.Count; j++)
            foreach (var i in prevAlive)
            {
                var (segOk, pen) = SegCheck(stages[s - 1].Sols[i], stages[s].Sols[j], carry, motus);
                if (!segOk) continue;
                var qa = stages[s - 1].Sols[i];
                var qb = stages[s].Sols[j];
                // Only block θ1 hemisphere flips into the release column (not bay→Swing).
                var intoRelease = stages[s].Name is "place:hold" or "place:release";
                if (intoRelease && Math.Abs(Wrap180(qb[0] - qa[0])) > 110) continue;
                var jump = 0.0;
                var leapBad = false;
                for (var k = 0; k < 5; k++)
                {
                    var d = Wrap180(qb[k] - qa[k]);
                    if (intoRelease && Math.Abs(d) > 70) leapBad = true;
                    jump += d * d;
                }
                if (leapBad) continue;
                var c = cost[s - 1][i] + jump + pen;
                if (c < cost[s][j]) { cost[s][j] = c; prev[s][j] = i; }
            }
        }
        var bi = -1; var bc = 1e18;
        for (var j2 = 0; j2 < stages[n - 1].Sols.Count; j2++)
            if (cost[n - 1][j2] < bc) { bc = cost[n - 1][j2]; bi = j2; }
        if (bi < 0)
        {
            var far = -1;
            for (var s2 = 0; s2 < n; s2++)
            for (var j3 = 0; j3 < cost[s2].Length; j3++)
                if (cost[s2][j3] < 1e17) far = s2;
            return (false, 0, new(), far, far + 1 < n ? stages[far + 1].Name : "");
        }
        var seq = new List<(string, double[])>();
        var jj = bi;
        for (var s3 = n - 1; s3 >= 0; s3--)
        {
            seq.Add((stages[s3].Name, stages[s3].Sols[jj].ToArray()));
            jj = prev[s3][jj];
        }
        seq.Reverse();
        return (true, bc, seq, -1, "");
    }

    static List<(string Label, double[] Q, double GR, int Hold)> ChainItems(
        List<(string Name, double[] Q)> seq, double open, double closed)
    {
        var grasp = seq.Last(s => s.Name == "pick:grasp");
        var lift = seq.First(s => s.Name == "Lift");
        var items = new List<(string, double[], double, int)>
        {
            ("Home", BambooIcdMotion.HomeDeg.ToArray(), open, 0)
        };
        foreach (var s in seq.Where(s => s.Name == "Reenter"))
            items.Add(("Reenter", s.Q, open, 0));
        foreach (var s in seq.Where(s => s.Name == "pick:approach"))
            items.Add(("Approach", s.Q, open, 0));
        items.Add(("Grasp", grasp.Q, open, 0));
        items.Add(("Close", grasp.Q.ToArray(), closed, 1));
        items.Add(("Lift", lift.Q, closed, 1));
        (string, double[], double, int)? depart = null;
        (string, double[], double, int)? clear = null;
        (string, double[], double, int)? retract = null;
        var afterLift = false;
        foreach (var s in seq)
        {
            if (s.Name == "Lift") { afterLift = true; continue; }
            if (!afterLift) continue;
            var nm = s.Name;
            if (nm == "Retract") { retract = ("Retract", s.Q, open, 2); continue; }
            if (nm == "Depart") { depart = ("Depart", s.Q, closed, 1); continue; }
            if (nm == "Clear") { clear = ("Clear", s.Q, open, 2); continue; }
            if (nm is "pick:approach" or "pick:grasp") continue;
            var label = nm == "place:hold" ? "Hold" : nm == "place:release" ? "Lower" : nm;
            items.Add((label, s.Q, closed, 1));
        }
        var freezeQ = depart is { } d0 ? d0.Item2 : items.Last(i => i.Item1 is "Lower" or "Hold" or "Swing" or "Pitch").Item2;
        if (depart is { } d) items.Add(("Depart", d.Item2.ToArray(), closed, 1));
        items.Add(("Release", freezeQ.ToArray(), closed, 2));
        items.Add(("Open", freezeQ.ToArray(), open, 2));
        if (clear is { } c) items.Add(c);
        if (retract is { } r) items.Add(r);
        return items;
    }

    static List<BambooIcdMotion.MotionKey> Stamp(List<(string Label, double[] Q, double GR, int Hold)> items)
    {
        var weights = new List<double> { 0 };
        for (var i = 1; i < items.Count; i++)
        {
            var s = 0.0;
            for (var k = 0; k < 5; k++) s += Math.Abs(items[i].Q[k] - items[i - 1].Q[k]);
            s += Math.Abs(items[i].GR - items[i - 1].GR) * 0.35;
            if (items[i].Label is "Close" or "Open") s += 36;
            if (s < 4) s = 4;
            weights.Add(s);
        }
        var total = weights.Sum();
        var acc = 0.0;
        var keys = new List<BambooIcdMotion.MotionKey>();
        for (var i = 0; i < items.Count; i++)
        {
            acc += weights[i];
            keys.Add(new BambooIcdMotion.MotionKey
            {
                T = i == items.Count - 1 ? 1 : acc / total,
                Label = items[i].Label,
                Q = items[i].Q.ToArray(),
                GR = items[i].GR,
                Hold = items[i].Hold
            });
        }
        keys[0].T = 0;
        return keys;
    }

    public static string? MemberIdAt(IReadOnlyList<CycleBound> cycles, double t)
    {
        foreach (var c in cycles)
            if (t >= c.T0 - 1e-12 && t <= c.T1 + 1e-12) return c.MemberId;
        return cycles.Count > 0 ? cycles[^1].MemberId : null;
    }

    public static int MemberIndexAt(IReadOnlyList<CycleBound> cycles, double t)
    {
        for (var i = 0; i < cycles.Count; i++)
            if (t >= cycles[i].T0 - 1e-12 && t <= cycles[i].T1 + 1e-12) return i;
        return Math.Max(0, cycles.Count - 1);
    }
}
