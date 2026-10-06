namespace Motus.Viewer.Services;

/// <summary>
/// Motus string-identity tasks and motion keys for /bamboo-truss: each member is a pick (single store
/// feed) → place (vertical upright at an edge midpoint) pair. After the first upright is planted it is
/// fixed at both ends (ground + pole sleeve); later cycles hang from that cane (brachiation) via
/// <see cref="BambooIcdMotion.HangPose"/>.
/// </summary>
public static class BambooTrussMotion
{
    public sealed class CycleBound
    {
        public required string MemberId { get; init; }
        public double T0 { get; init; }
        public double T1 { get; init; }
        /// <summary>Hang used for store pick (always the support pole).</summary>
        public BambooIcdMotion.HangPose PickHang { get; init; } = BambooIcdMotion.HangPose.Pole;
        /// <summary>Hang used for place; after baseL this is the walk rail on that upright.</summary>
        public BambooIcdMotion.HangPose PlaceHang { get; init; } = BambooIcdMotion.HangPose.Pole;
        /// <summary>Place hang (brachiation rail when not on the pole).</summary>
        public BambooIcdMotion.HangPose Hang => PlaceHang;
    }

    public sealed class TrussBuild
    {
        public string Error { get; set; } = "";
        public List<BambooIcdMotion.MotionKey> Keys { get; set; } = new();
        public List<CycleBound> Cycles { get; set; } = new();
        public List<BambooIcdMotion.TaskEdit> Tasks { get; set; } = new();
    }

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

        var allKeys = new List<BambooIcdMotion.MotionKey>();
        var cycles = new List<CycleBound>();
        var n = pairs.Count;
        for (var i = 0; i < n; i++)
        {
            var (pick, place, memberId) = pairs[i];
            var placeHang = BambooTrussLayout.HangForMemberIndex(i);
            // Canes released in earlier cycles stand as uprights; the carried cane must sweep clear of them.
            var placed = new List<BambooIcdCell.StrutPlacement>();
            for (var k = 0; k < i; k++)
                if (PlaneWorld(pairs[k].Place, "release") is { } pr)
                    placed.Add(new BambooIcdCell.StrutPlacement(pr.X, pr.Y, pr.Z, pr.Pitch, pr.Yaw));
            var one = BuildOne(pick, place, memberId, BambooIcdMotion.HangPose.Pole, placeHang, placed);
            if (!string.IsNullOrEmpty(one.Error))
                return new TrussBuild { Error = $"{memberId}: {one.Error}", Tasks = tasks.ToList(), Keys = HomeKeys() };

            var t0 = i / (double)n;
            var t1 = (i + 1) / (double)n;
            cycles.Add(new CycleBound
            {
                MemberId = memberId, T0 = t0, T1 = t1,
                PickHang = BambooIcdMotion.HangPose.Pole, PlaceHang = placeHang
            });
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

        // Brachiation: after baseL is planted (ground + pole sleeve, both ends fixed) the auto path
        // hang-walks along that upright before the remaining members (Motus-clear WalkHang clamp).
        if (n >= 2)
            InsertBrachiate(allKeys, cycles);

        for (var i = 1; i < allKeys.Count; i++)
            if (allKeys[i].T <= allKeys[i - 1].T)
                allKeys[i].T = allKeys[i - 1].T + 1e-6;
        allKeys[^1].T = 1;

        return new TrussBuild { Error = "", Keys = allKeys, Cycles = cycles, Tasks = tasks.ToList() };
    }

    /// <summary>
    /// After baseL is planted it is fixed at both ends (ground + pole sleeve). Insert a Motus-clear
    /// hang-walk along that upright (Home posture on <see cref="BambooTrussLayout.WalkHang"/>) between
    /// cycle 0 and cycle 1, re-stamping every key into the new T windows.
    /// </summary>
    static void InsertBrachiate(List<BambooIcdMotion.MotionKey> keys, List<CycleBound> cycles)
    {
        if (cycles.Count < 2) return;

        // Snapshot each member's keys normalised to [0,1] inside its old cycle window.
        var memberKeys = new List<List<BambooIcdMotion.MotionKey>>();
        foreach (var c in cycles)
        {
            var span = Math.Max(1e-9, c.T1 - c.T0);
            var prefix = c.MemberId + ":";
            var list = new List<BambooIcdMotion.MotionKey>();
            foreach (var k in keys)
            {
                if (!k.Label.StartsWith(prefix, StringComparison.Ordinal)) continue;
                if (k.T < c.T0 - 1e-9 || k.T > c.T1 + 1e-9) continue;
                list.Add(new BambooIcdMotion.MotionKey
                {
                    T = (k.T - c.T0) / span,
                    Label = k.Label,
                    Q = k.Q.ToArray(),
                    GR = k.GR,
                    Hold = k.Hold
                });
            }
            if (list.Count > 0)
            {
                list[0].T = 0;
                list[^1].T = 1;
            }
            memberKeys.Add(list);
        }

        const double brachFrac = 0.06;
        var n = cycles.Count;
        var memberShare = (1.0 - brachFrac) / n;
        var newCycles = new List<CycleBound>();
        var newKeys = new List<BambooIcdMotion.MotionKey>();

        void StampMember(int mi, double t0, double t1)
        {
            var c = cycles[mi];
            newCycles.Add(new CycleBound
            {
                MemberId = c.MemberId, T0 = t0, T1 = t1,
                PickHang = c.PickHang, PlaceHang = c.PlaceHang
            });
            var span = t1 - t0;
            foreach (var k in memberKeys[mi])
            {
                newKeys.Add(new BambooIcdMotion.MotionKey
                {
                    T = t0 + k.T * span,
                    Label = k.Label,
                    Q = k.Q.ToArray(),
                    GR = k.GR,
                    Hold = k.Hold
                });
            }
        }

        // Cycle 0 (baseL), then brachiate walk on the both-ends-fixed rail, then remaining members.
        StampMember(0, 0, memberShare);
        var b0 = memberShare;
        var b1 = memberShare + brachFrac;
        newCycles.Add(new CycleBound
        {
            MemberId = "brachiate",
            T0 = b0,
            T1 = b1,
            PickHang = BambooIcdMotion.HangPose.Pole,
            PlaceHang = BambooTrussLayout.WalkHang
        });
        // Home posture throughout; scan interpolates hang Y along the rail (WalkHangY0→Y1→Y0).
        newKeys.Add(new BambooIcdMotion.MotionKey
        {
            T = b0, Label = "brachiate:Home", Q = BambooIcdMotion.HomeDeg.ToArray(),
            GR = BambooTrussLayout.JawOpenMm, Hold = 0
        });
        newKeys.Add(new BambooIcdMotion.MotionKey
        {
            T = (b0 + b1) / 2, Label = "brachiate:Walk", Q = BambooIcdMotion.HomeDeg.ToArray(),
            GR = BambooTrussLayout.JawOpenMm, Hold = 0
        });
        newKeys.Add(new BambooIcdMotion.MotionKey
        {
            T = b1, Label = "brachiate:Home", Q = BambooIcdMotion.HomeDeg.ToArray(),
            GR = BambooTrussLayout.JawOpenMm, Hold = 0
        });
        for (var mi = 1; mi < n; mi++)
        {
            var t0 = b1 + (mi - 1) * memberShare;
            var t1 = mi == n - 1 ? 1.0 : b1 + mi * memberShare;
            StampMember(mi, t0, t1);
        }

        for (var i = 1; i < newKeys.Count; i++)
            if (newKeys[i].T <= newKeys[i - 1].T)
                newKeys[i].T = newKeys[i - 1].T + 1e-6;
        newKeys[^1].T = 1;

        keys.Clear();
        keys.AddRange(newKeys);
        cycles.Clear();
        cycles.AddRange(newCycles);
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

    static List<double[]> Solve(Pose p, BambooIcdMotion.HangPose hang) =>
        BambooIcdMotion.IkSolveAxis(p.X, p.Y, p.Z, p.Pitch, p.Axis, hang, maxAxisErrDeg: 5);

    static BambooIcdMotion.MotionBuild BuildOne(
        BambooIcdMotion.TaskEdit pick, BambooIcdMotion.TaskEdit place, string memberId,
        BambooIcdMotion.HangPose pickHang, BambooIcdMotion.HangPose placeHang,
        IReadOnlyList<BambooIcdCell.StrutPlacement> placed)
    {
        Pose? Req(BambooIcdMotion.TaskEdit task, string name)
        {
            var pl = task.Planes.FirstOrDefault(p => p.Name == name);
            return pl is null ? null : new Pose(pl.X, pl.Y0 + pl.YMm / 1000.0, pl.Z, pl.PitchDeg, pl.YawDeg);
        }

        var approach = Req(pick, "approach"); var grasp = Req(pick, "grasp");
        var hold = Req(place, "hold"); var release = Req(place, "release");
        if (approach is null || grasp is null || hold is null || release is null)
            return new BambooIcdMotion.MotionBuild { Error = "plane missing", Keys = HomeKeys() };

        var sa = Solve(approach.Value, pickHang);
        var sg = Solve(grasp.Value, pickHang);
        var sh = Solve(hold.Value, placeHang);
        var sr = Solve(release.Value, placeHang);
        var bad = new List<string>();
        if (sa.Count == 0) bad.Add("pick:approach");
        if (sg.Count == 0) bad.Add("pick:grasp");
        if (sh.Count == 0) bad.Add("place:hold");
        if (sr.Count == 0) bad.Add("place:release");
        if (bad.Count > 0)
            return new BambooIcdMotion.MotionBuild { Error = string.Join(", ", bad) + " unreachable", Keys = HomeKeys() };

        var G = grasp.Value; var H = hold.Value; var R = release.Value;

        // Pick under the pole hang, lift and pitch the 0.5 m cane upright, Home while holding,
        // then (if brachiating) continue from Home under the walk hang into the place pose.
        var stages = new List<Stage>
        {
            new() { Name = "pick:approach", Sols = sa },
            new() { Name = "pick:grasp", Sols = sg }
        };

        // Lift under pick hang, then pitch the cane upright while moving toward the place XZ.
        List<double[]>? liftSols = null;
        foreach (var dy in new[] { 0.04, 0.045, 0.035, 0.05, 0.03 })
        {
            liftSols = BambooIcdMotion.IkSolveAxis(G.X, G.Y + dy, G.Z, 0, G.Axis, pickHang, 8);
            if (liftSols.Count > 0) break;
        }
        if (liftSols is null || liftSols.Count == 0)
            return new BambooIcdMotion.MotionBuild { Error = "transit toward place:hold unreachable (Lift)", Keys = HomeKeys() };
        stages.Add(new Stage { Name = "Lift", Sols = liftSols });

        // Pitch upright preferring early transit, expanding toward place XZ only as IK needs.
        // Motus-aware vias: every candidate (transit fraction u, height y) whose carried cane clears
        // the already-planted uprights is offered to the linker, so the chain can swing the 0.5 m cane
        // up where it does not sweep through a neighbouring member (SegCheck vetoes such sweeps).
        foreach (var tilt in new[] { -20.0, -30, -40, -55, -70, -80 })
        {
            var clear = BambooTrussLayout.StrutLength / 2 * Math.Abs(Math.Sin(tilt * Math.PI / 180)) + BambooIcdCell.StrutRadius + 0.02;
            var axis = BambooIcdMotion.StrutAxis(tilt, 0);
            List<double[]> ps = new();
            if (placed.Count == 0)
            {
                foreach (var uMax in new[] { 0.0, 0.15, 0.3, 0.45, 0.55, 0.75, 1.0 })
                {
                    for (var u = 0.0; u <= uMax && ps.Count == 0; u += 0.05)
                    {
                        var x = G.X + (H.X - G.X) * u;
                        var z = G.Z + (H.Z - G.Z) * u;
                        for (var yy = clear; yy <= 0.30; yy += 0.01)
                        {
                            ps = BambooIcdMotion.IkSolveAxis(x, yy, z, tilt, axis, pickHang, 15);
                            if (ps.Count > 0) break;
                        }
                    }
                    if (ps.Count > 0) break;
                }
            }
            else
            {
                // Candidate via XZ: the straight store→place transit, plus the x = G.X column (the
                // feed's own vertical plane, which is clear of every upright since all keep |x| ≥ 0.05).
                var vias = new List<(double X, double Z)>();
                for (var u = 0.0; u <= 1.0 + 1e-9; u += 0.05)
                    vias.Add((G.X + (H.X - G.X) * u, G.Z + (H.Z - G.Z) * u));
                for (var z = G.Z; z <= 0.34 + 1e-9; z += 0.02)
                    vias.Add((G.X, z));
                foreach (var (x, z) in vias)
                for (var yy = clear; yy <= 0.30 + 1e-9; yy += 0.01)
                {
                    foreach (var q in BambooIcdMotion.IkSolveAxis(x, yy, z, tilt, axis, pickHang, 15))
                        if (ViaClear(q, pickHang, placed, CaneMargin)) ps.Add(q);
                }
                if (ps.Count > MaxViaSols)
                {
                    var step = ps.Count / (double)MaxViaSols;
                    ps = Enumerable.Range(0, MaxViaSols).Select(k => ps[(int)(k * step)]).ToList();
                }
            }
            if (ps.Count == 0) continue;
            stages.Add(new Stage { Name = "Pitch", Sols = ps });
        }
        if (placed.Count > 0 && stages.Count(s => s.Name == "Pitch") < 2)
        {
            // No clear via: fall back to the unconstrained transit so the Motus scan names the contact.
            return BuildOne(pick, place, memberId, pickHang, placeHang, Array.Empty<BambooIcdCell.StrutPlacement>());
        }
        if (stages.Count(s => s.Name == "Pitch") < 2)
            return new BambooIcdMotion.MotionBuild { Error = "transit toward place:hold unreachable (Pitch)", Keys = HomeKeys() };

        // Upright via: with the cane already vertical, thread between planted neighbours before the
        // hold. Candidates are vertical-cane poses on an XZ grid spanning feed→place (always including
        // the hold column itself, so the via may collapse to a straight descent onto the hold).
        if (placed.Count > 0)
        {
            var vs = new List<double[]>();
            var up = BambooIcdMotion.StrutAxis(-90, 0);
            var x0 = Math.Min(G.X, H.X) - 0.02; var x1 = Math.Max(G.X, H.X) + 0.02;
            var z0 = Math.Min(G.Z, H.Z) - 0.04; var z1 = Math.Max(G.Z, H.Z) + 0.04;
            for (var x = x0; x <= x1 + 1e-9; x += 0.02)
            for (var z = z0; z <= z1 + 1e-9; z += 0.02)
            for (var yy = H.Y; yy <= 0.36 + 1e-9; yy += 0.01)
                foreach (var q in BambooIcdMotion.IkSolveAxis(x, yy, z, -90, up, pickHang, 15))
                    if (ViaClear(q, pickHang, placed, CaneMargin * 2)) vs.Add(q);
            foreach (var q in BambooIcdMotion.IkSolveAxis(H.X, H.Y + 0.02, H.Z, -90, up, pickHang, 15))
                vs.Add(q);
            if (vs.Count > MaxSwingSols)
            {
                var step = vs.Count / (double)(MaxSwingSols);
                vs = Enumerable.Range(0, MaxSwingSols).Select(k => vs[(int)(k * step)]).ToList();
            }
            // Two via slots (the linker may repeat one pose) so the cane can duck inside a planted
            // neighbour's radius while the arm sweeps past its azimuth, then reach out to the hold.
            if (vs.Count > 0)
            {
                stages.Add(new Stage { Name = "Swing", Sols = vs });
                stages.Add(new Stage { Name = "Swing", Sols = vs });
            }
        }

        // Brachiation transfer via Home only when the hang frame actually changes (walk rail).
        // Pole→pole Home Motus-sweeps the grip carriage through already-planted uprights.
        if (!SameHang(pickHang, placeHang))
            stages.Add(new Stage { Name = "Transfer", Sols = [BambooIcdMotion.HomeDeg.ToArray()] });

        // Into hold/release under the place hang.
        stages.Add(new Stage { Name = "place:hold", Sols = sh });
        stages.Add(new Stage { Name = "place:release", Sols = sr });

        // Clear via: pull open jaw away from the upright before folding Home (Motus-critical for 0.5 m).
        List<double[]>? clearSols = null;
        foreach (var (x, y, z, tilt) in new (double, double, double, double)[]
                 {
                     (0, 0.20, 0.10, 0), (0, 0.18, 0.08, 0), (0, 0.22, 0.12, -30),
                     (R.X * 0.3, 0.22, R.Z * 0.3, -45), (0, 0.16, 0.09, 0)
                 })
        {
            var axis = BambooIcdMotion.StrutAxis(tilt, 0);
            clearSols = BambooIcdMotion.IkSolveAxis(x, y, z, tilt, axis, placeHang, 15);
            if (clearSols.Count > 0) break;
        }
        if (clearSols is { Count: > 0 })
            stages.Add(new Stage { Name = "Clear", Sols = clearSols });
        stages.Add(new Stage { Name = "Retract", Sols = [BambooIcdMotion.HomeDeg.ToArray()] });

        // LinkStages uses pickHang for segments before Transfer and placeHang after.
        var chained = LinkStagesDual(stages, pickHang, placeHang, placed);
        if (!chained.Ok && placed.Count > 0)
        {
            // No obstacle-aware chain (e.g. the target is occupied): fall back to the plain transit and
            // let the Motus scan name the physical contact rather than hiding it behind a build error.
            return BuildOne(pick, place, memberId, pickHang, placeHang, Array.Empty<BambooIcdCell.StrutPlacement>());
        }
        if (!chained.Ok)
            return new BambooIcdMotion.MotionBuild
            {
                Error = string.IsNullOrEmpty(chained.Next) ? "path unlinkable" : chained.Next + " unreachable",
                Keys = HomeKeys()
            };

        var keys = Stamp(ChainItems(chained.Seq, place.JawMm, pick.JawMm));
        return new BambooIcdMotion.MotionBuild { Error = "", Keys = keys };
    }

    static bool SameHang(BambooIcdMotion.HangPose a, BambooIcdMotion.HangPose b) =>
        a.IsPole == b.IsPole && Math.Abs(a.X - b.X) < 1e-9 && Math.Abs(a.Y - b.Y) < 1e-9 && Math.Abs(a.Z - b.Z) < 1e-9;

    const double DepartDy = 0.04;

    sealed class Stage { public string Name = ""; public List<double[]> Sols = new(); }

    static double Wrap180(double a)
    {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    /// <summary>Required surface gap (m) between the carried cane and a planted upright in planning.</summary>
    const double CaneMargin = 0.008;

    /// <summary>Cap on Motus-aware pitch via candidates per tilt stage (keeps the linker cheap).</summary>
    const int MaxViaSols = 120;

    /// <summary>Cap on upright (Swing) via candidates.</summary>
    const int MaxSwingSols = 160;

    /// <summary>Smallest surface gap between the carried cane at <paramref name="q"/> and any planted upright.</summary>
    static bool ViaClear(double[] q, BambooIcdMotion.HangPose hang, IReadOnlyList<BambooIcdCell.StrutPlacement> placed, double margin)
    {
        var f = BambooIcdMotion.FkJaw(q, hang);
        return CaneClearance(f, placed) > margin && GripperClearance(f, placed) >= 0;
    }

    static double CaneClearance(double[] q, BambooIcdMotion.HangPose hang, IReadOnlyList<BambooIcdCell.StrutPlacement> placed) =>
        CaneClearance(BambooIcdMotion.FkJaw(q, hang), placed);

    static double CaneClearance(BambooIcdMotion.FkResult f, IReadOnlyList<BambooIcdCell.StrutPlacement> placed)
    {
        if (placed.Count == 0) return double.MaxValue;
        var h = BambooTrussLayout.StrutLength / 2;
        var a0 = new[] { f.Pos[0] - f.Z[0] * h, f.Pos[1] - f.Z[1] * h, f.Pos[2] - f.Z[2] * h };
        var a1 = new[] { f.Pos[0] + f.Z[0] * h, f.Pos[1] + f.Z[1] * h, f.Pos[2] + f.Z[2] * h };
        var best = double.MaxValue;
        foreach (var p in placed)
        {
            var ax = p.Axis;
            var b0 = new[] { p.X - ax.X * h, p.Y - ax.Y * h, p.Z - ax.Z * h };
            var b1 = new[] { p.X + ax.X * h, p.Y + ax.Y * h, p.Z + ax.Z * h };
            best = Math.Min(best, SegSeg(a0, a1, b0, b1) - 2 * BambooIcdCell.StrutRadius);
        }
        return best;
    }

    /// <summary>Approximate link half-thickness (m) of the URDF arm boxes (≈0.041×0.061 links).</summary>
    const double LinkRadius = 0.03;
    /// <summary>Linker cost for a segment whose arm skeleton grazes a planted upright (Motus has the final say).</summary>
    const double ArmPenalty = 2e5;

    /// <summary>Smallest gap between the arm skeleton (joint polyline) at <paramref name="q"/> and any planted upright.</summary>
    static double ArmClearance(BambooIcdMotion.FkResult f, IReadOnlyList<BambooIcdCell.StrutPlacement> placed)
    {
        if (placed.Count == 0) return double.MaxValue;
        var pts = f.Pts;
        var h = BambooTrussLayout.StrutLength / 2;
        var best = double.MaxValue;
        foreach (var p in placed)
        {
            var ax = p.Axis;
            var b0 = new[] { p.X - ax.X * h, p.Y - ax.Y * h, p.Z - ax.Z * h };
            var b1 = new[] { p.X + ax.X * h, p.Y + ax.Y * h, p.Z + ax.Z * h };
            for (var i = 2; i < pts.Count; i++)
                best = Math.Min(best, SegSeg(pts[i - 1], pts[i], b0, b1) - BambooIcdCell.StrutRadius - LinkRadius);
        }
        return best;
    }

    /// <summary>Gripper envelope radius (m) around the jaw point: finger rails are 0.08 m long.</summary>
    const double GripperRadius = 0.045;

    /// <summary>Gap between the gripper envelope (sphere about the jaw) and any planted upright.</summary>
    static double GripperClearance(BambooIcdMotion.FkResult f, IReadOnlyList<BambooIcdCell.StrutPlacement> placed)
    {
        var j = f.Pos;
        var h = BambooTrussLayout.StrutLength / 2;
        var best = double.MaxValue;
        foreach (var p in placed)
        {
            var ax = p.Axis;
            var b0 = new[] { p.X - ax.X * h, p.Y - ax.Y * h, p.Z - ax.Z * h };
            var b1 = new[] { p.X + ax.X * h, p.Y + ax.Y * h, p.Z + ax.Z * h };
            best = Math.Min(best, SegSeg(j, j, b0, b1) - BambooIcdCell.StrutRadius - GripperRadius);
        }
        return best;
    }

    /// <summary>Closest distance between segments p0–p1 and q0–q1.</summary>
    static double SegSeg(double[] p0, double[] p1, double[] q0, double[] q1)
    {
        double Dot(double[] a, double[] b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        var d1 = new[] { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
        var d2 = new[] { q1[0] - q0[0], q1[1] - q0[1], q1[2] - q0[2] };
        var r = new[] { p0[0] - q0[0], p0[1] - q0[1], p0[2] - q0[2] };
        double a = Dot(d1, d1), e = Dot(d2, d2), f = Dot(d2, r), c = Dot(d1, r), b = Dot(d1, d2);
        var den = a * e - b * b;
        double s, t;
        if (a < 1e-12) { s = 0; t = Math.Clamp(f / e, 0, 1); }
        else
        {
            s = den > 1e-12 ? Math.Clamp((b * f - c * e) / den, 0, 1) : 0;
            t = (b * s + f) / e;
            if (t < 0) { t = 0; s = Math.Clamp(-c / a, 0, 1); }
            else if (t > 1) { t = 1; s = Math.Clamp((b - c) / a, 0, 1); }
        }
        var dx = p0[0] + d1[0] * s - (q0[0] + d2[0] * t);
        var dy = p0[1] + d1[1] * s - (q0[1] + d2[1] * t);
        var dz = p0[2] + d1[2] * s - (q0[2] + d2[2] * t);
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    static (bool Ok, double Penalty) SegCheck(double[] a, double[] b, bool carrying, BambooIcdMotion.HangPose hang,
        IReadOnlyList<BambooIcdCell.StrutPlacement>? placed = null)
    {
        var floorBad = false;
        var armBad = false;
        for (var s = 0; s <= 20; s++)
        {
            var u = s / 20.0;
            var q = a.Select((v, k) => v + (b[k] - v) * u).ToArray();
            for (var i = 0; i < 5; i++)
                if (q[i] < BambooIcdMotion.LimitsDeg[i].Min - 0.05 || q[i] > BambooIcdMotion.LimitsDeg[i].Max + 0.05)
                    return (false, 0);
            var f = BambooIcdMotion.FkJaw(q, hang);
            if (placed is { Count: > 0 })
            {
                var grip = GripperClearance(f, placed);
                // Carried cane (and the gripper around it) must not sweep through a planted upright.
                if (carrying && (grip < 0 || CaneClearance(f, placed) < CaneMargin * 0.5))
                    return (false, 0);
                if (!armBad && (grip < 0 || ArmClearance(f, placed) < 0)) armBad = true;
            }
            if (floorBad) continue;
            foreach (var pt in f.Pts)
                if (pt[1] < -0.002) floorBad = true;
            if (carrying)
            {
                var ay = f.Z[1];
                var bot = f.Pos[1] - BambooTrussLayout.StrutLength / 2 * Math.Abs(ay) - BambooIcdCell.StrutRadius * Math.Sqrt(Math.Max(0, 1 - ay * ay));
                if (bot < -0.004) floorBad = true;
            }
        }
        return (true, (floorBad ? BambooIcdMotion.FloorPenalty : 0) + (armBad ? ArmPenalty : 0));
    }

    static BambooIcdMotion.HangPose HangForStage(string name, BambooIcdMotion.HangPose pickHang, BambooIcdMotion.HangPose placeHang)
    {
        // Transfer and later stages run under the place/walk hang.
        if (name is "Transfer" or "place:hold" or "place:release" or "Depart" or "Retract" or "Hold" or "Lower")
            return placeHang;
        return pickHang;
    }

    static (bool Ok, double Cost, List<(string Name, double[] Q)> Seq, int Far, string Next) LinkStagesDual(
        List<Stage> stages, BambooIcdMotion.HangPose pickHang, BambooIcdMotion.HangPose placeHang,
        IReadOnlyList<BambooIcdCell.StrutPlacement> placed)
    {
        var n = stages.Count;
        var cost = stages.Select(st => st.Sols.Select(_ => 1e18).ToArray()).ToArray();
        var prev = stages.Select(st => st.Sols.Select(_ => -1).ToArray()).ToArray();
        for (var i = 0; i < stages[0].Sols.Count; i++)
        {
            var (ok0, pen0) = SegCheck(BambooIcdMotion.HomeDeg, stages[0].Sols[i], false, pickHang, placed);
            if (ok0) cost[0][i] = pen0;
        }
        for (var s = 1; s < n; s++)
        {
            var carry = s >= 2 && stages[s].Name is not ("Retract" or "Depart" or "Clear" or "Transfer");
            // Crossing Transfer: hang frame changes while holding at Home — skip floor check under either hang.
            var hang = HangForStage(stages[s].Name, pickHang, placeHang);
            var hangPrev = HangForStage(stages[s - 1].Name, pickHang, placeHang);
            var transfer = stages[s].Name == "Transfer" || stages[s - 1].Name == "Transfer";
            for (var j = 0; j < stages[s].Sols.Count; j++)
            {
                var qb = stages[s].Sols[j];
                // Lower bound (cost so far + joint jump) per predecessor; penalties are ≥ 0, so the
                // expensive swept check only runs while a predecessor can still beat the best found.
                var cand = new List<(int I, double Lb)>();
                for (var i = 0; i < stages[s - 1].Sols.Count; i++)
                {
                    if (cost[s - 1][i] >= 1e17) continue;
                    var qa = stages[s - 1].Sols[i];
                    var jump = 0.0;
                    for (var k = 0; k < 5; k++)
                    {
                        var d = Wrap180(qb[k] - qa[k]);
                        jump += d * d;
                    }
                    // Prefer staying on the same hang when possible.
                    if (!hang.Equals(hangPrev)) jump += 50;
                    cand.Add((i, cost[s - 1][i] + jump));
                }
                foreach (var (i, lb) in cand.OrderBy(c => c.Lb))
                {
                    if (lb >= cost[s][j]) break;
                    bool segOk; double pen;
                    if (transfer)
                    {
                        // Pure joint check at Home; hang teleport is the brachiation step.
                        (segOk, pen) = (true, 0);
                        for (var k = 0; k < 5; k++)
                        {
                            var q = qb[k];
                            if (q < BambooIcdMotion.LimitsDeg[k].Min - 0.05 || q > BambooIcdMotion.LimitsDeg[k].Max + 0.05)
                            { segOk = false; break; }
                        }
                    }
                    else
                        (segOk, pen) = SegCheck(stages[s - 1].Sols[i], qb, carry, hang, placed);
                    if (!segOk) continue;
                    var c = lb + pen;
                    if (c < cost[s][j]) { cost[s][j] = c; prev[s][j] = i; }
                }
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
        var items = new List<(string, double[], double, int)>
        {
            ("Home", BambooIcdMotion.HomeDeg.ToArray(), open, 0),
            ("Approach", seq[0].Q, open, 0),
            ("Grasp", seq[1].Q, open, 0),
            ("Close", seq[1].Q.ToArray(), closed, 1),
            ("Lift", seq[2].Q, closed, 1)
        };
        (string, double[], double, int)? depart = null;
        (string, double[], double, int)? retract = null;
        for (var k = 3; k < seq.Count; k++)
        {
            var nm = seq[k].Name;
            if (nm == "Retract") { retract = ("Retract", seq[k].Q, open, 2); continue; }
            if (nm == "Depart") { depart = ("Depart", seq[k].Q, open, 2); continue; }
            if (nm is "Swing" or "Pitch" or "Transfer")
            {
                items.Add((nm, seq[k].Q, closed, 1));
                continue;
            }
            if (nm == "Clear") { /* added after Open below */ continue; }
            var label = nm == "place:hold" ? "Hold" : nm == "place:release" ? "Lower" : nm;
            items.Add((label, seq[k].Q, closed, 1));
        }
        var lowerQ = seq[1].Q;
        for (var li = items.Count - 1; li >= 0; li--)
            if (items[li].Item1 == "Lower") { lowerQ = items[li].Item2; break; }
        items.Add(("Open", lowerQ.ToArray(), open, 2));
        foreach (var (name, q) in seq)
            if (name == "Clear") items.Add(("Clear", q, open, 2));
        if (depart is { } d) items.Add(d);
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
