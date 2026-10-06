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
            var one = BuildOne(pick, place, memberId, BambooIcdMotion.HangPose.Pole, placeHang);
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

        // Brachiation: after baseL is planted it is fixed at both ends (ground + pole sleeve) and
        // BambooTrussLayout.WalkHang names that rail. The five-member Motus path stays on the pole hang
        // (compact uprights are reachable); animating a Home hang-teleport onto baseL collides Motus-true
        // with the cane, so the walk is available as a hang target rather than an auto-inserted segment.

        for (var i = 1; i < allKeys.Count; i++)
            if (allKeys[i].T <= allKeys[i - 1].T)
                allKeys[i].T = allKeys[i - 1].T + 1e-6;
        allKeys[^1].T = 1;

        return new TrussBuild { Error = "", Keys = allKeys, Cycles = cycles, Tasks = tasks.ToList() };
    }

    /// <summary>
    /// After baseL is planted it is fixed at both ends (ground + pole sleeve). Steal a small T window
    /// from the joint between cycle 0 and 1 and hang-walk along that upright (Home posture).
    /// </summary>
    static void InsertBrachiate(List<BambooIcdMotion.MotionKey> keys, List<CycleBound> cycles)
    {
        if (cycles.Count < 2) return;
        var tJoin = cycles[0].T1;
        // Shrink later cycles' T to make room for brachiate in (tJoin - span/2, tJoin + span/2).
        const double span = 0.04;
        var t0 = Math.Max(cycles[0].T0 + 1e-3, tJoin - span / 2);
        var t1 = Math.Min(cycles[1].T1 - 1e-3, tJoin + span / 2);
        if (t1 <= t0 + 1e-3) return;

        // Mark cycle 0 end early and shift cycle 1+.
        cycles[0] = new CycleBound
        {
            MemberId = cycles[0].MemberId, T0 = cycles[0].T0, T1 = t0,
            PickHang = cycles[0].PickHang, PlaceHang = cycles[0].PlaceHang
        };
        // Insert a synthetic cycle for the walk so the scan can SetHang along baseL.
        var walk = new CycleBound
        {
            MemberId = "brachiate",
            T0 = t0,
            T1 = t1,
            PickHang = BambooIcdMotion.HangPose.Pole,
            PlaceHang = BambooTrussLayout.WalkHang
        };
        cycles.Insert(1, walk);
        var rem = 1.0 - t1;
        var oldStart = cycles[2].T0;
        for (var i = 2; i < cycles.Count; i++)
        {
            var c = cycles[i];
            var u0 = (c.T0 - oldStart) / Math.Max(1e-9, 1.0 - oldStart);
            var u1 = (c.T1 - oldStart) / Math.Max(1e-9, 1.0 - oldStart);
            cycles[i] = new CycleBound
            {
                MemberId = c.MemberId,
                T0 = t1 + u0 * rem,
                T1 = t1 + u1 * rem,
                PickHang = c.PickHang,
                PlaceHang = c.PlaceHang
            };
        }

        // Re-stamp member keys into new cycle windows; append brachiate Home keys.
        // Simpler: inject Home keys at t0..t1 labelled brachiate:*.
        keys.Add(new BambooIcdMotion.MotionKey
        {
            T = t0, Label = "brachiate:Home", Q = BambooIcdMotion.HomeDeg.ToArray(),
            GR = BambooTrussLayout.JawOpenMm, Hold = 0
        });
        keys.Add(new BambooIcdMotion.MotionKey
        {
            T = (t0 + t1) / 2, Label = "brachiate:Walk", Q = BambooIcdMotion.HomeDeg.ToArray(),
            GR = BambooTrussLayout.JawOpenMm, Hold = 0
        });
        keys.Add(new BambooIcdMotion.MotionKey
        {
            T = t1, Label = "brachiate:Home", Q = BambooIcdMotion.HomeDeg.ToArray(),
            GR = BambooTrussLayout.JawOpenMm, Hold = 0
        });
        keys.Sort((a, b) => a.T.CompareTo(b.T));
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
        BambooIcdMotion.HangPose pickHang, BambooIcdMotion.HangPose placeHang)
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
        foreach (var tilt in new[] { -20.0, -30, -40, -55, -70, -80 })
        {
            var clear = BambooTrussLayout.StrutLength / 2 * Math.Abs(Math.Sin(tilt * Math.PI / 180)) + BambooIcdCell.StrutRadius + 0.02;
            var axis = BambooIcdMotion.StrutAxis(tilt, 0);
            List<double[]> ps = new();
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
            if (ps.Count == 0) continue;
            stages.Add(new Stage { Name = "Pitch", Sols = ps });
        }
        if (stages.Count(s => s.Name == "Pitch") < 2)
            return new BambooIcdMotion.MotionBuild { Error = "transit toward place:hold unreachable (Pitch)", Keys = HomeKeys() };

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
        var chained = LinkStagesDual(stages, pickHang, placeHang);
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

    static (bool Ok, double Penalty) SegCheck(double[] a, double[] b, bool carrying, BambooIcdMotion.HangPose hang)
    {
        var floorBad = false;
        for (var s = 0; s <= 20; s++)
        {
            var u = s / 20.0;
            var q = a.Select((v, k) => v + (b[k] - v) * u).ToArray();
            for (var i = 0; i < 5; i++)
                if (q[i] < BambooIcdMotion.LimitsDeg[i].Min - 0.05 || q[i] > BambooIcdMotion.LimitsDeg[i].Max + 0.05)
                    return (false, 0);
            if (floorBad) continue;
            var f = BambooIcdMotion.FkJaw(q, hang);
            foreach (var pt in f.Pts)
                if (pt[1] < -0.002) floorBad = true;
            if (carrying)
            {
                var ay = f.Z[1];
                var bot = f.Pos[1] - BambooTrussLayout.StrutLength / 2 * Math.Abs(ay) - BambooIcdCell.StrutRadius * Math.Sqrt(Math.Max(0, 1 - ay * ay));
                if (bot < -0.004) floorBad = true;
            }
        }
        return (true, floorBad ? BambooIcdMotion.FloorPenalty : 0);
    }

    static BambooIcdMotion.HangPose HangForStage(string name, BambooIcdMotion.HangPose pickHang, BambooIcdMotion.HangPose placeHang)
    {
        // Transfer and later stages run under the place/walk hang.
        if (name is "Transfer" or "place:hold" or "place:release" or "Depart" or "Retract" or "Hold" or "Lower")
            return placeHang;
        return pickHang;
    }

    static (bool Ok, double Cost, List<(string Name, double[] Q)> Seq, int Far, string Next) LinkStagesDual(
        List<Stage> stages, BambooIcdMotion.HangPose pickHang, BambooIcdMotion.HangPose placeHang)
    {
        var n = stages.Count;
        var cost = stages.Select(st => st.Sols.Select(_ => 1e18).ToArray()).ToArray();
        var prev = stages.Select(st => st.Sols.Select(_ => -1).ToArray()).ToArray();
        for (var i = 0; i < stages[0].Sols.Count; i++)
        {
            var (ok0, pen0) = SegCheck(BambooIcdMotion.HomeDeg, stages[0].Sols[i], false, pickHang);
            if (ok0) cost[0][i] = pen0;
        }
        for (var s = 1; s < n; s++)
        {
            var carry = s >= 2 && stages[s].Name is not ("Retract" or "Depart" or "Clear" or "Transfer");
            // Crossing Transfer: hang frame changes while holding at Home — skip floor check under either hang.
            var hang = HangForStage(stages[s].Name, pickHang, placeHang);
            var hangPrev = HangForStage(stages[s - 1].Name, pickHang, placeHang);
            for (var j = 0; j < stages[s].Sols.Count; j++)
            for (var i = 0; i < stages[s - 1].Sols.Count; i++)
            {
                if (cost[s - 1][i] >= 1e17) continue;
                bool segOk; double pen;
                if (stages[s].Name == "Transfer" || stages[s - 1].Name == "Transfer")
                {
                    // Pure joint check at Home; hang teleport is the brachiation step.
                    (segOk, pen) = (true, 0);
                    for (var k = 0; k < 5; k++)
                    {
                        var q = stages[s].Sols[j][k];
                        if (q < BambooIcdMotion.LimitsDeg[k].Min - 0.05 || q > BambooIcdMotion.LimitsDeg[k].Max + 0.05)
                        { segOk = false; break; }
                    }
                }
                else
                    (segOk, pen) = SegCheck(stages[s - 1].Sols[i], stages[s].Sols[j], carry, hang);
                if (!segOk) continue;
                var jump = 0.0;
                var qa = stages[s - 1].Sols[i];
                var qb = stages[s].Sols[j];
                for (var k = 0; k < 5; k++)
                {
                    var d = Wrap180(qb[k] - qa[k]);
                    jump += d * d;
                }
                // Prefer staying on the same hang when possible.
                if (!hang.Equals(hangPrev)) jump += 50;
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
