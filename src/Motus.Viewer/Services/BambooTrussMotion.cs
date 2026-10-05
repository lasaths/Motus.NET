namespace Motus.Viewer.Services;

/// <summary>
/// Motus string-identity tasks and motion keys for /bamboo-truss: each member is a pick (store
/// slot) → place (truss edge midpoint) pair. Spatial IK via <see cref="BambooIcdMotion.IkSolveSpatial"/>.
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

    public static List<BambooIcdMotion.TaskEdit> DefaultTasks()
    {
        var tasks = new List<BambooIcdMotion.TaskEdit>();
        foreach (var m in BambooTrussLayout.Members)
        {
            tasks.Add(new BambooIcdMotion.TaskEdit
            {
                Identity = "pick",
                Object = m.Id,
                JawMm = BambooTrussLayout.JawClosedMm,
                Planes =
                [
                    new BambooIcdMotion.PlaneEdit
                    {
                        Name = "approach",
                        X = m.Store.X,
                        // Keep approach reachable: higher stack slots use a shorter lift (cap ≤ 0.062).
                        Y0 = Math.Min(0.062, m.Store.Y + BambooTrussLayout.StoreApproachDy),
                        Z = m.Store.Z,
                        PitchDeg = m.Store.PitchDeg,
                        YMm = 0
                    },
                    new BambooIcdMotion.PlaneEdit
                    {
                        Name = "grasp",
                        X = m.Store.X,
                        Y0 = m.Store.Y,
                        Z = m.Store.Z,
                        PitchDeg = m.Store.PitchDeg,
                        YMm = 0
                    }
                ]
            });
            tasks.Add(new BambooIcdMotion.TaskEdit
            {
                Identity = "place",
                Object = m.Id,
                JawMm = BambooTrussLayout.JawOpenMm,
                Planes =
                [
                    new BambooIcdMotion.PlaneEdit
                    {
                        Name = "hold",
                        X = m.Place.X,
                        Y0 = m.Place.Y + BambooTrussLayout.PlaceHoldDy,
                        Z = m.Place.Z,
                        PitchDeg = m.Place.PitchDeg,
                        YMm = 0
                    },
                    new BambooIcdMotion.PlaneEdit
                    {
                        Name = "release",
                        X = m.Place.X,
                        Y0 = m.Place.Y,
                        Z = m.Place.Z,
                        PitchDeg = m.Place.PitchDeg,
                        YMm = 0
                    }
                ]
            });
        }
        return tasks;
    }

    /// <summary>Pick/place task pairs in member order (object name = member id).</summary>
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
            var one = BuildOne(pick, place, memberId);
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

        // Ensure strictly increasing T (duplicate boundaries from cycle joins).
        for (var i = 1; i < allKeys.Count; i++)
            if (allKeys[i].T <= allKeys[i - 1].T)
                allKeys[i].T = allKeys[i - 1].T + 1e-6;
        allKeys[^1].T = 1;

        return new TrussBuild { Error = "", Keys = allKeys, Cycles = cycles, Tasks = tasks.ToList() };
    }

    static List<BambooIcdMotion.MotionKey> HomeKeys() =>
    [
        new() { T = 0, Label = "Home", Q = BambooIcdMotion.HomeDeg.ToArray(), GR = BambooTrussLayout.JawOpenMm, Hold = 0 },
        new() { T = 1, Label = "Home", Q = BambooIcdMotion.HomeDeg.ToArray(), GR = BambooTrussLayout.JawOpenMm, Hold = 0 }
    ];

    static (double X, double Y, double Z, double A)? PlaneWorld(BambooIcdMotion.TaskEdit task, string name)
    {
        var pl = task.Planes.FirstOrDefault(p => p.Name == name);
        if (pl is null) return null;
        return (pl.X, pl.Y0 + pl.YMm / 1000.0, pl.Z, pl.PitchDeg);
    }

    static BambooIcdMotion.MotionBuild BuildOne(BambooIcdMotion.TaskEdit pick, BambooIcdMotion.TaskEdit place, string memberId)
    {
        var want = new (string Plane, BambooIcdMotion.TaskEdit Task, string Tag)[]
        {
            ("approach", pick, "pick:approach"),
            ("grasp", pick, "pick:grasp"),
            ("hold", place, "place:hold"),
            ("release", place, "place:release")
        };
        var pose = new Dictionary<string, (double X, double Y, double Z, double A)>();
        var solved = new Dictionary<string, List<double[]>>();
        var bad = new List<string>();
        foreach (var (plane, task, tag) in want)
        {
            var w = PlaneWorld(task, plane);
            if (w is null) return new BambooIcdMotion.MotionBuild { Error = tag + " missing.", Keys = HomeKeys() };
            pose[plane] = w.Value;
            var sols = BambooIcdMotion.IkSolveSpatial(w.Value.X, w.Value.Y, w.Value.Z, w.Value.A);
            if (sols.Count == 0) bad.Add(tag);
            else solved[plane] = sols;
        }
        if (bad.Count > 0)
            return new BambooIcdMotion.MotionBuild { Error = string.Join(", ", bad) + " unreachable", Keys = HomeKeys() };

        // Planar-only pairs reuse the battle-tested ICD pick/place builder (clear under Motus).
        if (Math.Abs(pose["grasp"].X) < 1e-6 && Math.Abs(pose["release"].X) < 1e-6
            && Math.Abs(pose["approach"].X) < 1e-6 && Math.Abs(pose["hold"].X) < 1e-6)
        {
            var planar = BambooIcdMotion.BuildMotion(new[] { pick, place });
            if (string.IsNullOrEmpty(planar.Error))
                return planar;
            // fall through to spatial builder if the planar heuristic path fails
        }

        var G = pose["grasp"];
        var H = pose["hold"];
        var R = pose["release"];

        var stages = new List<Stage>
        {
            new() { Name = "pick:approach", Sols = solved["approach"] },
            new() { Name = "pick:grasp", Sols = solved["grasp"] }
        };

        // Lift above grasp within the pitch-0 reachable band (y ≲ 0.065 at store Z), then climb
        // into a high pitch-down corridor toward hold. Linear (x,y,z,pitch) lerp hits an IK hole.
        var liftSols = BambooIcdMotion.IkSolveSpatial(G.X, Math.Min(G.Y + 0.05, 0.062), G.Z, G.A);
        if (liftSols.Count == 0)
            liftSols = BambooIcdMotion.IkSolveSpatial(G.X, Math.Min(G.Y + 0.02, 0.062), G.Z, G.A);
        if (liftSols.Count == 0)
            liftSols = BambooIcdMotion.IkSolveSpatial(G.X, 0.10, G.Z + 0.04, -30);
        if (liftSols.Count == 0)
            liftSols = BambooIcdMotion.IkSolveSpatial(G.X, 0.12, 0.12, -45);
        if (liftSols.Count == 0)
            return new BambooIcdMotion.MotionBuild { Error = "lift unreachable", Keys = HomeKeys() };
        stages.Add(new Stage { Name = "Lift", Sols = liftSols });

        var vias = new (string Name, double X, double Y, double Z, double A)[]
        {
            ("Pitch", G.X + (H.X - G.X) * 0.25, 0.14, G.Z + (H.Z - G.Z) * 0.25, G.A + (H.A - G.A) * 0.25),
            ("Pitch", G.X + (H.X - G.X) * 0.4, 0.18, G.Z + (H.Z - G.Z) * 0.45, -75),
            ("Pitch", H.X, 0.20, H.Z, -90),
            ("Pitch", H.X, Math.Max(H.Y + 0.02, 0.18), H.Z, H.A)
        };
        foreach (var vp in vias)
        {
            var ps = BambooIcdMotion.IkSolveSpatial(vp.X, vp.Y, vp.Z, vp.A);
            if (ps.Count == 0)
            {
                // Fallbacks around the same corridor.
                foreach (var (fy, fa) in new[] { (0.18, -75.0), (0.20, -90.0), (0.22, -90.0), (0.18, -90.0) })
                {
                    ps = BambooIcdMotion.IkSolveSpatial(vp.X, fy, vp.Z, fa);
                    if (ps.Count > 0) break;
                    ps = BambooIcdMotion.IkSolveSpatial(H.X, fy, H.Z, fa);
                    if (ps.Count > 0) break;
                }
            }
            if (ps.Count == 0)
                return new BambooIcdMotion.MotionBuild { Error = $"transit toward place:hold unreachable ({vp.Name})", Keys = HomeKeys() };
            stages.Add(new Stage { Name = vp.Name, Sols = ps });
        }

        stages.Add(new Stage { Name = "place:hold", Sols = solved["hold"] });
        stages.Add(new Stage { Name = "place:release", Sols = solved["release"] });

        var dep = BambooIcdMotion.IkSolveSpatial(R.X, R.Y + 0.04, R.Z - 0.04, R.A);
        if (dep.Count == 0) dep = BambooIcdMotion.IkSolveSpatial(R.X, R.Y + 0.03, R.Z, R.A);
        if (dep.Count > 0) stages.Add(new Stage { Name = "Depart", Sols = dep });
        stages.Add(new Stage { Name = "Retract", Sols = [BambooIcdMotion.HomeDeg.ToArray()] });

        var chained = LinkStages(stages);
        if (!chained.Ok)
            return new BambooIcdMotion.MotionBuild
            {
                Error = string.IsNullOrEmpty(chained.Next) ? "path unlinkable" : chained.Next + " unreachable",
                Keys = HomeKeys()
            };

        var keys = Stamp(ChainItems(chained.Seq, place.JawMm, pick.JawMm));
        return new BambooIcdMotion.MotionBuild { Error = "", Keys = keys };
    }

    // ---- planner helpers (same rules as BambooIcdMotion: limits hard, floor = preference) ----

    sealed class Stage { public string Name = ""; public List<double[]> Sols = new(); }

    static double Wrap180(double a)
    {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    static (bool Ok, double Penalty) SegCheck(double[] a, double[] b, bool carrying)
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
            var f = BambooIcdMotion.FkJaw(q);
            foreach (var pt in f.Pts)
                if (pt[1] < -0.002) floorBad = true;
            if (carrying)
            {
                var ay = f.Z[1];
                var bot = f.Pos[1] - 0.15 * Math.Abs(ay) - 0.007 * Math.Sqrt(Math.Max(0, 1 - ay * ay));
                if (bot < -0.004) floorBad = true;
            }
        }
        return (true, floorBad ? BambooIcdMotion.FloorPenalty : 0);
    }

    static (bool Ok, double Cost, List<(string Name, double[] Q)> Seq, int Far, string Next) LinkStages(List<Stage> stages)
    {
        var n = stages.Count;
        var cost = stages.Select(st => st.Sols.Select(_ => 1e18).ToArray()).ToArray();
        var prev = stages.Select(st => st.Sols.Select(_ => -1).ToArray()).ToArray();
        for (var i = 0; i < stages[0].Sols.Count; i++)
        {
            var (ok0, pen0) = SegCheck(BambooIcdMotion.HomeDeg, stages[0].Sols[i], false);
            if (ok0) cost[0][i] = pen0;
        }
        for (var s = 1; s < n; s++)
        {
            var carry = s >= 2 && stages[s].Name != "Retract" && stages[s].Name != "Depart";
            for (var j = 0; j < stages[s].Sols.Count; j++)
            for (var i = 0; i < stages[s - 1].Sols.Count; i++)
            {
                if (cost[s - 1][i] >= 1e17) continue;
                var (segOk, pen) = SegCheck(stages[s - 1].Sols[i], stages[s].Sols[j], carry);
                if (!segOk) continue;
                var jump = 0.0;
                var qa = stages[s - 1].Sols[i];
                var qb = stages[s].Sols[j];
                for (var k = 0; k < 5; k++)
                {
                    var d = Wrap180(qb[k] - qa[k]);
                    jump += d * d;
                }
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
            var label = nm == "place:hold" ? "Hold" : nm == "place:release" ? "Lower" : nm;
            items.Add((label, seq[k].Q, closed, 1));
        }
        var lowerQ = seq[1].Q;
        for (var li = items.Count - 1; li >= 0; li--)
            if (items[li].Item1 == "Lower") { lowerQ = items[li].Item2; break; }
        items.Add(("Open", lowerQ.ToArray(), open, 2));
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
