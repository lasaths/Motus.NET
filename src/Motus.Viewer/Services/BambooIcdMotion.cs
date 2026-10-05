using Motus.Core;

namespace Motus.Viewer.Services;

/// <summary>
/// ICD/LIS bamboo-arm planar IK and pick/place motion keys for the Motus.Viewer page.
/// Matches the retired HTML preview kinematics (SEG lengths, axes, HOME_Q) and the bamboo URDF
/// (tests assert Motus FK == <see cref="FkJaw"/>). The planner's floor heuristics only pick IK branches;
/// collision is decided by Motus.NET (<see cref="BambooIcdCell"/>, <see cref="BambooMotionScan"/>).
/// Frame: Three.js Y-up viewer space (pole along +X at y=POLE_Y). Motus world is Z-up,
/// Motus = Rx(+90°)·viewer; joint angles are shared 1:1 (same θ vector).
/// </summary>
public static class BambooIcdMotion
{
    public static readonly double Deg = Math.PI / 180.0;
    public static readonly double PoleY = 0.40;
    public static readonly double[] Seg = { 0.030, 0.080, 0.088, 0.088, 0.088, 0.046 };
    public static readonly (double Min, double Max)[] LimitsDeg =
    {
        (-170, 170), (-90, 90), (-135, 135), (-90, 90), (-170, 170)
    };
    public static readonly double[] HomeDeg = { -90, -72, -80, -24, 0 };
    public static readonly (double X, double Y, double Z) Pick = (0, 0.00778, 0.09194);
    public static readonly (double X, double Y, double Z) Place = (0, 0.1506, 0.2000);

    public sealed class PlaneEdit
    {
        public string Name { get; set; } = "";
        public double X { get; set; }
        public double Y0 { get; set; }
        public double Z { get; set; }
        public double PitchDeg { get; set; }
        public double YMm { get; set; }
    }

    public sealed class TaskEdit
    {
        public string Identity { get; set; } = "pick";
        public string Object { get; set; } = "strut";
        public double JawMm { get; set; } = 40;
        public List<PlaneEdit> Planes { get; set; } = new();
    }

    public sealed class MotionKey
    {
        public double T { get; set; }
        public string Label { get; set; } = "Home";
        public double[] Q { get; set; } = HomeDeg.ToArray();
        public double GR { get; set; } = 40;
        public int Hold { get; set; }
    }

    public sealed class MotionBuild
    {
        public string Error { get; set; } = "";
        public List<MotionKey> Keys { get; set; } = new();
    }

    /// <summary>Open R-grip opening used by a motion (the widest gR in its keys).</summary>
    public static double OpenJawMm(IReadOnlyList<MotionKey> keys) => keys.Count == 0 ? 40 : keys.Max(k => k.GR);

    public static List<TaskEdit> DefaultTasks() =>
    [
        new TaskEdit
        {
            Identity = "pick",
            Object = "strut",
            JawMm = 14,
            Planes =
            [
                new PlaneEdit { Name = "approach", X = Pick.X, Y0 = Pick.Y + 0.040, Z = Pick.Z, PitchDeg = 0, YMm = 0 },
                new PlaneEdit { Name = "grasp", X = Pick.X, Y0 = Pick.Y, Z = Pick.Z, PitchDeg = 0, YMm = 0 }
            ]
        },
        new TaskEdit
        {
            Identity = "place",
            Object = "strut",
            JawMm = 40,
            Planes =
            [
                new PlaneEdit { Name = "hold", X = Place.X, Y0 = Place.Y + 0.100, Z = Place.Z, PitchDeg = -90, YMm = -80 },
                new PlaneEdit { Name = "release", X = Place.X, Y0 = Place.Y, Z = Place.Z, PitchDeg = -90, YMm = 0 }
            ]
        }
    ];

    public static TaskPath ToTaskPath(IReadOnlyList<TaskEdit> tasks)
    {
        var list = new List<TaskInstance>();
        foreach (var t in tasks)
        {
            var frames = new Dictionary<string, Frame>();
            foreach (var p in t.Planes)
                frames[p.Name] = PlaneToMotusFrame(p.X, p.Y0 + p.YMm / 1000.0, p.Z, p.PitchDeg);
            list.Add(new TaskInstance(t.Identity, frames, new Dictionary<string, object>
            {
                ["object"] = t.Object,
                ["jaw_mm"] = t.JawMm
            }));
        }
        return new TaskPath(list);
    }

    /// <summary>
    /// Task plane (viewer Y-up position + pitch about X) → Motus Z-up TCP frame.
    /// Viewer jaw orientation at a plane is Rx(pitch); Motus = Rx(+90°)·viewer ⇒ Motus rotation Rx(90° + pitch),
    /// position (x, −z, y). This is exactly the bamboo URDF tool0 pose when the arm grasps the plane.
    /// </summary>
    public static Frame PlaneToMotusFrame(double x, double y, double z, double pitchDeg)
    {
        var half = (90.0 + pitchDeg) * Deg / 2;
        return new Frame(x, -z, y, qw: Math.Cos(half), qx: Math.Sin(half), qy: 0, qz: 0);
    }

    static double Wrap180(double a)
    {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    public static List<double[]> IkSolve(double y, double z, double alphaDeg)
    {
        var alpha = alphaDeg * Deg;
        var L = 0.088;
        var Ljaw = 0.088 + 0.046;
        var y2 = PoleY - Seg[0] - Seg[1];
        var Ay = y2 - y - Ljaw * Math.Cos(alpha);
        var Az = -z - Ljaw * Math.Sin(alpha);
        var r = Math.Sqrt(Ay * Ay + Az * Az);
        var sols = new List<double[]>();
        if (r > 2 * L + 1e-5) return sols;
        var off = Math.Acos(Math.Clamp(r / (2 * L), -1, 1));
        var psi = Math.Atan2(Az, Ay);
        for (var s = 0; s < 2; s++)
        {
            var sign = s == 0 ? 1 : -1;
            var beta = psi + sign * off;
            var gamma = psi - sign * off;
            var q = new[]
            {
                -90.0,
                Wrap180(beta / Deg),
                Wrap180((gamma - beta) / Deg),
                Wrap180(alphaDeg - gamma / Deg),
                0.0
            };
            var ok = true;
            for (var i = 0; i < 5; i++)
            {
                if (q[i] < LimitsDeg[i].Min - 1e-3 || q[i] > LimitsDeg[i].Max + 1e-3) ok = false;
                q[i] = Math.Clamp(q[i], LimitsDeg[i].Min, LimitsDeg[i].Max);
            }
            if (!ok) continue;
            var f = FkJaw(q);
            if (Math.Sqrt((f.Pos[1] - y) * (f.Pos[1] - y) + (f.Pos[2] - z) * (f.Pos[2] - z)) > 0.002) continue;
            var eZ1 = -Math.Sin(alpha);
            var eZ2 = Math.Cos(alpha);
            if (Math.Sqrt(f.Z[0] * f.Z[0] + (f.Z[1] - eZ1) * (f.Z[1] - eZ1) + (f.Z[2] - eZ2) * (f.Z[2] - eZ2)) > 0.05) continue;
            sols.Add(q);
        }
        return sols;
    }

    /// <summary>
    /// Spatial IK for a viewer-frame TCP at (x,y,z) with pitch about X: solve the planar arm at
    /// radial distance r = hypot(x,z), then yaw θ1 so the working plane faces the target.
    /// θ5 is aligned so the jaw Z matches the strut axis of a placement with the same pitch
    /// (PlacementWorld uses Rx(pitch) only — axis (0, −sin pitch, cos pitch)). When x = 0 and
    /// z ≥ 0 this matches <see cref="IkSolve"/> (θ5 stays 0).
    /// </summary>
    public static List<double[]> IkSolveSpatial(double x, double y, double z, double pitchDeg)
    {
        var r = Math.Sqrt(x * x + z * z);
        if (r < 1e-9) return new List<double[]>();
        var planar = IkSolve(y, r, pitchDeg);
        var yawDeg = Math.Atan2(x, z) / Deg;
        var pitch = pitchDeg * Deg;
        // Expected jaw/strut Z for PlacementWorld(Rx(pitch)).
        var axis = new[] { 0.0, -Math.Sin(pitch), Math.Cos(pitch) };
        var sols = new List<double[]>();
        foreach (var s in planar)
        {
            var q = s.ToArray();
            q[0] = Math.Clamp(-90.0 + yawDeg, LimitsDeg[0].Min, LimitsDeg[0].Max);
            AlignWristToAxis(q, axis);
            var f = FkJaw(q);
            if (Math.Sqrt((f.Pos[0] - x) * (f.Pos[0] - x) + (f.Pos[1] - y) * (f.Pos[1] - y) + (f.Pos[2] - z) * (f.Pos[2] - z)) > 0.005)
                continue;
            sols.Add(q);
        }
        return sols;
    }

    /// <summary>Set θ5 (about jaw Y) so jaw Z is closest to ±<paramref name="axis"/>.</summary>
    static void AlignWristToAxis(double[] q, double[] axis)
    {
        var best = q[4];
        var bestDot = -1.0;
        for (var t5 = LimitsDeg[4].Min; t5 <= LimitsDeg[4].Max; t5 += 2.5)
        {
            q[4] = t5;
            var z = FkJaw(q).Z;
            var dot = Math.Abs(z[0] * axis[0] + z[1] * axis[1] + z[2] * axis[2]);
            if (dot > bestDot) { bestDot = dot; best = t5; }
        }
        // Refine ±2.5°
        var lo = Math.Max(LimitsDeg[4].Min, best - 2.5);
        var hi = Math.Min(LimitsDeg[4].Max, best + 2.5);
        for (var t5 = lo; t5 <= hi; t5 += 0.5)
        {
            q[4] = t5;
            var z = FkJaw(q).Z;
            var dot = Math.Abs(z[0] * axis[0] + z[1] * axis[1] + z[2] * axis[2]);
            if (dot > bestDot) { bestDot = dot; best = t5; }
        }
        q[4] = best;
    }

    public readonly struct FkResult
    {
        public double[] Pos { get; init; }
        public double[] Z { get; init; }
        public List<double[]> Pts { get; init; }
    }

    public static FkResult FkJaw(double[] deg)
    {
        var q = deg.Select(d => d * Deg).ToArray();
        var R = Ry(90 * Deg);
        var p = new[] { 0.0, PoleY, 0.0 };
        var axes = new[] { "Y", "X", "X", "X", "Y" };
        var pts = new List<double[]> { (double[])p.Clone() };
        for (var i = 0; i < 5; i++)
        {
            p = Add(p, Apply(R, new[] { 0.0, -Seg[i], 0.0 }));
            pts.Add((double[])p.Clone());
            R = Mul(R, axes[i] == "Y" ? Ry(q[i]) : Rx(q[i]));
        }
        var jaw = Add(p, Apply(R, new[] { 0.0, -Seg[5], 0.0 }));
        pts.Add(jaw);
        return new FkResult { Pos = jaw, Z = Apply(R, new[] { 0.0, 0.0, 1.0 }), Pts = pts };
    }

    static double[] Rx(double a)
    {
        var c = Math.Cos(a); var s = Math.Sin(a);
        return [1, 0, 0, 0, c, -s, 0, s, c];
    }
    static double[] Ry(double a)
    {
        var c = Math.Cos(a); var s = Math.Sin(a);
        return [c, 0, s, 0, 1, 0, -s, 0, c];
    }
    static double[] Mul(double[] A, double[] B)
    {
        var C = new double[9];
        for (var r = 0; r < 3; r++)
            for (var c = 0; c < 3; c++)
                C[r * 3 + c] = A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
        return C;
    }
    static double[] Apply(double[] R, double[] v) =>
        [R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
         R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
         R[6] * v[0] + R[7] * v[1] + R[8] * v[2]];
    static double[] Add(double[] a, double[] b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];

    /// <summary>Cost added per segment that breaks the floor heuristics (branch preference, not a veto).</summary>
    public const double FloorPenalty = 1e6;

    /// <summary>
    /// Joint limits are a hard veto (kinematic feasibility). The point-model floor / carried-strut checks
    /// from the retired HTML preview only steer IK branch choice via <see cref="FloorPenalty"/>: they never
    /// reject a path, so a colliding request still plays and Motus.NET stops it on the first hit.
    /// </summary>
    static (bool Ok, double Penalty) SegCheck(double[] a, double[] b, bool carrying)
    {
        var floorBad = false;
        for (var s = 0; s <= 20; s++)
        {
            var u = s / 20.0;
            var q = a.Select((v, k) => v + (b[k] - v) * u).ToArray();
            for (var i = 0; i < 5; i++)
                if (q[i] < LimitsDeg[i].Min - 0.05 || q[i] > LimitsDeg[i].Max + 0.05) return (false, 0);
            if (floorBad) continue;
            var f = FkJaw(q);
            foreach (var pt in f.Pts)
                if (pt[1] < -0.002) floorBad = true;
            if (carrying)
            {
                var ay = f.Z[1];
                var bot = f.Pos[1] - 0.15 * Math.Abs(ay) - 0.007 * Math.Sqrt(Math.Max(0, 1 - ay * ay));
                if (bot < -0.004) floorBad = true;
            }
        }
        return (true, floorBad ? FloorPenalty : 0);
    }

    sealed class Stage { public string Name = ""; public List<double[]> Sols = new(); }

    static (bool Ok, double Cost, List<(string Name, double[] Q)> Seq, int Far, string Next) LinkStages(List<Stage> stages)
    {
        var n = stages.Count;
        var cost = stages.Select(st => st.Sols.Select(_ => 1e18).ToArray()).ToArray();
        var prev = stages.Select(st => st.Sols.Select(_ => -1).ToArray()).ToArray();
        for (var i = 0; i < stages[0].Sols.Count; i++)
        {
            var (ok0, pen0) = SegCheck(HomeDeg, stages[0].Sols[i], false);
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
            ("Home", HomeDeg.ToArray(), open, 0),
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

    static List<MotionKey> Stamp(List<(string Label, double[] Q, double GR, int Hold)> items)
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
        var keys = new List<MotionKey>();
        for (var i = 0; i < items.Count; i++)
        {
            acc += weights[i];
            keys.Add(new MotionKey
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

    static List<MotionKey> HomeKeys(double gR) =>
    [
        new MotionKey { T = 0, Label = "Home", Q = HomeDeg.ToArray(), GR = gR, Hold = 0 },
        new MotionKey { T = 1, Label = "Home", Q = HomeDeg.ToArray(), GR = gR, Hold = 0 }
    ];

    static TaskEdit? First(IReadOnlyList<TaskEdit> tasks, string id) =>
        tasks.FirstOrDefault(t => t.Identity == id);

    static (double X, double Y, double Z, double A)? PlaneWorld(TaskEdit? task, string name)
    {
        var pl = task?.Planes.FirstOrDefault(p => p.Name == name);
        if (pl is null) return null;
        return (pl.X, pl.Y0 + pl.YMm / 1000.0, pl.Z, pl.PitchDeg);
    }

    static List<double[]> ElbowSols(double y, double z, double a)
    {
        var sols = IkSolve(y, z, a);
        var mild = sols.Where(q => Math.Abs(q[2]) <= 88).ToList();
        return mild.Count > 0 ? mild : sols;
    }

    public static MotionBuild BuildMotion(IReadOnlyList<TaskEdit> tasks)
    {
        var pick = First(tasks, "pick");
        var place = First(tasks, "place");
        if (pick is null || place is null)
            return new MotionBuild { Error = $"No task with identity \"{(pick is null ? "pick" : "place")}\".", Keys = HomeKeys(40) };
        if (!double.IsFinite(pick.JawMm) || !double.IsFinite(place.JawMm))
            return new MotionBuild { Error = "jawMm must be a number.", Keys = HomeKeys(40) };

        var want = new (string Plane, TaskEdit Task, string Tag)[]
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
            if (w is null) return new MotionBuild { Error = tag + " missing.", Keys = HomeKeys(place.JawMm) };
            pose[plane] = w.Value;
            if (Math.Abs(w.Value.X) > 1e-6) { bad.Add(tag); continue; }
            var sols = IkSolve(w.Value.Y, w.Value.Z, w.Value.A);
            if (sols.Count == 0) bad.Add(tag);
            else solved[plane] = sols;
        }
        if (bad.Count > 0)
            return new MotionBuild { Error = string.Join(", ", bad) + " unreachable", Keys = HomeKeys(place.JawMm) };

        var G = pose["grasp"]; var H = pose["hold"]; var R = pose["release"];
        var tuned = new List<Stage>
        {
            new() { Name = "pick:approach", Sols = solved["approach"] },
            new() { Name = "pick:grasp", Sols = solved["grasp"] }
        };
        var viaPts = new (string Name, double Y, double Z, double A)[]
        {
            ("Lift", G.Y + 0.07222, G.Z + 0.04806, G.A),
            ("Pitch", 0.16, 0.22, -30),
            ("Pitch", 0.18, 0.26, -45),
            ("Pitch", 0.22, 0.26, -60),
            ("Pitch", 0.20, 0.26, -75)
        };
        var viaOk = true;
        (double Cost, List<(string Name, double[] Q)> Seq)? tunedFallback = null;
        foreach (var vp in viaPts)
        {
            var vs = ElbowSols(vp.Y, vp.Z, vp.A);
            if (vs.Count == 0) { viaOk = false; break; }
            tuned.Add(new Stage { Name = vp.Name, Sols = vs });
        }
        if (viaOk)
        {
            tuned.Add(new Stage { Name = "place:hold", Sols = solved["hold"] });
            tuned.Add(new Stage { Name = "place:release", Sols = solved["release"] });
            var dep = ElbowSols(R.Y + 0.0294, R.Z - 0.14, R.A);
            if (dep.Count > 0) tuned.Add(new Stage { Name = "Depart", Sols = dep });
            tuned.Add(new Stage { Name = "Retract", Sols = [HomeDeg.ToArray()] });
            var tunedChain = LinkStages(tuned);
            if (tunedChain.Ok && tunedChain.Cost < FloorPenalty)
            {
                var keys = Stamp(ChainItems(tunedChain.Seq, place.JawMm, pick.JawMm));
                return new MotionBuild { Error = "", Keys = keys };
            }
            if (tunedChain.Ok) tunedFallback = (tunedChain.Cost, tunedChain.Seq);
        }

        var lifts = new List<( (double Y, double Z, double A) P, List<double[]> Sols)>();
        foreach (var zg in new[] { 0.25, 0.4, 0.55, 0.7, 0.15 })
        foreach (var yl in new[] { 0.02, 0.035, 0.05, 0.065 })
        {
            var lp = (Y: G.Y + yl, Z: G.Z + zg * (H.Z - G.Z), A: G.A);
            var ls = IkSolve(lp.Y, lp.Z, lp.A);
            if (ls.Count > 0) lifts.Add((lp, ls));
        }
        if (lifts.Count == 0)
        {
            if (tunedFallback is { } tf0)
                return new MotionBuild { Error = "", Keys = Stamp(ChainItems(tf0.Seq, place.JawMm, pick.JawMm)) };
            return new MotionBuild { Error = "lift toward place:hold unreachable", Keys = HomeKeys(place.JawMm) };
        }

        var candidates = new List<(bool Ok, double Cost, List<(string Name, double[] Q)> Seq)>();
        var failFar = -1; var failNext = "";
        foreach (var lift in lifts)
        {
            var stages = new List<Stage>
            {
                new() { Name = "pick:approach", Sols = solved["approach"] },
                new() { Name = "pick:grasp", Sols = solved["grasp"] },
                new() { Name = "Lift", Sols = lift.Sols }
            };
            var pitchOk = true;
            var lp0 = lift.P;
            for (var pi = 1; pi <= 3; pi++)
            {
                var u = pi / 4.0;
                var pp = (
                    Y: lp0.Y + (H.Y - lp0.Y) * u,
                    Z: lp0.Z + (H.Z - lp0.Z) * u,
                    A: lp0.A + (H.A - lp0.A) * u);
                var ps = IkSolve(pp.Y, pp.Z, pp.A);
                if (ps.Count == 0) { pitchOk = false; break; }
                stages.Add(new Stage { Name = "Pitch", Sols = ps });
            }
            if (!pitchOk)
            {
                if (failFar < 2) { failFar = 2; failNext = "lift toward place:hold"; }
                continue;
            }
            stages.Add(new Stage { Name = "place:hold", Sols = solved["hold"] });
            stages.Add(new Stage { Name = "place:release", Sols = solved["release"] });
            stages.Add(new Stage { Name = "Retract", Sols = [HomeDeg.ToArray()] });
            var chained = LinkStages(stages);
            if (!chained.Ok)
            {
                if (chained.Far > failFar) { failFar = chained.Far; failNext = chained.Next; }
                continue;
            }
            candidates.Add((true, chained.Cost, chained.Seq));
        }
        if (tunedFallback is { } tf) candidates.Add((true, tf.Cost, tf.Seq));
        if (candidates.Count == 0)
        {
            var msg = "lift toward place:hold unreachable";
            if (failFar < 0) msg = "pick:approach unreachable";
            else if (failNext is "pick:grasp" or "place:hold" or "place:release") msg = failNext + " unreachable";
            else if (failNext == "lift toward place:hold") msg = "lift toward place:hold unreachable";
            return new MotionBuild { Error = msg, Keys = HomeKeys(place.JawMm) };
        }
        candidates.Sort((a, b) => a.Cost.CompareTo(b.Cost));
        return new MotionBuild
        {
            Error = "",
            Keys = Stamp(ChainItems(candidates[0].Seq, place.JawMm, pick.JawMm))
        };
    }

    public static (double[] Q, double GR, int Hold, string Label) Sample(IReadOnlyList<MotionKey> keys, double t)
    {
        if (keys.Count == 0) return (HomeDeg.ToArray(), 40, 0, "Home");
        if (t <= 0) return (keys[0].Q.ToArray(), keys[0].GR, keys[0].Hold, keys[0].Label);
        if (t >= 1)
        {
            var last = keys[^1];
            return (last.Q.ToArray(), last.GR, last.Hold, last.Label);
        }
        var i = 0;
        while (i < keys.Count - 2 && keys[i + 1].T < t) i++;
        var a = keys[i]; var b = keys[i + 1];
        var denom = b.T - a.T;
        var u = denom <= 1e-8 ? 1 : (t - a.T) / denom;
        var e = u * u * (3 - 2 * u);
        var q = a.Q.Select((v, k) => v + (b.Q[k] - v) * e).ToArray();
        return (q, a.GR + (b.GR - a.GR) * e, u >= 1 ? b.Hold : a.Hold, e < 0.5 ? a.Label : b.Label);
    }
}
