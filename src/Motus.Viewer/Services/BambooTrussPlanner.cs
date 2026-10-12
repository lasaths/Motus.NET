using Motus.Core;
using Motus.Geometry;
using K = Motus.Viewer.Services.BambooGripKinematics;

namespace Motus.Viewer.Services;

/// <summary>
/// Grounded, base-swapping planner for /bamboo-truss. At every instant at least one gripper is clamped on a real
/// fixed object and is the IK base; the arm only ever moves by changing joint angles (no rigid moves of the whole
/// robot). Grips are solved with <see cref="BambooGripKinematics.SolveGrip"/> to within
/// <see cref="BambooGripKinematics.GripPosTolM"/> / <see cref="BambooGripKinematics.GripAxisTolDeg"/>.
/// Every segment is Motus-checked (robot links, both grippers, held cane, mount parts vs environment) at
/// <see cref="VerifyStepDeg"/> before it is accepted.
/// </summary>
public sealed class BambooTrussPlanner
{
    /// <summary>Motion rules (PTP-only, park after place, sample resolutions, …) this planner follows.</summary>
    public MotionConstraints Constraints { get; }

    /// <summary>Joint step for the coarse via search (degrees) — <see cref="MotionConstraints.SearchStepDeg"/>.</summary>
    public double SearchStepDeg => Constraints.SearchStepDeg;
    /// <summary>Joint step every accepted segment is verified at (degrees) — <see cref="MotionConstraints.VerifyStepDeg"/>.</summary>
    public double VerifyStepDeg => Constraints.VerifyStepDeg;
    /// <summary>Pre-grasp / retreat distance along the gripper's own +Y (metres).</summary>
    public const double ApproachM = 0.04;
    /// <summary>
    /// Point-to-point only (Lassie: "the robot can only move point to point") — <see cref="MotionConstraints.PointToPointOnly"/>:
    /// every move is a joint-space PTP segment between IK waypoints (all joints interpolated, densely Motus-checked;
    /// via-points / RRT in joint space when the direct segment hits).
    /// </summary>
    bool Ptp => Constraints.PointToPointOnly;

    /// <summary>
    /// Placed members a gripper may clamp (walk on): with <see cref="MotionConstraints.FixedMemberRule"/> only the
    /// members of the rigid anchored subframe (<see cref="BambooTrussLayout.FixedBothEnds"/>), else every placed member.
    /// </summary>
    public IReadOnlyList<string> Clampable(IEnumerable<string> placedIds) =>
        Constraints.FixedMemberRule ? BambooTrussLayout.FixedBothEnds(placedIds)
        : placedIds.Where(id => BambooTrussLayout.Members.Any(m => m.Id == id)).ToList();

    /// <summary>PTP waypoint: the IK solution for the free gripper at <paramref name="t"/> backed off
    /// <paramref name="backoff"/> along its own +Y, nearest to the current joints (any branch).</summary>
    double[]? PtpIk(K.GripTarget t, double backoff, int axisSign) =>
        K.SolveGrip(_base, _baseWorld, t, new[] { _q }, 16, backoff, axisSign: axisSign).OrderBy(x => K.JointDist(x, _q)).FirstOrDefault();

    /// <summary>
    /// PTP path to <paramref name="qg"/>: the direct joint segment, else via a PTP waypoint at the pose backed off
    /// along the free gripper's Y (<paramref name="backoffs"/>; the waypoint → target is one joint-interpolated
    /// segment, densely checked), else via-points / RRT in joint space. Returns the vias (without qg) or null.
    /// </summary>
    List<double[]>? PtpPath(double[] qg, K.GripTarget target, int sign, params double[] backoffs)
    {
        if (MoveOk(_q, qg, SearchStepDeg) && MoveOk(_q, qg, VerifyStepDeg)) return new List<double[]>();
        foreach (var b in backoffs)
        {
            var qv = K.SolveGrip(_base, _baseWorld, target, new[] { qg }, 8, b, axisSign: sign).OrderBy(x => K.JointDist(x, qg)).FirstOrDefault();
            if (qv is null || K.JointDist(qv, qg) > 45) continue;
            if (!PoseOk(qv, _gR, _gL, _hold, _lOn, _rOn) || !MoveOk(qv, qg, VerifyStepDeg)) continue;
            var pv = TransitPath(_q, qv);
            if (pv is null) continue;
            pv.Add(qv);
            return pv;
        }
        return TransitPath(_q, qg);
    }

    /// <summary>Optional PTP departure from a released object: one joint segment to the pose backed off 4 / 2 cm, if
    /// reachable and Motus-clear; otherwise nothing (the next transit leaves from here, Motus-checked).</summary>
    void PtpDepart(K.GripTarget from, string label, string? lOn, string? rOn)
    {
        var sign = FreeAxisSign(from, _q);
        foreach (var d in new[] { ApproachM, -ApproachM, ApproachM / 2, -ApproachM / 2 })
        {
            var q = PtpIk(from, d, sign);
            if (q is null || K.JointDist(q, _q) > 60) continue;
            if (!SegOk(_q, q, _gR, _gR, _gL, _gL, _hold, _hold, lOn, rOn, VerifyStepDeg)) continue;
            Emit(label, q);
            return;
        }
    }
    /// <summary>Cartesian sub-step for approach / lift / slide moves (metres).</summary>
    public const double CartStepM = 0.01;

    public sealed record GripRecord(string Label, string Gripper, string On, double PosM, double AxisDeg);

    readonly BambooIcdCell _cell;
    readonly ReplayRandom _rng = new(11);

    /// <summary>
    /// Seeded random stream whose position is part of a deep checkpoint: rolling back also rewinds the stream, so a
    /// branch that was tried and abandoned leaves no trace and a re-run of the same calls is bit-identical.
    /// </summary>
    sealed class ReplayRandom(int seed)
    {
        Random _r = new(seed);
        readonly List<int> _log = new(); // -1 = NextDouble, otherwise Next(max)
        public int Position => _log.Count;
        public double NextDouble() { _log.Add(-1); return _r.NextDouble(); }
        public int Next(int max) { _log.Add(max); return _r.Next(max); }
        public void Rewind(int position)
        {
            if (position >= _log.Count) return;
            _r = new Random(seed);
            for (var i = 0; i < position; i++) { if (_log[i] < 0) _r.NextDouble(); else _r.Next(_log[i]); }
            _log.RemoveRange(position, _log.Count - position);
        }
    }

    K.Gripper _base = K.Gripper.L;
    double[] _baseWorld = BambooTrussLayout.StartBase;
    string _baseOn = BambooTrussLayout.S1.Name;
    double[] _q = new double[5];
    double _gR = BambooTrussLayout.JawOpenMm, _gL = BambooTrussLayout.JawClosedMm;
    int _hold;
    string? _lOn = BambooTrussLayout.S1.Name, _rOn;
    Frame? _held;
    double[]? _activeRest, _activeReleased;
    string? _activeId;
    readonly List<(string Name, double[] World)> _placed = new();
    string _cycle = "start";
    string _lStake = BambooTrussLayout.S1.Name;

    public List<BambooIcdMotion.MotionKey> Keys { get; } = new();
    public List<(string Cycle, int First, int Last)> CycleKeys { get; } = new();
    public List<GripRecord> Grips { get; } = new();
    public List<string> Notes { get; } = new();
    public string Error { get; private set; } = "";
    public int Checks { get; private set; }
    /// <summary>Why candidate grips were rejected (diagnostics for notes / tests).</summary>
    public List<string> Diag { get; } = new();

    public BambooTrussPlanner(BambooIcdCell cell, MotionConstraints? constraints = null)
    {
        _cell = cell;
        Constraints = (constraints ?? MotionConstraints.Current).Validate();
    }

    // ---------------- world ----------------

    List<CollisionObject>? _env; int _envKey = int.MinValue;
    List<CollisionObject> Env(int hold)
    {
        var key = HashCode.Combine(hold, _placed.Count, _activeId, _activeReleased is null);
        if (_env is not null && key == _envKey) return _env;
        // Between members (start, walk) the stock is full: the next cane lies on the pegs (activeId null).
        var env = BambooTrussCollision.Environment(_cell, _placed, _activeId, _activeId is null ? null : _activeRest, _activeReleased, heldTracked: true, hold);
        _envKey = key;
        return _env = env;
    }

    /// <summary>Allowed touches: jaw plates of each gripper on what it clamps (and only that).</summary>
    public static List<(string A, string B)> Allowed(string? lOn, string? rOn) => BambooTrussCollision.Allowed(lOn, rOn);

    /// <summary>L-grip (mount) world frame, viewer coordinates, for a base and joint angles.</summary>
    public static double[] LWorld(string? baseGripper, double[] baseWorld, IReadOnlyList<double> q) =>
        baseGripper == "R" ? K.Mul(baseWorld, K.Inv(K.Tlr(q))) : baseWorld;

    public static double[] RWorld(string? baseGripper, double[] baseWorld, IReadOnlyList<double> q) =>
        baseGripper == "R" ? baseWorld : K.Mul(baseWorld, K.Tlr(q));

    public static double[] ToMotus(double[] viewer) => Transforms.Multiply(BambooIcdCell.ViewerToMotus, viewer);

    string BaseName => _base == K.Gripper.L ? "L" : "R";
    double[] LNow(IReadOnlyList<double> q) => LWorld(BaseName, _baseWorld, q);

    public readonly System.Diagnostics.Stopwatch PoseWatch = new();
    bool PoseOk(double[] q, double gR, double gL, int hold, string? lOn, string? rOn)
    {
        Checks++;
        PoseWatch.Start();
        var ok = BambooTrussCollision.Contacts(_cell, BaseName, _baseWorld, q, gR, gL, hold == 1 ? _held : null, Env(hold), lOn, rOn, 1).Count == 0;
        PoseWatch.Stop();
        return ok;
    }

    public IReadOnlyList<CollisionContact> ContactsNow(double[] q, double gR, double gL, int hold, string? lOn, string? rOn)
    {
        return BambooTrussCollision.Contacts(_cell, BaseName, _baseWorld, q, gR, gL, hold == 1 ? _held : null, Env(hold), lOn, rOn, 8);
    }

    /// <summary>Segment from the current state to (qb, gRb, gLb): intermediate samples keep the current hold.</summary>
    bool SegOk(double[] qa, double[] qb, double gRa, double gRb, double gLa, double gLb, int holdA, int holdB,
        string? lOn, string? rOn, double step)
    {
        var dq = K.JointDist(qa, qb);
        var dg = Math.Max(Math.Abs(gRb - gRa), Math.Abs(gLb - gLa));
        var n = Math.Max(1, (int)Math.Ceiling(Math.Max(dq / step, dg / (step <= VerifyStepDeg ? Constraints.JawStepMm : Constraints.SearchJawStepMm))));
        var q = new double[5];
        for (var s = 1; s <= n; s++)
        {
            var u = s / (double)n;
            for (var k = 0; k < 5; k++) q[k] = qa[k] + (qb[k] - qa[k]) * u;
            if (!PoseOk(q, gRa + (gRb - gRa) * u, gLa + (gLb - gLa) * u, s == n ? holdB : holdA, lOn, rOn)) return false;
        }
        // The verifying pass also checks exactly the canonical samples BambooTrussScan takes on this segment
        // (BambooTrussCollision.CanonicalCount), with the end key's hold, so whatever it accepts the scan accepts.
        if (step <= VerifyStepDeg)
        {
            var ns = BambooTrussCollision.CanonicalCount(dq, dg, Constraints);
            for (var s = 1; s < ns; s++)
            {
                var u = s / (double)ns;
                for (var k = 0; k < 5; k++) q[k] = qa[k] + (qb[k] - qa[k]) * u;
                if (!PoseOk(q, gRa + (gRb - gRa) * u, gLa + (gLb - gLa) * u, holdB, lOn, rOn)) return false;
            }
        }
        return true;
    }

    /// <summary>Joint step of the playback / test scan — <see cref="MotionConstraints.ScanStepDeg"/>.</summary>
    public double ScanStepDeg => Constraints.ScanStepDeg;

    bool MoveOk(double[] qa, double[] qb, double step) =>
        SegOk(qa, qb, _gR, _gR, _gL, _gL, _hold, _hold, _lOn, _rOn, step);

    // ---------------- keys ----------------

    void Emit(string label, double[] q, double? gR = null, double? gL = null, int? hold = null)
    {
        _q = q.ToArray();
        if (gR is { } r) _gR = r;
        if (gL is { } l) _gL = l;
        if (hold is { } h) _hold = h;
        Keys.Add(new BambooIcdMotion.MotionKey
        {
            Label = $"{_cycle}:{label}",
            Q = _q.ToArray(),
            GR = _gR,
            GL = _gL,
            Hold = _hold,
            Base = BaseName,
            BaseWorld = _baseWorld.ToArray(),
            BaseOn = _baseOn,
            LOn = _lOn,
            ROn = _rOn
        });
    }

    void BeginCycle(string name)
    {
        if (CycleKeys.Count > 0)
        {
            var last = CycleKeys[^1];
            CycleKeys[^1] = (last.Cycle, last.First, Keys.Count - 1);
        }
        _cycle = name;
        CycleKeys.Add((name, Keys.Count, Keys.Count - 1));
    }

    void EndCycles()
    {
        if (CycleKeys.Count == 0) return;
        var last = CycleKeys[^1];
        CycleKeys[^1] = (last.Cycle, last.First, Keys.Count - 1);
    }

    bool Fail(string why) { Error = why; Diag.Add("  fail: " + why); return false; }

    // ---------------- motion primitives ----------------

    /// <summary>
    /// Joint-space transit to <paramref name="qb"/>: direct if Motus-clear, else a few fold vias, else bidirectional
    /// RRT-Connect (Motus-checked edges) with shortcutting. Every returned edge is verified at <see cref="VerifyStepDeg"/>.
    /// </summary>
    List<double[]>? TransitPath(double[] qa, double[] qb)
    {
        bool Seg(double[] a, double[] b) => MoveOk(a, b, SearchStepDeg) && MoveOk(a, b, VerifyStepDeg);
        if (Seg(qa, qb)) return new List<double[]>();
        foreach (var t in new[] { 0.5 })
        foreach (var (b2, b3, b4) in new[] { (0.0, 0.0, 0.0), (45.0, -90.0, 45.0), (-45.0, 90.0, -45.0), (30.0, -60.0, 30.0), (-30.0, 60.0, -30.0) })
        {
            var v = new double[5];
            for (var k = 0; k < 5; k++) v[k] = qa[k] + (qb[k] - qa[k]) * t;
            v[1] = b2; v[2] = b3; v[3] = b4;
            if (!PoseOk(v, _gR, _gL, _hold, _lOn, _rOn)) continue;
            if (Seg(qa, v) && Seg(v, qb)) return new List<double[]> { v };
        }
        // Up to three RRT-Connect attempts (the shared seeded RNG moves on, so each attempt explores differently).
        for (var i = 0; i < RrtAttempts; i++)
            if (Rrt(qa, qb) is { } path) return path;
        return null;
    }

    /// <summary>RRT-Connect attempts per transit (walk searches lower it: a failing candidate step costs every attempt).</summary>
    public int RrtAttempts { get; set; } = 3;

    /// <summary>RRT-Connect iterations per attempt (walk searches lower it).</summary>
    public int RrtIters { get; set; } = 1500;
    const double RrtStepDeg = 8;

    static double Dist2(double[] a, double[] b)
    {
        var s = 0.0;
        for (var k = 0; k < 5; k++) { var d = a[k] - b[k]; s += d * d; }
        return s;
    }

    List<double[]>? Rrt(double[] qa, double[] qb)
    {
        var ta = new List<double[]> { qa }; var pa = new List<int> { -1 };
        var tb = new List<double[]> { qb }; var pb = new List<int> { -1 };
        bool Edge(double[] a, double[] b) => MoveOk(a, b, SearchStepDeg);
        double[] Steer(double[] from, double[] to, double step)
        {
            var d = K.JointDist(from, to);
            if (d <= step) return to.ToArray();
            var v = new double[5];
            for (var k = 0; k < 5; k++) v[k] = from[k] + (to[k] - from[k]) * step / d;
            return v;
        }
        int Nearest(List<double[]> tree, double[] q)
        {
            var best = 0; var bd = double.MaxValue;
            for (var i = 0; i < tree.Count; i++) { var d = Dist2(tree[i], q); if (d < bd) { bd = d; best = i; } }
            return best;
        }
        // Extend: returns index of new node or -1 (trapped).
        int Extend(List<double[]> tree, List<int> par, double[] target)
        {
            var n = Nearest(tree, target);
            var q = Steer(tree[n], target, RrtStepDeg);
            if (!PoseOk(q, _gR, _gL, _hold, _lOn, _rOn) || !Edge(tree[n], q)) return -1;
            tree.Add(q); par.Add(n);
            return tree.Count - 1;
        }
        var swapped = false;
        for (var it = 0; it < RrtIters; it++)
        {
            var sample = new double[5];
            for (var k = 0; k < 5; k++)
            {
                var (lo, hi) = BambooIcdMotion.LimitsDeg[k];
                sample[k] = lo + (hi - lo) * _rng.NextDouble();
            }
            var na = Extend(ta, pa, sample);
            if (na >= 0)
            {
                // Connect: greedily grow the other tree toward the new node.
                var target = ta[na];
                var nb = -1;
                while (true)
                {
                    var n = Nearest(tb, target);
                    if (K.JointDist(tb[n], target) < 1e-9) { nb = n; break; }
                    var q = Steer(tb[n], target, RrtStepDeg);
                    if (!PoseOk(q, _gR, _gL, _hold, _lOn, _rOn) || !Edge(tb[n], q)) break;
                    tb.Add(q); pb.Add(n);
                    if (K.JointDist(q, target) < 1e-9) { nb = tb.Count - 1; break; }
                }
                if (nb >= 0)
                {
                    var pathA = new List<double[]>(); for (var i = na; i >= 0; i = pa[i]) pathA.Add(ta[i]);
                    pathA.Reverse();
                    var pathB = new List<double[]>(); for (var i = pb[nb]; i >= 0; i = pb[i]) pathB.Add(tb[i]);
                    var full = pathA.Concat(pathB).ToList();
                    if (swapped) full.Reverse();
                    var smooth = Shortcut(full);
                    if (smooth is not null) return smooth.Skip(1).Take(smooth.Count - 2).ToList();
                }
            }
            (ta, tb) = (tb, ta); (pa, pb) = (pb, pa); swapped = !swapped;
        }
        return null;
    }

    /// <summary>Shortcut an RRT path (coarse-checked) and verify every remaining edge at <see cref="VerifyStepDeg"/>.</summary>
    List<double[]>? Shortcut(List<double[]> path)
    {
        var p = path.ToList();
        for (var tries = 0; tries < 60 && p.Count > 2; tries++)
        {
            var i = _rng.Next(p.Count - 2); var j = i + 2 + _rng.Next(p.Count - i - 2);
            if (MoveOk(p[i], p[j], SearchStepDeg) && MoveOk(p[i], p[j], VerifyStepDeg)) p.RemoveRange(i + 1, j - i - 1);
        }
        // Greedy pass: from each node jump as far as possible.
        var outp = new List<double[]> { p[0] };
        var a = 0;
        while (a < p.Count - 1)
        {
            var b = p.Count - 1;
            while (b > a + 1 && !(MoveOk(p[a], p[b], SearchStepDeg) && MoveOk(p[a], p[b], VerifyStepDeg))) b--;
            if (b == a + 1 && !MoveOk(p[a], p[b], VerifyStepDeg)) return null;
            outp.Add(p[b]); a = b;
        }
        return outp;
    }

    double[] Perturb(double[] q, double sigma)
    {
        var v = new double[5];
        for (var k = 0; k < 5; k++)
        {
            var (lo, hi) = BambooIcdMotion.LimitsDeg[k];
            v[k] = Math.Clamp(q[k] + (2 * _rng.NextDouble() - 1) * sigma, lo, hi);
        }
        return v;
    }

    bool Transit(string label, double[] qb)
    {
        var path = TransitPath(_q, qb);
        if (path is null) return false;
        foreach (var v in path) Emit(label, v);
        Emit(label, qb);
        return true;
    }

    /// <summary>
    /// Cartesian sequence for the free gripper: one IK per <paramref name="targets"/> entry, each seeded from the
    /// previous joints (continuous branch, ≤ 15° per step). Null if any step is unreachable or jumps.
    /// </summary>
    List<double[]>? Cartesian(double[] from, IEnumerable<(K.GripTarget T, double Backoff)> targets, int axisSign)
    {
        var path = new List<double[]>();
        var cur = from;
        foreach (var (t, b) in targets)
        {
            var next = K.SolveGrip(_base, _baseWorld, t, new[] { cur }, 0, b, axisSign: axisSign)
                .OrderBy(x => K.JointDist(x, cur)).FirstOrDefault();
            if (next is null || K.JointDist(next, cur) > 15) return null;
            path.Add(next);
            cur = next;
        }
        return path;
    }

    int FreeAxisSign(K.GripTarget t, double[] q)
    {
        var g = K.Free(_base, _baseWorld, q);
        return g[2] * t.Ax + g[6] * t.Ay + g[10] * t.Az >= 0 ? 1 : -1;
    }

    void RecordGrip(string label, string gripper, K.GripTarget t, double[] frame)
    {
        var (p, a) = K.Residual(frame, t);
        Grips.Add(new GripRecord($"{_cycle}:{label}", gripper, t.On, p, a));
    }

    /// <summary>
    /// R-grip (free, L-base) moves to a cane grip: transit → pre-grasp (backed off <see cref="ApproachM"/> along its +Y,
    /// jaws open) → straight-line approach → grasp → close. <paramref name="onName"/> is the clamped object's scene name.
    /// </summary>
    bool RGrasp(K.GripTarget target, string onName, int axisSign, bool pickActive, int solutionIndex = -1)
    {
        if (_base != K.Gripper.L) return Fail("R-grasp needs the L-grip as base");
        var open = BambooTrussLayout.JawOpenMm;
        var sols = K.SolveGrip(_base, _baseWorld, target, new[] { _q }, 32, axisSign: axisSign)
            .OrderBy(x => PreferKey(x)).Take(8).ToList();
        if (solutionIndex >= 0) sols = sols.Skip(solutionIndex).Take(1).ToList();
        Diag.Add($"{_cycle}: R grasp {onName} ({target.X:F3},{target.Y:F3},{target.Z:F3}): {sols.Count} IK solutions");
        foreach (var qg in sols)
        {
            if (!PoseOk(qg, open, _gL, _hold, _lOn, _rOn))
            { Diag.Add($"  grasp pose collides: {BambooIcdCell.Describe(ContactsNow(qg, open, _gL, _hold, _lOn, _rOn))} q=[{string.Join(",", qg.Select(x => x.ToString("F0")))}]"); continue; }
            var sign = FreeAxisSign(target, qg);
            List<double[]> approach;
            if (Ptp) approach = new List<double[]> { qg }; // PTP straight to the grasp configuration
            else
            {
                var outward = new List<(K.GripTarget, double)>();
                for (var d = CartStepM; d <= ApproachM + 1e-9; d += CartStepM) outward.Add((target, d));
                var back = Cartesian(qg, outward, sign);
                if (back is null) { Diag.Add("  approach unreachable"); continue; }
                approach = Enumerable.Reverse(back).Append(qg).ToList(); // pre … grasp
            }
            var ok = true;
            var bad = -1;
            for (var i = 0; i + 1 < approach.Count && ok; i++)
            {
                ok = SegOk(approach[i], approach[i + 1], open, open, _gL, _gL, _hold, _hold, _lOn, _rOn, VerifyStepDeg);
                if (!ok) bad = i;
            }
            if (!ok) { Diag.Add($"  approach collides near step {bad}: {BambooIcdCell.Describe(ContactsNow(approach[bad], open, _gL, _hold, _lOn, _rOn))} / {BambooIcdCell.Describe(ContactsNow(approach[bad + 1], open, _gL, _hold, _lOn, _rOn))}"); continue; }
            if (!PoseOk(approach[0], open, _gL, _hold, _lOn, _rOn)) continue;
            // Close: jaws clamp the object (plates may touch it from here on). Checked before anything is emitted so
            // a colliding close just moves on to the next IK solution. The held offset is captured first, so the
            // closed pose is checked with the cane in hand.
            var newROn = onName;
            var holdAfter = pickActive ? 1 : _hold;
            var rFrame = K.Free(_base, _baseWorld, qg);
            var heldBefore = _held;
            if (pickActive)
                _held = Transforms.ToFrame(Transforms.Multiply(Transforms.Inverse(ToMotus(rFrame)), _activeRest!));
            if (!SegOk(qg, qg, open, BambooTrussLayout.JawClosedMm, _gL, _gL, _hold, holdAfter, _lOn, newROn, VerifyStepDeg))
            {
                Diag.Add($"  close on {onName} collides: {BambooIcdCell.Describe(ContactsNow(qg, BambooTrussLayout.JawClosedMm, _gL, holdAfter, _lOn, newROn))}");
                _held = heldBefore;
                continue;
            }
            var heldAfter = _held;
            _held = heldBefore;
            // Open the jaws where we are (if closed) then transit to the pre-grasp.
            var k0 = Keys.Count; var q0 = _q; var gR0 = _gR;
            if (Math.Abs(_gR - open) > 1e-9)
            {
                if (!SegOk(_q, _q, _gR, open, _gL, _gL, _hold, _hold, _lOn, _rOn, VerifyStepDeg)) continue;
                Emit("Open", _q, gR: open);
            }
            var path = Ptp ? PtpPath(qg, target, sign, ApproachM, ApproachM / 2) : TransitPath(_q, approach[0]);
            if (path is null)
            {
                Diag.Add("  no transit to pre-grasp");
                Keys.RemoveRange(k0, Keys.Count - k0); _q = q0; _gR = gR0;
                continue;
            }
            if (Ptp)
            {
                foreach (var v in path) Emit("PTP via", v);
                Emit("PTP Grasp", qg);
            }
            else
            {
                foreach (var v in path) Emit("Transit", v);
                // A key labels the motion that ends at it: reaching the pre-grasp is still the transit.
                Emit("Transit", approach[0]);
                for (var i = 1; i + 1 < approach.Count; i++) Emit("Approach", approach[i]);
                Emit("Grasp", qg);
            }
            _held = heldAfter;
            RecordGrip("Close", "R", target, rFrame);
            _rOn = newROn;
            Emit("Close", qg, gR: BambooTrussLayout.JawClosedMm, hold: holdAfter);
            return true;
        }
        return Fail($"{_cycle}: no Motus-clear R grasp of {onName} at ({target.X:F3},{target.Y:F3},{target.Z:F3}) from {_baseOn}");
    }

    /// <summary>R-grip backs straight off the object it just released (along its own +Y), jaws open.</summary>
    bool RRetreat(K.GripTarget from)
    {
        if (Ptp) { PtpDepart(from, "PTP Retreat", _lOn, _rOn); return true; }
        var sign = FreeAxisSign(from, _q);
        var outward = new List<(K.GripTarget, double)>();
        for (var d = CartStepM; d <= ApproachM + 1e-9; d += CartStepM) outward.Add((from, d));
        var back = Cartesian(_q, outward, sign);
        if (back is null) return Fail($"{_cycle}: retreat from {from.On} unreachable");
        var prev = _q;
        foreach (var q in back)
        {
            if (!SegOk(prev, q, _gR, _gR, _gL, _gL, _hold, _hold, _lOn, _rOn, VerifyStepDeg))
                return Fail($"{_cycle}: retreat from {from.On} collides: {BambooIcdCell.Describe(ContactsNow(q, _gR, _gL, _hold, _lOn, _rOn))}");
            Emit("Retreat", q);
            prev = q;
        }
        return true;
    }

    // ---------------- phases ----------------

    /// <summary>Start: L-grip clamped on stake S1 (exact frame), arm parked at a Motus-clear posture.</summary>
    public bool Start()
    {
        BeginCycle("start");
        RecordGrip("Clamp", "L", BambooTrussLayout.S1.Grip, _baseWorld);
        var parks = new List<double[]>
        {
            new double[] { 0, -30, 60, -30, 0 }, new double[] { 0, 30, -60, 30, 0 }, new double[] { 0, 0, 0, 0, 0 },
            new double[] { 90, -30, 60, -30, 0 }, new double[] { -90, -30, 60, -30, 0 }, new double[] { 0, -60, 120, -60, 0 }
        };
        foreach (var p in parks)
            if (PoseOk(p, _gR, _gL, 0, _lOn, _rOn)) { Emit("Park", p, hold: 0); return true; }
        for (var i = 0; i < 200; i++)
        {
            var p = Perturb(new double[5], 120);
            if (PoseOk(p, _gR, _gL, 0, _lOn, _rOn)) { Emit("Park", p, hold: 0); return true; }
        }
        return Fail("start: no Motus-clear park posture on S1");
    }

    /// <summary>Pick the feed cane and place it exactly on <paramref name="member"/> (L-grip stays clamped on its stake).</summary>
    /// <summary>Grasp IK solutions tried one by one per stock offset (0: the first Motus-clear one only).</summary>
    public static int GraspSolutionsPerOffset { get; set; } = 0;

    public bool BuildMember(BambooTrussLayout.MemberDef member)
    {
        BeginCycle(member.Id);
        var store = member.Store;
        // Re-grasp from a relay: the member's cane is parked in a connector — pick it there, pull it out along its axis.
        var parkedIdx = _placed.FindIndex(p => p.Name.StartsWith("placed-" + member.Id + ">", StringComparison.Ordinal));
        (string Name, double[] World)? parked = parkedIdx >= 0 ? _placed[parkedIdx] : null;
        (double X, double Y, double Z)? pullDir = null;
        if (parked is { } pk)
        {
            var rid = pk.Name["placed-".Length..];
            store = BambooTrussLayout.Def(rid)!.Place;
            pullDir = BambooTrussLayout.RelayDir(rid);
            _placed.RemoveAt(parkedIdx);
            _envKey = int.MinValue;
        }
        var parking = BambooTrussLayout.IsRelay(member.Id);
        _activeId = member.Id;
        _activeRest = parked is { } pw ? pw.World : BambooIcdCell.PlacementWorld(store);
        _activeReleased = null;
        _held = null;
        _hold = 0;
        var design = member.Place;
        var designM = BambooIcdCell.PlacementWorld(design);
        var u = design.Axis;
        // Grasp offsets along the stock cane, clear of its pegs (near either end first: far placements need them).
        var offsets = BambooTrussLayout.StockGraspOffsets;
        var startKeys = Keys.Count;
        var snapshot = Snapshot();
        // Each grasp IK solution holds the cane differently (which side the slot comes from), which decides whether
        // the far placements are reachable at all — so every offset tries the nearest few grasp solutions in turn.
        foreach (var (s, graspIdx) in offsets.SelectMany(o => (GraspSolutionsPerOffset <= 0 ? new[] { -1 } : Enumerable.Range(0, GraspSolutionsPerOffset)).Select(i => (o, i))))
        {
            Restore(snapshot, startKeys);
            var pick = K.OnStrut(store, s, BambooIcdCell.Strut);
            if (!RGrasp(pick, BambooIcdCell.Strut, 0, pickActive: true, solutionIndex: graspIdx)) continue;
            // Held geometry: cane centre / axis in the tool frame → where the tool must be at the design pose.
            var H = Transforms.FromFrame(_held!.Value);
            var ct = (X: H[3], Y: H[7], Z: H[11]);
            var at = (X: H[2], Y: H[6], Z: H[10]);
            var lambda = -(ct.X * at.X + ct.Y * at.Y + ct.Z * at.Z);
            var sigma = at.Z >= 0 ? 1 : -1;
            // Lift straight up (world +Y) with the cane.
            var liftSign = FreeAxisSign(pick, _q);
            // Lift the cane off the rail straight back along the jaw's own +Y (the way it came in; ≈ world up for
            // the near-top grasps the rail allows) — as far as the arm reaches, 2..6 cm.
            List<double[]>? lift = null;
            if (Ptp && pullDir is { } pd)
            {
                // Pull the cane out of the connector socket along its own axis (3 cm, one joint segment).
                var pt = new K.GripTarget(pick.X + 0.03 * pd.X, pick.Y + 0.03 * pd.Y, pick.Z + 0.03 * pd.Z, pick.Ax, pick.Ay, pick.Az, pick.On);
                var qo = K.SolveGrip(_base, _baseWorld, pt, new[] { _q }, 8).OrderBy(x => K.JointDist(x, _q)).FirstOrDefault();
                if (qo is not null && K.JointDist(qo, _q) < 30 && SegOk(_q, qo, _gR, _gR, _gL, _gL, 1, 1, _lOn, _rOn, VerifyStepDeg))
                    lift = new List<double[]> { qo };
                if (lift is null) { Error = $"{member.Id}: no Motus-clear pull-out from the connector (s={s})"; Diag.Add(Error); continue; }
            }
            else if (Ptp)
            {
                // PTP lift: one joint segment to the pose 6…2 cm back along the jaw's +Y (or none: the carry leaves).
                for (var n = 6; n >= 2 && lift is null; n--)
                    if (PtpIk(pick, n * CartStepM, liftSign) is { } ql && K.JointDist(ql, _q) < 60 &&
                        SegOk(_q, ql, _gR, _gR, _gL, _gL, 1, 1, _lOn, _rOn, VerifyStepDeg)) lift = new List<double[]> { ql };
                lift ??= new List<double[]>();
            }
            else
                for (var n = 6; n >= 2 && lift is null; n--)
                    lift = Cartesian(_q, Enumerable.Range(1, n).Select(i => (pick, i * CartStepM)), liftSign);
            if (lift is null) { Error = $"{member.Id}: lift unreachable (s={s})"; Diag.Add(Error); continue; }
            var okLift = true; var prev = _q;
            foreach (var q in lift)
            {
                if (!SegOk(prev, q, _gR, _gR, _gL, _gL, 1, 1, _lOn, _rOn, VerifyStepDeg)) { okLift = false; break; }
                prev = q;
            }
            if (!okLift) { Error = $"{member.Id}: lift collides (s={s})"; Diag.Add(Error); continue; }
            foreach (var q in lift) Emit(pullDir is not null ? "PTP pull out" : Ptp ? "PTP Lift" : "Lift", q);
            // Place targets: tool origin on the design axis at λ from the centre, tool Z = σ·axis (or the flipped cane).
            var placed = false;
            foreach (var flip in new[] { 1, -1 })
            {
                var target = new K.GripTarget(design.X + flip * lambda * u.X, design.Y + flip * lambda * u.Y, design.Z + flip * lambda * u.Z,
                    u.X, u.Y, u.Z, "place-" + member.Id);
                var sign = flip * sigma;
                var sols = K.SolveGrip(_base, _baseWorld, target, new[] { _q }, 32, axisSign: sign)
                    .OrderBy(x => K.JointDist(x, _q)).Take(8).ToList();
                Diag.Add($"  place flip={flip}: {sols.Count} IK solutions");
                var placeSnap = Snapshot(); var placeKeys = Keys.Count; var placeROn = _rOn;
                foreach (var qp in sols)
                {
                    if (!PoseOk(qp, _gR, _gL, 1, _lOn, _rOn)) { Diag.Add($"    place pose collides: {BambooIcdCell.Describe(ContactsNow(qp, _gR, _gL, 1, _lOn, _rOn))}"); continue; }
                    // The cane lands where jaw FK ∘ held offset puts it — grasp and place residuals add up, so the
                    // released pose itself must be within the grip tolerance of its design pose.
                    var (offM, offDeg) = OffDesign(Transforms.Multiply(ToMotus(K.Free(_base, _baseWorld, qp)), H), designM);
                    if (offM > K.GripPosTolM || offDeg > K.GripAxisTolDeg) { Diag.Add($"    released cane {offM * 1000:F3} mm / {offDeg:F3}° off design"); continue; }
                    // Bring the cane in along the jaw's own −Y onto its design line (pre-place backed off 4 cm).
                    List<double[]> down;
                    if (Ptp) down = new List<double[]> { qp }; // PTP carry straight to the place configuration
                    else
                    {
                        var above = Cartesian(qp, Enumerable.Range(1, 4).Select(i => (target, i * CartStepM)), sign);
                        if (above is null) { Diag.Add("    pre-place unreachable"); continue; }
                        down = Enumerable.Reverse(above).Append(qp).ToList();
                    }
                    var ok = PoseOk(down[0], _gR, _gL, 1, _lOn, _rOn);
                    for (var i = 0; i + 1 < down.Count && ok; i++)
                        ok = SegOk(down[i], down[i + 1], _gR, _gR, _gL, _gL, 1, 1, _lOn, _rOn, VerifyStepDeg);
                    if (!ok) { Diag.Add("    lower collides"); continue; }
                    double[]? qIns = null;
                    if (Ptp && parking)
                    {
                        // Socket: the cane end slides into the connector along the cane axis — pre-insert 3 cm out.
                        var od = BambooTrussLayout.RelayDir(member.Id);
                        var ti = new K.GripTarget(target.X + 0.03 * od.X, target.Y + 0.03 * od.Y, target.Z + 0.03 * od.Z, target.Ax, target.Ay, target.Az, target.On);
                        qIns = K.SolveGrip(_base, _baseWorld, ti, new[] { qp }, 8, axisSign: sign).OrderBy(x => K.JointDist(x, qp)).FirstOrDefault();
                        if (qIns is null || K.JointDist(qIns, qp) > 30 || !PoseOk(qIns, _gR, _gL, 1, _lOn, _rOn) ||
                            !SegOk(qIns, qp, _gR, _gR, _gL, _gL, 1, 1, _lOn, _rOn, VerifyStepDeg)) { Diag.Add("    no axial insert"); continue; }
                    }
                    var path = Ptp ? (qIns is not null ? PtpPath(qIns, target, sign, 4 * CartStepM, 2 * CartStepM) : PtpPath(qp, target, sign, 4 * CartStepM, 2 * CartStepM)) : TransitPath(_q, down[0]);
                    if (path is null) { Diag.Add("    no carry path"); continue; }
                    if (Ptp)
                    {
                        foreach (var v in path) Emit("PTP carry via", v);
                        if (qIns is not null) { Emit("PTP pre-insert", qIns); Emit("PTP insert", qp); }
                        else Emit("PTP Place", qp);
                    }
                    else
                    {
                        foreach (var v in path) Emit("Carry", v);
                        // The carry ends aligned above the design line (pre-place).
                        Emit("Carry", down[0]);
                        for (var i = 1; i + 1 < down.Count; i++) Emit("Lower", down[i]);
                        Emit("Lower", qp);
                    }
                    // Release: jaws open, cane stays exactly where the jaw put it.
                    var rFrame = K.Free(_base, _baseWorld, qp);
                    _activeReleased = Transforms.Multiply(ToMotus(rFrame), H);
                    var openROn = $"{BambooIcdCell.Strut}|placed-{member.Id}";
                    if (!SegOk(qp, qp, _gR, BambooTrussLayout.JawOpenMm, _gL, _gL, 1, 2, _lOn, openROn, VerifyStepDeg))
                    { _activeReleased = null; Restore(placeSnap, placeKeys); _rOn = placeROn; continue; }
                    RecordGrip("Lower", "R", target, rFrame);
                    RecordPlacement(member, designM);
                    _rOn = openROn;
                    Emit("Open", qp, gR: BambooTrussLayout.JawOpenMm, hold: 2);
                    _rOn = null;
                    if (!RRetreat(target)) return false;
                    // PTP: the placement must leave R a way out — it parks (Motus-clear transit) before the member
                    // counts as placed; a pose that wedges R between the canes moves on to the next solution.
                    if (Constraints.ParkAfterPlace && !RPark())
                    {
                        Diag.Add("    R wedged after the place (no park)");
                        Restore(placeSnap, placeKeys); _rOn = placeROn;
                        continue;
                    }
                    placed = true;
                    break;
                }
                if (placed) break;
            }
            if (!placed) { Error = $"{member.Id}: no Motus-clear carry/place (s={s})"; Diag.Add(Error); continue; }
            _placed.Add(("placed-" + member.Id, _activeReleased!));
            _activeId = null; _activeRest = null; _activeReleased = null; _held = null; _hold = 0;
            _envKey = int.MinValue;
            Notes.Add($"{member.Id}: store grip s={s:+0.00;-0.00} m");
            return true;
        }
        Restore(snapshot, startKeys);
        if (parked is { } back) { _placed.Insert(parkedIdx, back); _envKey = int.MinValue; }
        return Fail(string.IsNullOrEmpty(Error) ? $"{member.Id}: unreachable" : Error);
    }

    void RecordPlacement(BambooTrussLayout.MemberDef member, double[] designM)
    {
        var (m, deg) = OffDesign(_activeReleased!, designM);
        Grips.Add(new GripRecord($"{_cycle}:Placed", "cane", member.Id, m, deg));
    }

    /// <summary>Position (m) and cane-axis (°) offset of a released cane pose from its design pose (4×4 row-major).</summary>
    static (double M, double Deg) OffDesign(double[] r, double[] designM)
    {
        var dx = r[3] - designM[3]; var dy = r[7] - designM[7]; var dz = r[11] - designM[11];
        var dot = Math.Abs(r[2] * designM[2] + r[6] * designM[6] + r[10] * designM[10]);
        return (Math.Sqrt(dx * dx + dy * dy + dz * dz), Math.Acos(Math.Min(1, dot)) * 180 / Math.PI);
    }

    /// <summary>
    /// Walk with base swaps to another outboard stake. R clamps <paramref name="anchor"/> — a placed cane fixed at both
    /// ends (brachiation onto the built structure) or a stake — the base swaps to R, L slides up off its stake's free
    /// top, transits over <paramref name="target"/> and slides down onto it, L clamps, the base swaps back to L, and R
    /// opens, backs off and parks.
    /// </summary>
    public bool Walk(string anchor, BambooTrussLayout.Stake target)
    {
        BeginCycle(_cycle == "walk" ? "walk2" : "walk");
        var startKeys = Keys.Count;
        var snap = Snapshot();
        var from = BambooTrussLayout.StakeNamed(_lStake);
        var grips = new List<K.GripTarget>();
        if (anchor.StartsWith("placed-", StringComparison.Ordinal))
        {
            var id = anchor["placed-".Length..];
            if (!_placed.Any(p => p.Name == anchor)) return Fail($"walk: {anchor} is not placed");
            if (!Clampable(_placed.Select(p => p.Name["placed-".Length..])).Contains(id))
                return Fail($"walk: {anchor} is not fixed at both ends — the robot may not clamp it");
            var m = BambooTrussLayout.Members.First(x => x.Id == id);
            var half = BambooTrussLayout.StrutLength / 2 - 0.04;
            foreach (var f in new[] { 0.0, -0.5, 0.5, -1.0, 1.0, -0.25, 0.25, -0.75, 0.75 })
                grips.Add(K.OnStrut(PlacedPlacement(m), f * half, anchor));
        }
        else if (anchor.Split('@') is [var stakeName, var yText])
        {
            // "stake-X@y": R clamps stake X at height y (may be the stake L stands on, below the L clamp) — on the
            // stake itself: below its free top and above its ground pad.
            var st = BambooTrussLayout.StakeNamed(stakeName);
            var y = double.Parse(yText, System.Globalization.CultureInfo.InvariantCulture);
            if (y > st.Top - 0.015 || y < 0.03) return Fail($"walk: {anchor} is not on the stake (top {st.Top:F3} m)");
            grips.Add(st.At(y));
        }
        else grips.Add(BambooTrussLayout.StakeNamed(anchor).Grip);
        foreach (var g in grips)
            for (var idx = 0; idx < 4; idx++)
            {
                Restore(snap, startKeys);
                if (!RGrasp(g, g.On, 0, pickActive: false, solutionIndex: idx)) continue;
                if (!SwapTo(K.Gripper.R, g.On)) continue;
                if (!LSlide(from, up: true)) continue;
                if (!LOnto(target)) continue;
                if (!SwapTo(K.Gripper.L, target.Name)) continue;
                if (!RRetreat(g)) continue;
                if (!RPark()) continue;
                _lStake = target.Name;
                Notes.Add($"{_cycle}: R clamps {g.On} at ({g.X:F3},{g.Y:F3},{g.Z:F3}); L {from.Name} → {target.Name}; R parks");
                return true;
            }
        Restore(snap, startKeys);
        return Fail(string.IsNullOrEmpty(Error) ? $"walk: no Motus-clear {from.Name}→{target.Name} swap via {anchor}" : "walk: " + Error);
    }

    /// <summary>Placed member as a StrutPlacement-equivalent grip line (uses the actual released pose).</summary>
    BambooIcdCell.StrutPlacement PlacedPlacement(BambooTrussLayout.MemberDef m)
    {
        var w = _placed.First(p => p.Name == "placed-" + m.Id).World;
        // Motus → viewer: (x, y, z)_m → viewer (x, z, −y); axis likewise.
        var v = K.Mul(Transforms.FromAxisAngle(1, 0, 0, -Math.PI / 2), w);
        var ax = (v[2], v[6], v[10]);
        var pitch = Math.Asin(Math.Clamp(-ax.Item2, -1, 1)) * 180 / Math.PI;
        var yaw = Math.Atan2(ax.Item1, ax.Item3) * 180 / Math.PI;
        return new BambooIcdCell.StrutPlacement(v[3], v[7], v[11], pitch, yaw);
    }

    /// <summary>Members already placed (ids).</summary>
    public IReadOnlyList<string> PlacedIds => _placed.Select(p => p.Name["placed-".Length..]).ToList();
    /// <summary>Stake the L clamp is on (when L is the base).</summary>
    public string LStake => _lStake;

    // ---------------- verification hooks (planner ⇔ scan agreement) ----------------

    /// <summary>Current joints (degrees), jaw openings (mm), base gripper and what each gripper is on.</summary>
    public double[] JointsNow => _q.ToArray();
    public double GRNow => _gR;
    public double GLNow => _gL;
    public string BaseGripper => BaseName;
    public string? LOn => _lOn;
    public string? ROn => _rOn;

    /// <summary>
    /// Accept-and-emit one arbitrary PTP segment to (<paramref name="q"/>, jaws) exactly as the planner accepts its own
    /// segments (coarse pass, then the verify pass incl. the canonical scan points); emits a "probe" key if accepted.
    /// For the agreement property: whatever this accepts, <see cref="BambooTrussScan"/> must accept.
    /// </summary>
    public bool ProbeMove(double[] q, double gR, double gL)
    {
        if (!SegOk(_q, q, _gR, gR, _gL, gL, _hold, _hold, _lOn, _rOn, SearchStepDeg)) return false;
        if (!SegOk(_q, q, _gR, gR, _gL, gL, _hold, _hold, _lOn, _rOn, VerifyStepDeg)) return false;
        Emit("probe", q, gR, gL);
        return true;
    }

    /// <summary>
    /// Resume planning after key <paramref name="k"/> of an existing plan: keys and cycles up to k are kept, the robot
    /// state (base, joints, jaws, clamps) is key k's, the standing canes are <paramref name="placed"/>; new keys go into
    /// cycle <paramref name="cycle"/>. Only between members (hold 0, no cane in hand or being placed).
    /// </summary>
    public void ResumeAt(IReadOnlyList<BambooIcdMotion.MotionKey> keys, IReadOnlyList<BambooTrussMotion.CycleBound> cycles, int k,
        IEnumerable<(string Name, double[] World)> placed, string cycle)
    {
        var key = keys[k];
        if (key.Hold != 0) throw new InvalidOperationException($"ResumeAt: key {k} ({key.Label}) holds a cane");
        Keys.Clear();
        Keys.AddRange(keys.Take(k + 1).Select(x => new BambooIcdMotion.MotionKey
        {
            T = x.T, Label = x.Label, Q = x.Q.ToArray(), GR = x.GR, GL = x.GL, Hold = x.Hold, Base = x.Base,
            BaseWorld = x.BaseWorld?.ToArray(), BaseOn = x.BaseOn, LOn = x.LOn, ROn = x.ROn
        }));
        CycleKeys.Clear();
        foreach (var c in cycles.Where(c => c.FirstKey <= k)) CycleKeys.Add((c.MemberId, c.FirstKey, Math.Min(c.LastKey, k)));
        _base = key.Base == "R" ? K.Gripper.R : K.Gripper.L;
        _baseWorld = key.BaseWorld!.ToArray();
        _baseOn = key.BaseOn ?? _baseOn;
        _q = key.Q.ToArray(); _gR = key.GR; _gL = key.GL; _hold = 0;
        _lOn = key.LOn; _rOn = key.ROn;
        if (key.LOn is { } lo && (BambooTrussLayout.IsSupport(lo) || lo.StartsWith("placed-", StringComparison.Ordinal))) _lStake = lo;
        _held = null; _activeId = null; _activeRest = null; _activeReleased = null;
        _placed.Clear(); _placed.AddRange(placed.Select(p => (p.Name, p.World.ToArray())));
        _envKey = int.MinValue;
        Error = "";
        BeginCycle(cycle);
    }

    /// <summary>R-grip (open, L-base) transits to a Motus-clear park posture well away from where it is.</summary>
    bool RPark()
    {
        var parks = new List<double[]>
        {
            new double[] { 0, -30, 60, -30, 0 }, new double[] { 0, 30, -60, 30, 0 }, new double[] { 0, 0, 0, 0, 0 },
            new double[] { 90, -30, 60, -30, 0 }, new double[] { -90, -30, 60, -30, 0 }, new double[] { 0, -60, 120, -60, 0 }
        };
        for (var i = 0; i < 40; i++) parks.Add(Perturb(new double[5], 120));
        foreach (var p in parks.Where(p => K.JointDist(p, _q) > 15).OrderBy(p => K.JointDist(p, _q)))
        {
            if (!PoseOk(p, _gR, _gL, _hold, _lOn, _rOn)) continue;
            var path = TransitPath(_q, p);
            if (path is null) continue;
            foreach (var v in path) Emit("R:Park", v);
            Emit("R:Park", p);
            return true;
        }
        return Fail("walk: no Motus-clear R park after the swap");
    }

    /// <summary>
    /// Base swap with no motion: the other gripper is already clamped. The new base's world frame is its exact FK
    /// frame at the current joints, then the old base's jaws open.
    /// </summary>
    bool SwapTo(K.Gripper newBase, string newBaseOn)
    {
        var q = _q;
        var (l, r) = K.Grippers(_base, _baseWorld, q);
        _base = newBase;
        _baseWorld = newBase == K.Gripper.L ? l : r;
        _baseOn = newBaseOn;
        if (newBase == K.Gripper.R)
        {
            // L opens on its stake: plates may touch the stake while opening.
            var lOpen = BambooTrussLayout.LJawOpenMm;
            if (!SegOk(q, q, _gR, _gR, _gL, lOpen, _hold, _hold, _lOn, _rOn, VerifyStepDeg))
                return Fail($"walk: L open collides: {BambooIcdCell.Describe(ContactsNow(q, _gR, lOpen, _hold, _lOn, _rOn))}");
            Emit("swap base L→R", q);
            Emit("L:Open", q, gL: lOpen);
            _lOn = null;
        }
        else
        {
            var rOpen = BambooTrussLayout.JawOpenMm;
            if (!SegOk(q, q, _gR, rOpen, _gL, _gL, _hold, _hold, _lOn, _rOn, VerifyStepDeg))
                return Fail($"walk: R open collides: {BambooIcdCell.Describe(ContactsNow(q, rOpen, _gL, _hold, _lOn, _rOn))}");
            Emit("swap base R→L", q);
            Emit("R:Open", q, gR: rOpen);
            _rOn = null;
        }
        return true;
    }

    /// <summary>
    /// Brachiate onto the built structure: R clamps <paramref name="anchor"/> (a stake "stake-X@y" or a placed cane
    /// fixed at both ends), the base swaps to R, L leaves its stake (slides up off the free top), then — the L-grip is
    /// opened on one side (+y) to allow side clamping (differs from the ICD URDF, see gen_bamboo_urdf.py) — approaches
    /// placed cane <paramref name="memberId"/> from the side, clamps it, the base swaps back to L and R parks. Only
    /// canes fixed at both ends may be clamped. Every step is Motus-checked with the full L-grip / mount geometry.
    /// <paramref name="offsetFraction"/> / <paramref name="lSolution"/> pin the clamp point and L IK solution (roll
    /// about the cane) so callers can search them against what R must reach next.
    /// </summary>
    public bool Brachiate(string anchor, string memberId, double? offsetFraction = null, int lSolution = -1, string cycle = "brachiate")
    {
        BeginCycle(cycle);
        var startKeys = Keys.Count;
        var snap = Snapshot();
        var placedIds = _placed.Select(p => p.Name["placed-".Length..]).ToList();
        if (!Clampable(placedIds).Contains(memberId))
            return Fail($"brachiate: {memberId} is not fixed at both ends — the robot may not clamp it");
        if (_lStake.StartsWith("placed-", StringComparison.Ordinal))
            return Fail("brachiate: L is already on the structure");
        var from = BambooTrussLayout.StakeNamed(_lStake);
        var m = BambooTrussLayout.Members.First(x => x.Id == memberId);
        var line = PlacedPlacement(m);
        var name = "placed-" + memberId;
        var half = BambooTrussLayout.StrutLength / 2;
        // Clamp points along the cane: near either end first (where the stock and the remaining members are closest).
        var offsets = (offsetFraction is { } only ? new[] { only } : BrachiateOffsets).Select(f => f * (half - 0.03)).ToList();
        foreach (var anchorGrip in AnchorGrips(anchor))
            for (var idx = 0; idx < 3; idx++)
            {
                Restore(snap, startKeys);
                if (!RGrasp(anchorGrip, anchorGrip.On, 0, pickActive: false, solutionIndex: idx)) continue;
                if (!SwapTo(K.Gripper.R, anchorGrip.On)) continue;
                // A stake with a free top: slide up off it. An anchor column (connector above): back off sideways
                // out of the L-grip's open side.
                if (from.Name.StartsWith("anchor-", StringComparison.Ordinal))
                {
                    var lf0 = K.Free(_base, _baseWorld, _q);
                    if (!LBackOff(new K.GripTarget(lf0[3], lf0[7], lf0[11], 0, 1, 0, from.Name))) continue;
                }
                else if (!LSlide(from, up: true)) continue;
                var k1 = Keys.Count; var s1 = Snapshot();
                foreach (var off in offsets)
                {
                    Restore(s1, k1);
                    var target = K.OnStrut(line, off, name);
                    if (!LSideGrasp(target, name, lSolution)) continue;
                    if (!SwapTo(K.Gripper.L, name)) continue;
                    if (!RRetreat(anchorGrip)) continue;
                    if (!RPark()) continue;
                    _lStake = name;
                    Notes.Add($"brachiate: R clamps {anchorGrip.On} at ({anchorGrip.X:F3},{anchorGrip.Y:F3},{anchorGrip.Z:F3}); L {from.Name} → {name} at {off:+0.000;-0.000} m from its centre (side approach); R parks");
                    return true;
                }
            }
        Restore(snap, startKeys);
        return Fail(string.IsNullOrEmpty(Error) ? $"brachiate: no Motus-clear move onto {name} via {anchor}" : "brachiate: " + Error);
    }

    /// <summary>
    /// Hand-over-hand along the chord: with L clamped on a placed cane (fixed at both ends), R side-clamps a double-fixed
    /// cane <paramref name="rCane"/> ("placed-X"), the base swaps to R at rest, L opens and backs off the cane sideways
    /// (out of its open side), side-clamps <paramref name="lCane"/> at <paramref name="lOffsetFraction"/>·(half − 3 cm)
    /// from its centre (IK solution <paramref name="lSolution"/>), the base swaps back to L and R parks.
    /// </summary>
    public bool HandOverHand(string rCane, string lCane, double lOffsetFraction, int lSolution = -1, string cycle = "hand-over-hand",
        double? rOffsetM = null)
    {
        BeginCycle(cycle);
        var startKeys = Keys.Count;
        var snap = Snapshot();
        var placedIds = _placed.Select(p => p.Name["placed-".Length..]).ToList();
        var fixedBoth = Clampable(placedIds);
        if (!_lStake.StartsWith("placed-", StringComparison.Ordinal) || _base != K.Gripper.L)
            return Fail("hand-over-hand: L must be the base, clamped on a placed cane");
        if (!fixedBoth.Contains(lCane) || !fixedBoth.Contains(rCane["placed-".Length..]))
            return Fail("hand-over-hand: both canes must be fixed at both ends");
        var oldL = _lStake;
        var oldLine = PlacedPlacement(BambooTrussLayout.Members.First(x => x.Id == oldL["placed-".Length..]));
        var m = BambooTrussLayout.Members.First(x => x.Id == lCane);
        var name = "placed-" + lCane;
        var target = K.OnStrut(PlacedPlacement(m), lOffsetFraction * (BambooTrussLayout.StrutLength / 2 - 0.03), name);
        foreach (var anchorGrip in AnchorGrips(rCane, rOffsetM))
            for (var idx = 0; idx < 3; idx++)
            {
                Restore(snap, startKeys);
                if (!RGrasp(anchorGrip, anchorGrip.On, 0, pickActive: false, solutionIndex: idx)) continue;
                if (!SwapTo(K.Gripper.R, anchorGrip.On)) continue;
                // L (now free, jaws open) backs off its cane out of the open side.
                var lf = K.Free(_base, _baseWorld, _q);
                var a = oldLine.Axis;
                var from = new K.GripTarget(lf[3], lf[7], lf[11], a.X, a.Y, a.Z, oldL);
                if (!LBackOff(from)) continue;
                if (!LSideGrasp(target, name, lSolution)) continue;
                if (!SwapTo(K.Gripper.L, name)) continue;
                if (!RRetreat(anchorGrip)) continue;
                if (!RPark()) continue;
                _lStake = name;
                Notes.Add($"{_cycle}: R clamps {anchorGrip.On} at ({anchorGrip.X:F3},{anchorGrip.Y:F3},{anchorGrip.Z:F3}); L {oldL} → {name} at {lOffsetFraction * (BambooTrussLayout.StrutLength / 2 - 0.03):+0.000;-0.000} m from its centre; R parks");
                return true;
            }
        Restore(snap, startKeys);
        return Fail(string.IsNullOrEmpty(Error) ? $"hand-over-hand: no Motus-clear step onto {name}" : "hand-over-hand: " + Error);
    }

    /// <summary>
    /// Step off the structure back onto a stake: with L clamped on a placed cane, R clamps <paramref name="rCane"/>
    /// ("placed-X", fixed at both ends) from the side, the base swaps to R at rest, L opens and backs off its cane out of
    /// its open side, transits above <paramref name="stakeName"/>, slides down onto it and closes; the base swaps back to
    /// L and R parks.
    /// </summary>
    public bool StepToStake(string rCane, string stakeName, string cycle = "walk-step", double? rOffsetM = null)
    {
        BeginCycle(cycle);
        var startKeys = Keys.Count;
        var snap = Snapshot();
        var fixedBoth = Clampable(_placed.Select(p => p.Name["placed-".Length..]));
        if (!_lStake.StartsWith("placed-", StringComparison.Ordinal) || _base != K.Gripper.L)
            return Fail("step to stake: L must be the base, clamped on a placed cane");
        if (!fixedBoth.Contains(rCane["placed-".Length..]))
            return Fail("step to stake: R may only clamp a cane fixed at both ends");
        var oldL = _lStake;
        var oldLine = PlacedPlacement(BambooTrussLayout.Members.First(x => x.Id == oldL["placed-".Length..]));
        var stake = BambooTrussLayout.StakeNamed(stakeName);
        foreach (var anchorGrip in AnchorGrips(rCane, rOffsetM))
            for (var idx = 0; idx < 2; idx++)
            {
                Restore(snap, startKeys);
                if (!RGrasp(anchorGrip, anchorGrip.On, 0, pickActive: false, solutionIndex: idx)) continue;
                if (!SwapTo(K.Gripper.R, anchorGrip.On)) continue;
                var lf = K.Free(_base, _baseWorld, _q);
                var a = oldLine.Axis;
                if (!LBackOff(new K.GripTarget(lf[3], lf[7], lf[11], a.X, a.Y, a.Z, oldL))) continue;
                if (!LOnto(stake)) continue;
                if (!SwapTo(K.Gripper.L, stake.Name)) continue;
                if (!RRetreat(anchorGrip)) continue;
                if (!RPark()) continue;
                _lStake = stake.Name;
                Notes.Add($"{_cycle}: R clamps {anchorGrip.On} at ({anchorGrip.X:F3},{anchorGrip.Y:F3},{anchorGrip.Z:F3}); L {oldL} → {stake.Name}; R parks");
                return true;
            }
        Restore(snap, startKeys);
        return Fail(string.IsNullOrEmpty(Error) ? $"step to stake: no Motus-clear step onto {stakeName}" : "step to stake: " + Error);
    }

    /// <summary>World frame of the current base gripper (row-major 4×4, viewer frame).</summary>
    public double[] BaseWorldNow => _baseWorld.ToArray();
    /// <summary>Current base gripper ("L" / "R").</summary>
    public string BaseNow => BaseName;
    /// <summary>Current joint vector (deg).</summary>
    public double[] QNow => _q.ToArray();

    // ---------------- targeted walk landing ----------------

    /// <summary>When set, grasp IK solutions are tried nearest to this free-gripper frame first (targeted landing).</summary>
    double[]? _preferFree;

    double PreferKey(double[] q)
    {
        if (_preferFree is null) return K.JointDist(q, _q);
        var f = K.Free(_base, _baseWorld, q);
        var d = 0.0;
        for (var i = 0; i < 12; i++) d += Math.Abs(f[i] - _preferFree[i]);
        return d;
    }

    /// <summary>A landing candidate: R clamps <c>RCane</c> at <c>ROff</c> (frame <c>RFrame</c>), L lands on
    /// <c>LCane</c> at <c>LOff</c> with frame <c>LFrame</c>; <c>Score</c> = stock + place IK offsets reachable.</summary>
    public sealed record Landing(string RCane, double ROff, double[] RFrame, string LCane, double LOff, double[] LFrame, int Score);

    /// <summary>Clamp points (m from the centre) tried for targeted landings.</summary>
    public static double[] LandOffsets { get; set; } = { -1.0, -0.66, -0.33, 0.0, 0.33, 0.66, 1.0 };
    /// <summary>Real-planner attempts per targeted landing search.</summary>
    public static int LandTries { get; set; } = 14;

    /// <summary>
    /// Targeted landing candidates for building <paramref name="member"/>: one node-transfer step from the current L
    /// base (R clamps a fixed member, L lands on a fixed member). Pure IK first (true FK of both grippers): every R
    /// grasp solution fixes the R frame, every L solution from there fixes the L landing frame (point + roll); a
    /// candidate is kept when that L frame has IK both to the stock cane and to the member's place line.
    /// </summary>
    public List<Landing> LandingCandidates(BambooTrussLayout.MemberDef member)
    {
        var res = new List<Landing>();
        if (_base != K.Gripper.L) return res;
        var placedIds = _placed.Select(p => p.Name["placed-".Length..]).ToList();
        var fixedIds = Clampable(placedIds);
        var half = BambooTrussLayout.StrutLength / 2;
        var store = member.Store; var d = member.Place; var u = d.Axis;
        var pk = _placed.FirstOrDefault(p => p.Name.StartsWith("placed-" + member.Id + ">", StringComparison.Ordinal));
        if (pk.Name is not null) store = BambooTrussLayout.Def(pk.Name["placed-".Length..])!.Place;
        var stockT = BambooTrussLayout.StockGraspOffsets.Select(o => K.OnStrut(store, o, "s")).ToList();
        var placeT = BambooTrussLayout.StockGraspOffsets.Select(o => new K.GripTarget(d.X + o * u.X, d.Y + o * u.Y, d.Z + o * u.Z, u.X, u.Y, u.Z, "p")).ToList();
        int nR = 0, nL = 0, nS = 0;
        foreach (var rc in fixedIds)
        {
            var rl = PlacedPlacement(BambooTrussLayout.Members.First(x => x.Id == rc));
            foreach (var fr in new[] { -1.0, -0.5, 0.0, 0.5, 1.0 })
            {
                var g = K.OnStrut(rl, fr * (half - 0.04), "placed-" + rc);
                foreach (var q1 in Distinct(K.SolveGrip(K.Gripper.L, _baseWorld, g, new[] { _q }, 16)))
                {
                    var G = K.Free(K.Gripper.L, _baseWorld, q1); nR++;
                    foreach (var lc in fixedIds)
                    {
                        var ll = PlacedPlacement(BambooTrussLayout.Members.First(x => x.Id == lc));
                        foreach (var fl in LandOffsets)
                        {
                            var t = K.OnStrut(ll, fl * (half - 0.03), "placed-" + lc);
                            if ("placed-" + lc == rc) continue;
                            foreach (var q2 in Distinct(K.SolveGrip(K.Gripper.R, G, t, new[] { q1 }, 12)))
                            {
                                var W = K.Free(K.Gripper.R, G, q2); nL++;
                                var ns = stockT.Count(x => K.SolveGrip(K.Gripper.L, W, x, null, 6).Count > 0);
                                if (ns == 0) continue;
                                nS++;
                                var np = placeT.Count(x => K.SolveGrip(K.Gripper.L, W, x, null, 6).Count > 0);
                                if (np == 0) continue;
                                res.Add(new Landing(rc, fr * (half - 0.04), G, lc, fl * (half - 0.03), W, ns + np));
                            }
                        }
                    }
                }
            }
        }
        Diag.Add($"landing {member.Id}: R grasps {nR}, L landings {nL}, with stock IK {nS}, with stock+place IK {res.Count}");
        return res.OrderByDescending(x => x.Score).ToList();
    }

    static List<double[]> Distinct(List<double[]> sols)
    {
        var outp = new List<double[]>();
        foreach (var q in sols) if (outp.All(o => K.JointDist(o, q) > 2)) outp.Add(q);
        return outp;
    }

    /// <summary>
    /// One node-transfer step with a targeted landing: R clamps fixed member <paramref name="rCane"/> at
    /// <paramref name="rOffM"/> (the IK solution nearest <paramref name="rPref"/>), base swaps to R at rest, L backs off
    /// sideways out of its open side (off an anchor column or a cane), side-clamps fixed member <paramref name="lCane"/>
    /// at <paramref name="lOffM"/> (solution nearest the wanted landing frame <paramref name="lPref"/>: point + roll),
    /// base swaps back to L, R retreats and parks.
    /// </summary>
    public bool WalkStep(string rCane, double rOffM, double[]? rPref, string lCane, double lOffM, double[]? lPref, string cycle)
    {
        BeginCycle(cycle);
        var startKeys = Keys.Count;
        var snap = Snapshot();
        var fixedIds = Clampable(_placed.Select(p => p.Name["placed-".Length..]));
        if (_base != K.Gripper.L) return Fail("walk step: L must be the base");
        if (!fixedIds.Contains(rCane) || !fixedIds.Contains(lCane)) return Fail("walk step: both canes must be fixed into the structure");
        var oldL = _lStake;
        var onAnchor = BambooTrussLayout.IsSupport(oldL);
        var oldAxis = onAnchor ? (X: 0.0, Y: 1.0, Z: 0.0) : PlacedPlacement(BambooTrussLayout.Members.First(x => x.Id == oldL["placed-".Length..])).Axis;
        var name = "placed-" + lCane;
        var target = K.OnStrut(PlacedPlacement(BambooTrussLayout.Members.First(x => x.Id == lCane)), lOffM, name);
        var anchorGrip = K.OnStrut(PlacedPlacement(BambooTrussLayout.Members.First(x => x.Id == rCane)), rOffM, "placed-" + rCane);
        for (var idx = 0; idx < 3; idx++)
        {
            Restore(snap, startKeys);
            _preferFree = rPref;
            var ok = RGrasp(anchorGrip, anchorGrip.On, 0, pickActive: false, solutionIndex: idx);
            _preferFree = null;
            if (!ok) continue;
            if (!SwapTo(K.Gripper.R, anchorGrip.On)) continue;
            var lf = K.Free(_base, _baseWorld, _q);
            if (!LBackOff(new K.GripTarget(lf[3], lf[7], lf[11], oldAxis.X, oldAxis.Y, oldAxis.Z, oldL))) continue;
            _preferFree = lPref;
            ok = LSideGrasp(target, name);
            _preferFree = null;
            if (!ok) continue;
            if (!SwapTo(K.Gripper.L, name)) continue;
            if (!RRetreat(anchorGrip)) continue;
            if (!RPark()) continue;
            _lStake = name;
            Notes.Add($"{_cycle}: R clamps {anchorGrip.On} at {rOffM:+0.000;-0.000} m; L {oldL} → {name} at {lOffM:+0.000;-0.000} m (targeted landing); R parks");
            return true;
        }
        _preferFree = null;
        Restore(snap, startKeys);
        return Fail(string.IsNullOrEmpty(Error) ? $"walk step: no Motus-clear step onto {name}" : "walk step: " + Error);
    }

    /// <summary>Open L-grip (R-base) backs straight off the cane it released, out of its open side (+y), Motus-checked.</summary>
    bool LBackOff(K.GripTarget from)
    {
        if (Ptp) { PtpDepart(from, $"L:PTP off {from.On}", null, _rOn); _lOn = null; return true; }
        var sign = FreeAxisSign(from, _q);
        foreach (var side in new[] { -1.0, 1.0 })
        {
            var outward = new List<(K.GripTarget, double)>();
            for (var d = CartStepM; d <= ApproachM + 1e-9; d += CartStepM) outward.Add((from, side * d));
            var back = Cartesian(_q, outward, sign);
            if (back is null) continue;
            var ok = true; var prev = _q;
            foreach (var q in back)
            {
                if (!SegOk(prev, q, _gR, _gR, _gL, _gL, _hold, _hold, null, _rOn, VerifyStepDeg)) { ok = false; break; }
                prev = q;
            }
            if (!ok) continue;
            _lOn = null;
            foreach (var q in back) Emit($"L:Back off {from.On}", q);
            return true;
        }
        return Fail($"{_cycle}: L cannot back off {from.On} Motus-clear");
    }

    /// <summary>Clamp points along the target cane, as fractions of (half − 3 cm) from its centre.</summary>
    public static double[] BrachiateOffsets { get; set; } = { -1.0, -0.75, -0.5, 1.0, 0.75, 0.5, -0.25, 0.25, 0.0 };

    /// <summary>R clamp candidates on <paramref name="anchor"/>; <paramref name="rOffsetM"/> pins the point on a placed cane
    /// (metres from its centre).</summary>
    IEnumerable<K.GripTarget> AnchorGrips(string anchor, double? rOffsetM = null)
    {
        if (anchor.StartsWith("placed-", StringComparison.Ordinal))
        {
            var id = anchor["placed-".Length..];
            if (!_placed.Any(p => p.Name == anchor)) yield break;
            if (!Clampable(_placed.Select(p => p.Name["placed-".Length..])).Contains(id)) yield break;
            var m = BambooTrussLayout.Members.First(x => x.Id == id);
            var half = BambooTrussLayout.StrutLength / 2 - 0.04;
            if (rOffsetM is { } off) { yield return K.OnStrut(PlacedPlacement(m), Math.Clamp(off, -half, half), anchor); yield break; }
            foreach (var f in new[] { 0.0, -0.5, 0.5, -1.0, 1.0 })
                yield return K.OnStrut(PlacedPlacement(m), f * half, anchor);
        }
        else if (anchor.Split('@') is [var stakeName, var yText])
        {
            var st = BambooTrussLayout.StakeNamed(stakeName);
            var y = double.Parse(yText, System.Globalization.CultureInfo.InvariantCulture);
            if (y <= st.Top - 0.015 && y >= 0.03) yield return st.At(y);
        }
        else yield return BambooTrussLayout.StakeNamed(anchor).Grip;
    }

    /// <summary>
    /// L-grip (free, R-base, jaws open) clamps a cane from the side: transit → pre-grasp backed off along the jaw axis
    /// (either side; the jaw opens in a straight line) → straight approach → close.
    /// </summary>
    bool LSideGrasp(K.GripTarget target, string onName, int solutionIndex = -1)
    {
        if (_base != K.Gripper.R) return Fail("L side grasp needs the R-grip as base");
        var open = BambooTrussLayout.LJawOpenMm;
        var closed = BambooTrussLayout.JawClosedMm;
        var sols = K.SolveGrip(_base, _baseWorld, target, new[] { _q }, 32)
            .OrderBy(x => PreferKey(x)).Take(8).ToList();
        // A given IK solution fixes the L roll about the cane, i.e. what R can reach afterwards: callers may pick one.
        if (solutionIndex >= 0) sols = sols.Skip(solutionIndex).Take(1).ToList();
        Diag.Add($"{_cycle}: L side grasp {onName} ({target.X:F3},{target.Y:F3},{target.Z:F3}): {sols.Count} IK solutions");
        foreach (var qg in sols)
        {
            if (!PoseOk(qg, _gR, open, _hold, _lOn, _rOn))
            { Diag.Add($"  L grasp pose collides: {BambooIcdCell.Describe(ContactsNow(qg, _gR, open, _hold, _lOn, _rOn))}"); continue; }
            var sign = FreeAxisSign(target, qg);
            if (Ptp)
            {
                if (!SegOk(qg, qg, _gR, _gR, open, closed, _hold, _hold, onName, _rOn, VerifyStepDeg))
                { Diag.Add($"  L close on {onName} collides: {BambooIcdCell.Describe(ContactsNow(qg, _gR, closed, _hold, onName, _rOn))}"); continue; }
                var pp = PtpPath(qg, target, sign, ApproachM, -ApproachM, ApproachM / 2, -ApproachM / 2);
                if (pp is null) { Diag.Add("  no L PTP path to the clamp"); continue; }
                foreach (var v in pp) Emit("L:PTP via", v);
                Emit($"L:PTP to {onName}", qg);
                _lOn = onName;
                RecordGrip("L:Close", "L", target, K.Free(_base, _baseWorld, qg));
                Emit($"L:Close {onName}", qg, gL: closed);
                return true;
            }
            foreach (var side in new[] { 1.0, -1.0 })
            {
                var outward = new List<(K.GripTarget, double)>();
                for (var d = CartStepM; d <= ApproachM + 1e-9; d += CartStepM) outward.Add((target, side * d));
                var back = Cartesian(qg, outward, sign);
                if (back is null) { Diag.Add($"  L approach (side {side}) unreachable"); continue; }
                var approach = Enumerable.Reverse(back).Append(qg).ToList();
                var ok = PoseOk(approach[0], _gR, open, _hold, _lOn, _rOn);
                var bad = 0;
                for (var i = 0; i + 1 < approach.Count && ok; i++)
                {
                    ok = SegOk(approach[i], approach[i + 1], _gR, _gR, open, open, _hold, _hold, _lOn, _rOn, VerifyStepDeg);
                    if (!ok) bad = i + 1;
                }
                if (!ok) { Diag.Add($"  L approach (side {side}) collides at step {bad}/{approach.Count - 1}: {BambooIcdCell.Describe(ContactsNow(approach[bad], _gR, open, _hold, _lOn, _rOn))}"); continue; }
                if (!SegOk(qg, qg, _gR, _gR, open, closed, _hold, _hold, onName, _rOn, VerifyStepDeg))
                { Diag.Add($"  L close on {onName} collides: {BambooIcdCell.Describe(ContactsNow(qg, _gR, closed, _hold, onName, _rOn))}"); continue; }
                var path = TransitPath(_q, approach[0]);
                if (path is null) { Diag.Add("  no L transit to pre-grasp"); continue; }
                foreach (var v in path) Emit("L:Transit", v);
                Emit("L:Transit", approach[0]);
                for (var i = 1; i + 1 < approach.Count; i++) Emit($"L:Approach {onName}", approach[i]);
                Emit($"L:Approach {onName}", qg);
                _lOn = onName;
                RecordGrip("L:Close", "L", target, K.Free(_base, _baseWorld, qg));
                Emit($"L:Close {onName}", qg, gL: closed);
                return true;
            }
        }
        return Fail($"{_cycle}: no Motus-clear L side grasp of {onName} from {_baseOn}");
    }

    /// <summary>L-grip slides along a stake (R-base): up and off its free top, or (up = false) down onto it.</summary>
    bool LSlide(BambooTrussLayout.Stake stake, bool up)
    {
        if (Ptp && up)
        {
            // PTP off the stake: one joint move (via-points / RRT if needed) to above its free top.
            var lf = K.Free(_base, _baseWorld, _q);
            var sg = lf[6] >= 0 ? 1 : -1;
            var qa = K.SolveGrip(_base, _baseWorld, stake.At(stake.Top + BambooTrussLayout.StakeSlideClear), new[] { _q }, 16, axisSign: sg)
                .OrderBy(x => K.JointDist(x, _q)).FirstOrDefault();
            if (qa is null) return Fail($"walk: no IK above {stake.Name}");
            var pp = TransitPath(_q, qa);
            if (pp is null) return Fail($"walk: no Motus-clear PTP off {stake.Name}");
            foreach (var v in pp) Emit("L:PTP via", v);
            Emit($"L:PTP off {stake.Name}", qa);
            return true;
        }
        var y0 = stake.ClampY; var y1 = stake.Top + BambooTrussLayout.StakeSlideClear;
        var lFrame = K.Free(_base, _baseWorld, _q);
        var sign = lFrame[6] >= 0 ? 1 : -1;
        var n = (int)Math.Ceiling((y1 - y0) / CartStepM);
        var targets = Enumerable.Range(1, n).Select(i =>
        {
            var y = up ? y0 + (y1 - y0) * i / n : y1 - (y1 - y0) * i / n;
            return (stake.At(y), 0.0);
        });
        var path = Cartesian(_q, targets, sign);
        if (path is null) return Fail($"walk: L slide {(up ? "up off" : "down onto")} {stake.Name} unreachable");
        var prev = _q;
        foreach (var q in path)
        {
            if (!SegOk(prev, q, _gR, _gR, _gL, _gL, _hold, _hold, _lOn, _rOn, VerifyStepDeg))
                return Fail($"walk: L slide on {stake.Name} collides: {BambooIcdCell.Describe(ContactsNow(q, _gR, _gL, _hold, _lOn, _rOn))}");
            Emit(up ? $"L:Slide up {stake.Name}" : $"L:Slide down {stake.Name}", q);
            prev = q;
        }
        return true;
    }

    /// <summary>L-grip (R-base) transits above a stake's free top, slides down to its clamp height and closes.</summary>
    bool LOnto(BambooTrussLayout.Stake stake)
    {
        if (Ptp)
        {
            var snap0 = Snapshot(); var keys0 = Keys.Count;
            foreach (var qc in K.SolveGrip(_base, _baseWorld, stake.Grip, new[] { _q }, 32).OrderBy(x => K.JointDist(x, _q)).Take(4))
            {
                Restore(snap0, keys0);
                if (!PoseOk(qc, _gR, _gL, _hold, _lOn, _rOn)) continue;
                var closedL = BambooTrussLayout.JawClosedMm;
                if (!SegOk(qc, qc, _gR, _gR, _gL, closedL, _hold, _hold, stake.Name, _rOn, VerifyStepDeg)) continue;
                List<double[]>? pp = null;
                if (MoveOk(_q, qc, SearchStepDeg) && MoveOk(_q, qc, VerifyStepDeg)) pp = new List<double[]>();
                if (pp is null)
                {
                    // PTP waypoint above the stake's free top, then one joint segment down onto the clamp pose.
                    var sgn = K.Free(_base, _baseWorld, qc)[6] >= 0 ? 1 : -1;
                    var qa = K.SolveGrip(_base, _baseWorld, stake.At(stake.Top + BambooTrussLayout.StakeSlideClear), new[] { qc }, 8, axisSign: sgn)
                        .OrderBy(x => K.JointDist(x, qc)).FirstOrDefault();
                    if (qa is not null && K.JointDist(qa, qc) < 45 && PoseOk(qa, _gR, _gL, _hold, _lOn, _rOn) && MoveOk(qa, qc, VerifyStepDeg) &&
                        TransitPath(_q, qa) is { } pa) { pa.Add(qa); pp = pa; }
                }
                pp ??= TransitPath(_q, qc);
                if (pp is null) continue;
                foreach (var v in pp) Emit("L:PTP via", v);
                Emit($"L:PTP to {stake.Name}", qc);
                _lOn = stake.Name;
                RecordGrip("L:Close", "L", stake.Grip, K.Free(_base, _baseWorld, _q));
                Emit($"L:Close {stake.Name}", _q, gL: closedL);
                return true;
            }
            Restore(snap0, keys0);
            return Fail($"walk: L cannot PTP onto {stake.Name} Motus-clear");
        }
        var above = stake.At(stake.Top + BambooTrussLayout.StakeSlideClear);
        var sols = K.SolveGrip(_base, _baseWorld, above, new[] { _q }, 32)
            .OrderBy(x => K.JointDist(x, _q)).Take(8).ToList();
        var snap = Snapshot(); var keys = Keys.Count;
        foreach (var qa in sols)
        {
            Restore(snap, keys);
            if (!PoseOk(qa, _gR, _gL, _hold, _lOn, _rOn)) continue;
            var path = TransitPath(_q, qa);
            if (path is null) continue;
            foreach (var v in path) Emit("L:Transit", v);
            Emit($"L:Above {stake.Name}", qa);
            if (!LSlide(stake, up: false)) continue;
            var closed = BambooTrussLayout.JawClosedMm;
            if (!SegOk(_q, _q, _gR, _gR, _gL, closed, _hold, _hold, stake.Name, _rOn, VerifyStepDeg)) continue;
            _lOn = stake.Name;
            RecordGrip("L:Close", "L", stake.Grip, K.Free(_base, _baseWorld, _q));
            Emit($"L:Close {stake.Name}", _q, gL: closed);
            return true;
        }
        Restore(snap, keys);
        return Fail($"walk: L cannot reach above {stake.Name} Motus-clear");
    }

    // ---------------- snapshot / restore ----------------

    sealed record State(K.Gripper Base, double[] BaseWorld, string BaseOn, double[] Q, double GR, double GL, int Hold,
        string? LOn, string? ROn, Frame? Held, double[]? Rest, double[]? Released, string? ActiveId, int Placed, int Grips, int Notes, string LStake);

    State Snapshot() => new(_base, _baseWorld.ToArray(), _baseOn, _q.ToArray(), _gR, _gL, _hold, _lOn, _rOn, _held,
        _activeRest, _activeReleased, _activeId, _placed.Count, Grips.Count, Notes.Count, _lStake);

    /// <summary>Opaque planner checkpoint (state, keys, cycles) for goal-driven search over alternatives.</summary>
    public sealed class Checkpoint
    {
        internal Checkpoint(object state, int keys, int cycles) { State = state; KeyCount = keys; CycleCount = cycles; }
        internal object State { get; }
        internal int KeyCount { get; }
        internal int CycleCount { get; }
    }

    public Checkpoint Save() => new(Snapshot(), Keys.Count, CycleKeys.Count);

    /// <summary>
    /// Checkpoint that also copies the keys / cycles / grips / notes / placed canes, so a search can branch from it after
    /// other branches have overwritten the shared lists (BFS over walk states). Restore with <see cref="RollbackDeep"/>.
    /// </summary>
    public sealed class DeepCheckpoint
    {
        internal DeepCheckpoint(object state, List<BambooIcdMotion.MotionKey> keys, List<(string, int, int)> cycles,
            List<GripRecord> grips, List<string> notes, List<(string, double[])> placed, string cycle, int rng)
        { State = state; Keys = keys; Cycles = cycles; Grips = grips; Notes = notes; Placed = placed; Cycle = cycle; Rng = rng; }
        internal int Rng { get; }
        internal object State { get; }
        internal List<BambooIcdMotion.MotionKey> Keys { get; }
        internal List<(string, int, int)> Cycles { get; }
        internal List<GripRecord> Grips { get; }
        internal List<string> Notes { get; }
        internal List<(string, double[])> Placed { get; }
        internal string Cycle { get; }
        public int KeyCount => Keys.Count;
    }

    public DeepCheckpoint SaveDeep() => new(Snapshot(), Keys.ToList(), CycleKeys.ToList(), Grips.ToList(), Notes.ToList(),
        _placed.Select(p => (p.Name, p.World)).ToList(), _cycle, _rng.Position);

    public void RollbackDeep(DeepCheckpoint c)
    {
        Keys.Clear(); Keys.AddRange(c.Keys);
        CycleKeys.Clear(); CycleKeys.AddRange(c.Cycles);
        Grips.Clear(); Grips.AddRange(c.Grips);
        Notes.Clear(); Notes.AddRange(c.Notes);
        _placed.Clear(); _placed.AddRange(c.Placed);
        _cycle = c.Cycle;
        _rng.Rewind(c.Rng);
        Restore((State)c.State, c.Keys.Count);
        Error = "";
    }

    public void Rollback(Checkpoint c)
    {
        Restore((State)c.State, c.KeyCount);
        if (CycleKeys.Count > c.CycleCount) CycleKeys.RemoveRange(c.CycleCount, CycleKeys.Count - c.CycleCount);
        Error = "";
    }

    void Restore(State s, int keyCount)
    {
        _base = s.Base; _baseWorld = s.BaseWorld.ToArray(); _baseOn = s.BaseOn; _q = s.Q.ToArray(); _gR = s.GR; _gL = s.GL;
        _hold = s.Hold; _lOn = s.LOn; _rOn = s.ROn; _held = s.Held; _activeRest = s.Rest; _activeReleased = s.Released;
        _activeId = s.ActiveId; _lStake = s.LStake;
        if (_placed.Count > s.Placed) _placed.RemoveRange(s.Placed, _placed.Count - s.Placed);
        if (Grips.Count > s.Grips) Grips.RemoveRange(s.Grips, Grips.Count - s.Grips);
        if (Notes.Count > s.Notes) Notes.RemoveRange(s.Notes, Notes.Count - s.Notes);
        if (Keys.Count > keyCount) Keys.RemoveRange(keyCount, Keys.Count - keyCount);
        _envKey = int.MinValue;
    }

    /// <summary>Stamp key times by joint travel (+ gripper strokes) so playback speed tracks real motion.</summary>
    /// <summary>Timeline share of each base-swap pause (≈ 0.6 s at the default Play speed).</summary>
    public const double SwapPauseShare = 0.012;

    public void StampTimes()
    {
        EndCycles();
        var w = new List<double> { 0 };
        for (var i = 1; i < Keys.Count; i++)
        {
            var a = Keys[i - 1]; var b = Keys[i];
            var s = 0.0;
            for (var k = 0; k < 5; k++) s += Math.Abs(b.Q[k] - a.Q[k]);
            var dg = Math.Abs(b.GR - a.GR) + Math.Abs(b.GL - a.GL);
            if (dg > 1e-9) s += 10 + dg * 0.6;
            w.Add(Math.Max(s, 3));
        }
        // A base swap is a rest pause (nothing moves): give it a visible share of the timeline so its label shows.
        var moving = w.Sum();
        for (var i = 1; i < Keys.Count; i++)
            if (Keys[i].Label.Contains("swap base", StringComparison.Ordinal)) w[i] = Math.Max(w[i], SwapPauseShare * moving);
        var total = w.Sum();
        var acc = 0.0;
        for (var i = 0; i < Keys.Count; i++)
        {
            acc += w[i];
            Keys[i].T = total <= 0 ? 0 : acc / total;
        }
        if (Keys.Count > 0) { Keys[0].T = 0; Keys[^1].T = 1; }
    }
}
