using Motus.Core;
using Motus.Geometry;
using K = Motus.Viewer.Services.BambooGripKinematics;

namespace Motus.Viewer.Services;

/// <summary>
/// Motus collision scan for the grounded /bamboo-truss motion. Independently of the planner it re-derives, from the
/// keys alone, where every body is: the clamped base gripper's fixed world frame, the free gripper and the mount by
/// FK from that base, the active cane (resting on the feed, held at its true jaw offset, released exactly where the
/// jaw left it) and every earlier member at its real released pose. Robot links, both grippers, mount parts and the
/// held cane are checked against ground pads, posts, stakes and canes (and self) — the only allowed touches are the
/// jaw plates on whatever each gripper clamps.
/// </summary>
public sealed class BambooTrussScan
{
    public sealed record Hit(double T, double LastClearT, string Label, double[] QDeg, IReadOnlyList<CollisionContact> Contacts)
    {
        public string Describe() => BambooIcdCell.Describe(Contacts);
    }

    private readonly BambooIcdCell _cell;
    private readonly IReadOnlyList<BambooIcdMotion.MotionKey> _keys;
    private readonly IReadOnlyList<BambooTrussMotion.CycleBound> _cycles;
    private readonly IReadOnlyList<BambooTrussLayout.MemberDef> _members;
    private readonly List<CycleWorld> _world = new();
    private readonly List<double> _ts = new();
    private int _next;

    /// <summary>Per-cycle world: active cane track and the members already standing.</summary>
    public sealed class CycleWorld
    {
        public string? ActiveId { get; init; }
        public double[]? Rest { get; init; }
        public Frame? HeldOffset { get; init; }
        public double[]? Released { get; init; }
        public List<(string Name, double[] World)> Placed { get; init; } = new();
        public string? Miss { get; init; }
    }

    public double LastClearT { get; private set; } = -1;
    public Hit? FirstHit { get; private set; }
    public bool Complete => FirstHit is not null || _next >= _ts.Count;
    public int SampleCount => _ts.Count;
    public IReadOnlyList<double> SampleTimes => _ts;
    public IReadOnlyList<BambooTrussMotion.CycleBound> Cycles => _cycles;
    public IReadOnlyList<CycleWorld> World => _world;

    public BambooTrussScan(
        BambooIcdCell cell,
        IReadOnlyList<BambooIcdMotion.MotionKey> keys,
        IReadOnlyList<BambooTrussMotion.CycleBound> cycles,
        IReadOnlyList<BambooTrussLayout.MemberDef>? members = null,
        double gLmm = 24,
        double? maxStepDeg = null,
        MotionConstraints? constraints = null)
    {
        _cell = cell;
        var profile = constraints ?? MotionConstraints.Current;
        Constraints = maxStepDeg is { } step ? profile with { ScanStepDeg = step } : profile;
        _keys = keys;
        _cycles = cycles;
        _members = members ?? BambooTrussLayout.Members;
        var placed = new List<(string, double[])>();
        foreach (var c in _cycles)
        {
            var member = _members.FirstOrDefault(m => m.Id == c.MemberId) ?? (BambooTrussLayout.IsRelay(c.MemberId) ? BambooTrussLayout.Def(c.MemberId) : null);
            if (member is null)
            {
                // start / walk: nothing new in hand, the stock is full (next cane on the pegs).
                _world.Add(new CycleWorld { Placed = placed.ToList(), Rest = BambooIcdCell.PlacementWorld(BambooTrussLayout.Store) });
                continue;
            }
            var rest = BambooIcdCell.PlacementWorld(member.Store);
            // A cane parked in a connector for this member: it is picked from there (and leaves the placed set).
            var parked = placed.FindIndex(p => p.Item1.StartsWith("placed-" + member.Id + ">", StringComparison.Ordinal));
            if (parked >= 0) { rest = placed[parked].Item2; placed.RemoveAt(parked); }
            Frame? held = null; double[]? released = null; string? miss = null;
            var close = FindKey(c, ":Close", k => k.Hold == 1);
            var open = FindKey(c, ":Open", k => k.Hold == 2);
            if (close is null) miss = "no grasp";
            else
            {
                var jaw = BambooTrussPlanner.ToMotus(RWorld(close));
                var offset = Transforms.Multiply(Transforms.Inverse(jaw), rest);
                // A true grasp leaves the cane between the jaw plates: on the slot axis, centred in the jaws.
                var radial = Math.Sqrt(offset[3] * offset[3] + offset[7] * offset[7]);
                var axisDot = Math.Abs(offset[10]);
                if (radial > K.GripPosTolM || axisDot < Math.Cos(K.GripAxisTolDeg * Math.PI / 180))
                    miss = $"grasp misses strut ({radial * 1000:F2} mm off the slot axis, {Math.Acos(Math.Min(1, axisDot)) * 180 / Math.PI:F2}° off)";
                held = Transforms.ToFrame(offset);
                if (open is not null) released = Transforms.Multiply(BambooTrussPlanner.ToMotus(RWorld(open)), offset);
            }
            _world.Add(new CycleWorld { ActiveId = member.Id, Rest = rest, HeldOffset = held, Released = released, Placed = placed.ToList(), Miss = miss });
            if (released is not null) placed.Add(("placed-" + member.Id, released));
        }
        cell.SetMountViewer(BambooTrussLayout.StartBase);
        BuildSamples();
    }

    /// <summary>Motion rules the scan samples by (canonical points: <see cref="BambooTrussCollision.CanonicalCount"/>).</summary>
    public MotionConstraints Constraints { get; }

    BambooIcdMotion.MotionKey? FindKey(BambooTrussMotion.CycleBound c, string suffix, Func<BambooIcdMotion.MotionKey, bool> pred)
    {
        for (var i = c.FirstKey; i <= c.LastKey && i < _keys.Count; i++)
            if (_keys[i].Label.EndsWith(suffix, StringComparison.Ordinal) && pred(_keys[i])) return _keys[i];
        return null;
    }

    static double[] RWorld(BambooIcdMotion.MotionKey k) => BambooTrussPlanner.RWorld(k.Base, k.BaseWorld!, k.Q);

    /// <summary>
    /// Sample times: on every key segment exactly the canonical joint-space fractions the planner verifies
    /// (<see cref="BambooTrussCollision.CanonicalCount"/>), mapped through the playback easing to times.
    /// </summary>
    void BuildSamples()
    {
        _ts.Add(0);
        for (var i = 0; i + 1 < _keys.Count; i++)
        {
            var a = _keys[i]; var b = _keys[i + 1];
            var dq = K.JointDist(a.Q, b.Q);
            var dg = Math.Max(Math.Abs(b.GR - a.GR), Math.Abs(b.GL - a.GL));
            var n = BambooTrussCollision.CanonicalCount(dq, dg, Constraints);
            for (var s = 1; s <= n; s++) _ts.Add(a.T + (b.T - a.T) * BambooTrussCollision.TimeForFraction(s / (double)n));
        }
    }

    /// <summary>Last Motus-verified motion time (1 once the whole path is clear). Playback never runs past it.</summary>
    public double Frontier =>
        FirstHit is not null ? Math.Max(0, LastClearT)
        : _next >= _ts.Count ? 1.0
        : Math.Max(0, LastClearT);

    public void AdvanceTo(double t)
    {
        while (FirstHit is null && _next < _ts.Count && _ts[_next] <= t + 1e-12) CheckNext();
    }

    public void Step(int budget)
    {
        for (var i = 0; i < budget && FirstHit is null && _next < _ts.Count; i++) CheckNext();
    }

    /// <summary>Run Motus checks for at most <paramref name="budgetMs"/> wall-clock ms (WASM UI thread).</summary>
    public void StepFor(double budgetMs, double untilT = 1.0)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        while (FirstHit is null && _next < _ts.Count && _ts[_next] <= untilT + 1e-12)
        {
            CheckNext();
            if (sw.Elapsed.TotalMilliseconds >= budgetMs) break;
        }
    }

    public double Progress => _ts.Count == 0 ? 1 : Math.Min(1, _next / (double)_ts.Count);

    public BambooTrussScan RunToEnd() { AdvanceTo(1.0); return this; }

    /// <summary>Skip (treat as verified) every sample at or before <paramref name="t"/> — e.g. an already-scanned plan prefix.</summary>
    public BambooTrussScan SkipTo(double t)
    {
        while (_next < _ts.Count && _ts[_next] <= t + 1e-12) { LastClearT = _ts[_next]; _next++; }
        return this;
    }

    /// <summary>Base (clamped) gripper at t: which one, its fixed world frame (viewer) and what it is clamped on.</summary>
    public (string Gripper, double[] World, string On) BaseAt(double t)
    {
        if (_keys.Count == 0) return ("L", BambooTrussLayout.StartBase, BambooTrussLayout.S1.Name);
        var s = BambooTrussMotion.SampleState(_keys, t);
        var k = _keys[s.SegEnd];
        return (k.Base ?? "L", k.BaseWorld ?? BambooTrussLayout.StartBase, k.BaseOn ?? BambooTrussLayout.S1.Name);
    }

    /// <summary>L-grip (mount) and R-grip world frames (viewer 4×4) at t, by FK from the clamped base.</summary>
    public (double[] L, double[] R) GrippersAt(double t, IReadOnlyList<double>? qOverride = null)
    {
        var (g, w, _) = BaseAt(t);
        var q = qOverride ?? BambooTrussMotion.SampleState(_keys, t).Q;
        return (BambooTrussPlanner.LWorld(g, w, q), BambooTrussPlanner.RWorld(g, w, q));
    }

    /// <summary>One clamped gripper for the viewer marker: which gripper, what it is on, its grip frame (viewer), whether it is the IK base.</summary>
    public readonly record struct Clamp(string Gripper, string On, double[] Frame, bool IsBase, bool Held);

    /// <summary>
    /// Every gripper clamped at t (jaws at the clamp width on an object): the base always, plus the free gripper when
    /// it is closed on something — a stake or placed cane (fixed) or the cane in hand (held). Both show when both clamp.
    /// </summary>
    public List<Clamp> ClampsAt(double t, IReadOnlyList<double> qDeg, double gR, double gL)
    {
        var list = new List<Clamp>();
        var (g, _, on) = BaseAt(t);
        var (lw, rw) = GrippersAt(t, qDeg);
        list.Add(new Clamp(g, on, g == "R" ? rw : lw, true, false));
        if (_keys.Count == 0) return list;
        var k = _keys[BambooTrussMotion.SampleState(_keys, t).SegEnd];
        var closed = BambooTrussLayout.JawClosedMm + 0.5;
        void Add(string grip, string? obj, double jaw, double[] frame)
        {
            if (grip == g || obj is null || jaw > closed) return;
            var name = obj.Split('|')[^1];
            list.Add(new Clamp(grip, name, frame, false, name == BambooIcdCell.Strut));
        }
        Add("L", k.LOn, gL, lw);
        Add("R", k.ROn, gR, rw);
        return list;
    }

    /// <summary>Environment (everything that is not the robot) at t for a hold state.</summary>
    public List<CollisionObject> EnvironmentAt(double t, int hold)
    {
        if (_cycles.Count == 0)
        {
            var env = new List<CollisionObject>(_cell.GroundObjects);
            env.AddRange(BambooTrussLayout.FixtureObjects);
            return env;
        }
        var w = _world[BambooTrussMotion.CycleIndexAt(_keys, _cycles, t)];
        return BambooTrussCollision.Environment(_cell, w.Placed, w.ActiveId, w.Rest, w.Released, w.HeldOffset is not null, hold);
    }

    /// <summary>Viewer poses for every cane (placed members, active cane on the feed / held / released).</summary>
    /// <summary>Inner-node connectors present at t (for drawing; same rule as <see cref="EnvironmentAt"/>).</summary>
    public List<string> HubsAt(double t, int hold)
    {
        if (_cycles.Count == 0) return new List<string>();
        var w = _world[BambooTrussMotion.CycleIndexAt(_keys, _cycles, t)];
        var ids = w.Placed.Select(p => p.Name["placed-".Length..]).ToList();
        if (w.ActiveId is not null && hold == 2 && w.Released is not null) ids.Add(w.ActiveId);
        return BambooTrussLayout.InnerHubNodes(ids);
    }

    public List<(string Name, double[] Pose)> StrutPoses(double t, int hold, IReadOnlyList<double> qDeg)
    {
        var list = new List<(string, double[])>();
        if (_cycles.Count == 0) return list;
        var w = _world[BambooTrussMotion.CycleIndexAt(_keys, _cycles, t)];
        foreach (var (name, m) in w.Placed) list.Add((name, BambooIcdCell.ToViewerPose(m)));
        if (w.ActiveId is null)
        {
            if (w.Rest is not null) list.Add((BambooIcdCell.Strut, BambooIcdCell.ToViewerPose(w.Rest)));
            return list;
        }
        if (hold == 1 && w.HeldOffset is { } off)
        {
            var (_, r) = GrippersAt(t, qDeg);
            list.Add((BambooIcdCell.Strut, BambooIcdCell.ToViewerPose(Transforms.Multiply(BambooTrussPlanner.ToMotus(r), Transforms.FromFrame(off)))));
        }
        else if (hold == 2 && w.Released is not null) list.Add(("placed-" + w.ActiveId, BambooIcdCell.ToViewerPose(w.Released)));
        else if (w.Rest is not null) list.Add((BambooIcdCell.Strut, BambooIcdCell.ToViewerPose(w.Rest)));
        return list;
    }

    /// <summary>Motus contacts for an arbitrary pose at motion time t (manual sliders use the plan's base at t).</summary>
    public IReadOnlyList<CollisionContact> ContactsAt(double t, IReadOnlyList<double> q, double gR, double gL, int hold, int max = 32)
    {
        var s = BambooTrussMotion.SampleState(_keys, t);
        var k = _keys.Count > 0 ? _keys[s.SegEnd] : null;
        var (g, w, _) = BaseAt(t);
        var world = _cycles.Count > 0 ? _world[BambooTrussMotion.CycleIndexAt(_keys, _cycles, t)] : null;
        var held = hold == 1 ? world?.HeldOffset : null;
        return BambooTrussCollision.Contacts(_cell, g, w, q, gR, gL, held, EnvironmentAt(t, hold), k?.LOn, k?.ROn, max);
    }

    void CheckNext()
    {
        var t = _ts[_next];
        var s = BambooTrussMotion.SampleState(_keys, t);
        var contacts = ContactsAt(t, s.Q, s.GR, s.GL, s.Hold);
        if (contacts.Count > 0)
        {
            FirstHit = new Hit(t, LastClearT, s.Label, s.Q, contacts);
            return;
        }
        LastClearT = t;
        _next++;
    }

    public string? TrackMiss =>
        _world.Select((w, i) => w.Miss is { } m ? $"{_cycles[i].MemberId}: {m}" : null).FirstOrDefault(x => x is not null);
}
