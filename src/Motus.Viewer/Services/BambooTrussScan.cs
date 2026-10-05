using Motus.Core;

namespace Motus.Viewer.Services;

/// <summary>
/// Motus collision scan for a multi-member store→truss motion. Each cycle picks one store strut and
/// places it at its truss member pose; remaining store slots and already-placed members stay in the
/// scene as named obstacles.
/// </summary>
public sealed class BambooTrussScan
{
    public sealed record Hit(
        double T,
        double LastClearT,
        string Label,
        double[] QDeg,
        IReadOnlyList<CollisionContact> Contacts)
    {
        public string Describe() => BambooIcdCell.Describe(Contacts);
    }

    private readonly BambooIcdCell _cell;
    private readonly IReadOnlyList<BambooIcdMotion.MotionKey> _keys;
    private readonly IReadOnlyList<BambooTrussMotion.CycleBound> _cycles;
    private readonly IReadOnlyList<BambooTrussLayout.MemberDef> _members;
    private readonly List<BambooIcdCell.StrutTrack> _tracks;
    private readonly double _gL;
    private readonly List<double> _ts = new();
    private int _next;

    public double LastClearT { get; private set; } = -1;
    public Hit? FirstHit { get; private set; }
    public bool Complete => FirstHit is not null || _next >= _ts.Count;
    public int SampleCount => _ts.Count;
    public IReadOnlyList<BambooTrussMotion.CycleBound> Cycles => _cycles;

    public BambooTrussScan(
        BambooIcdCell cell,
        IReadOnlyList<BambooIcdMotion.MotionKey> keys,
        IReadOnlyList<BambooTrussMotion.CycleBound> cycles,
        IReadOnlyList<BambooTrussLayout.MemberDef>? members = null,
        double gLmm = 24,
        double maxStepDeg = 0.5)
    {
        _cell = cell;
        _keys = keys;
        _cycles = cycles;
        _members = members ?? BambooTrussLayout.Members;
        _gL = gLmm;
        _tracks = new List<BambooIcdCell.StrutTrack>();
        for (var i = 0; i < _cycles.Count; i++)
        {
            var member = _members.First(m => m.Id == _cycles[i].MemberId);
            var cycleKeys = KeysInCycle(i);
            _tracks.Add(cell.TrackStrut(cycleKeys, member.Store));
        }
        BuildSamples(maxStepDeg);
    }

    List<BambooIcdMotion.MotionKey> KeysInCycle(int index)
    {
        var c = _cycles[index];
        var span = Math.Max(1e-9, c.T1 - c.T0);
        var list = new List<BambooIcdMotion.MotionKey>();
        foreach (var k in _keys)
        {
            if (k.T < c.T0 - 1e-9) continue;
            if (k.T > c.T1 + 1e-9) break;
            list.Add(new BambooIcdMotion.MotionKey
            {
                T = (k.T - c.T0) / span,
                Label = StripMember(k.Label),
                Q = k.Q.ToArray(),
                GR = k.GR,
                Hold = k.Hold
            });
        }
        if (list.Count == 0)
        {
            list.Add(new BambooIcdMotion.MotionKey { T = 0, Label = "Home", Q = BambooIcdMotion.HomeDeg.ToArray(), GR = 40 });
            list.Add(new BambooIcdMotion.MotionKey { T = 1, Label = "Home", Q = BambooIcdMotion.HomeDeg.ToArray(), GR = 40 });
        }
        else
        {
            list[0].T = 0;
            list[^1].T = 1;
        }
        return list;
    }

    static string StripMember(string label)
    {
        var i = label.IndexOf(':');
        return i < 0 ? label : label[(i + 1)..];
    }

    void BuildSamples(double maxStepDeg)
    {
        _ts.Add(0);
        for (var i = 0; i + 1 < _keys.Count; i++)
        {
            var a = _keys[i]; var b = _keys[i + 1];
            var dq = 0.0;
            for (var k = 0; k < a.Q.Length; k++) dq = Math.Max(dq, Math.Abs(b.Q[k] - a.Q[k]));
            var dg = Math.Abs(b.GR - a.GR);
            var n = (int)Math.Ceiling(1.5 * Math.Max(dq / maxStepDeg, dg / 0.25));
            n = Math.Max(1, n);
            for (var s = 1; s <= n; s++)
                _ts.Add(a.T + (b.T - a.T) * s / n);
        }
    }

    public void AdvanceTo(double t)
    {
        while (FirstHit is null && _next < _ts.Count && _ts[_next] <= t + 1e-12)
            CheckNext();
    }

    public void Step(int budget)
    {
        for (var i = 0; i < budget && FirstHit is null && _next < _ts.Count; i++)
            CheckNext();
    }

    public BambooTrussScan RunToEnd()
    {
        AdvanceTo(1.0);
        return this;
    }

    /// <summary>Scene extras (named store leftovers + already placed members) and the active strut track for time t.</summary>
    public (BambooIcdCell.StrutTrack Track, List<(string Name, BambooIcdCell.StrutPlacement Pose)> Extras, int MemberIndex)
        WorldAt(double t, int hold)
    {
        var mi = BambooTrussMotion.MemberIndexAt(_cycles, t);
        var extras = new List<(string, BambooIcdCell.StrutPlacement)>();
        for (var i = 0; i < _members.Count; i++)
        {
            var m = _members[i];
            if (i > mi)
                extras.Add(($"store-{m.Id}", m.Store));
            else if (i < mi)
                extras.Add(($"placed-{m.Id}", m.Place));
            // i == mi: active Track owns rest / held / released (named "strut" in the scene).
        }
        var track = mi >= 0 && mi < _tracks.Count ? _tracks[mi] : _tracks[0];
        return (track, extras, mi);
    }

    /// <summary>Viewer poses for every strut (store remaining, held/released active, placed).</summary>
    public List<(string Name, double[] Pose)> StrutPoses(double t, int hold, IReadOnlyList<double> qDeg)
    {
        var (track, extras, mi) = WorldAt(t, hold);
        var list = new List<(string, double[])>();
        foreach (var (name, pose) in extras)
            list.Add((name, BambooIcdCell.ToViewerPose(BambooIcdCell.PlacementWorld(pose))));

        if (mi >= 0 && mi < _tracks.Count)
        {
            var world = _cell.StrutWorld(track, hold, qDeg);
            var name = hold == 2 && mi < _members.Count ? $"placed-{_members[mi].Id}" : BambooIcdCell.Strut;
            list.Add((name, BambooIcdCell.ToViewerPose(world)));
        }
        return list;
    }

    void CheckNext()
    {
        var t = _ts[_next];
        var s = BambooIcdMotion.Sample(_keys, t);
        var (track, extras, _) = WorldAt(t, s.Hold);
        var contacts = _cell.Contacts(s.Q, s.GR, _gL, s.Hold, StripMember(s.Label), track, extras);
        if (contacts.Count > 0)
        {
            FirstHit = new Hit(t, LastClearT, s.Label, s.Q, contacts);
            return;
        }
        LastClearT = t;
        _next++;
    }

    public string? TrackMiss =>
        _tracks.Select((tr, i) => tr.Miss is { } m ? $"{_cycles[i].MemberId}: {m}" : null)
            .FirstOrDefault(x => x is not null);
}
