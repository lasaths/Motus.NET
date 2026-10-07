using Motus.Core;

namespace Motus.Viewer.Services;

/// <summary>
/// Motus collision scan for a multi-member store→truss motion. Each cycle feeds one strut at the
/// shared store pose and places it at its truss member pose. Already-placed members stay in the
/// scene as named obstacles; waiting stock is not presented until its cycle (single-feed store).
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
    /// <summary>Motion times Motus checks, ascending.</summary>
    public IReadOnlyList<double> SampleTimes => _ts;
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
            if (_cycles[i].MemberId == "brachiate")
            {
                // Empty-hand hang-walk: reuse store track with Hold=0 keys (no cane in jaws).
                var cycleKeys = KeysInCycle(i);
                var brachTrack = cell.TrackStrut(cycleKeys, BambooTrussLayout.Store);
                cell.SetHang(_cycles[i].PickHang);
                _tracks.Add(brachTrack);
                continue;
            }
            var member = _members.First(m => m.Id == _cycles[i].MemberId);
            var memberKeys = KeysInCycle(i);
            cell.SetHang(_cycles[i].PickHang);
            var track = cell.TrackStrut(memberKeys, member.Store);
            // Held transit uses the store-grasp jaw offset; snap the Motus release pose to the designed
            // edge place so later cycles collide with the true member (pitch changes store→vertical plane).
            if (track.Miss is null && track.Released is not null)
            {
                track = new BambooIcdCell.StrutTrack
                {
                    Rest = track.Rest,
                    HeldOffset = track.HeldOffset,
                    Released = BambooIcdCell.PlacementWorld(member.Place)
                };
            }
            _tracks.Add(track);
        }
        cell.SetHang(BambooTrussLayout.StartDock);
        BuildSamples(maxStepDeg);
    }

    List<BambooIcdMotion.MotionKey> KeysInCycle(int index)
    {
        var c = _cycles[index];
        var span = Math.Max(1e-9, c.T1 - c.T0);
        var list = new List<BambooIcdMotion.MotionKey>();
        // Cycle joins share a boundary T (the next Home is stamped 1e-6 after the previous Retract), so
        // the T window alone would hand this cycle its neighbour's Retract/Home. Keys are labelled
        // "memberId:Label" — keep only this member's keys inside the window.
        var prefix = c.MemberId + ":";
        foreach (var k in _keys)
        {
            if (k.T < c.T0 - 1e-9) continue;
            if (k.T > c.T1 + 1e-9) break;
            if (!k.Label.StartsWith(prefix, StringComparison.Ordinal)) continue;
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
            // Brachiation moves the hang (mount) with the joints parked at Home: dq = 0, so sample the
            // hang translation instead — Motus must check the whole hang-walk, not just its waypoints.
            var ha = HangAt(a.T); var hb = HangAt(b.T);
            var dh = Math.Sqrt((hb.X - ha.X) * (hb.X - ha.X) + (hb.Y - ha.Y) * (hb.Y - ha.Y) + (hb.Z - ha.Z) * (hb.Z - ha.Z));
            n = Math.Max(n, (int)Math.Ceiling(dh / MaxHangStepM));
            n = Math.Max(1, n);
            for (var s = 1; s <= n; s++)
                _ts.Add(a.T + (b.T - a.T) * s / n);
        }
    }

    /// <summary>Max hang (mount) translation between Motus checks while brachiating (metres).</summary>
    public const double MaxHangStepM = 0.004;

    /// <summary>
    /// Last motion time Motus has verified clear (1 once the whole path is checked clear). Playback must
    /// not run ahead of this — it waits for the scan instead of forcing it synchronously.
    /// </summary>
    public double Frontier =>
        FirstHit is not null ? Math.Max(0, LastClearT)
        : _next >= _ts.Count ? 1.0
        : Math.Max(0, LastClearT);

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

    /// <summary>
    /// Run Motus checks for at most <paramref name="budgetMs"/> wall-clock milliseconds (WASM UI thread:
    /// a fixed sample count can take seconds in expensive stretches and freeze the page).
    /// </summary>
    public void StepFor(double budgetMs, double untilT = 1.0)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        while (FirstHit is null && _next < _ts.Count && _ts[_next] <= untilT + 1e-12)
        {
            CheckNext();
            if (sw.Elapsed.TotalMilliseconds >= budgetMs) break;
        }
    }

    /// <summary>Fraction of Motus samples checked (0..1).</summary>
    public double Progress => _ts.Count == 0 ? 1 : Math.Min(1, _next / (double)_ts.Count);

    public BambooTrussScan RunToEnd()
    {
        AdvanceTo(1.0);
        return this;
    }

    /// <summary>Scene extras (already placed members) and the active strut track for time t.</summary>
    public (BambooIcdCell.StrutTrack Track, List<(string Name, BambooIcdCell.StrutPlacement Pose)> Extras, int MemberIndex)
        WorldAt(double t, int hold)
    {
        var mi = BambooTrussMotion.MemberIndexAt(_cycles, t);
        var placedCount = 0;
        for (var ci = 0; ci < mi; ci++)
            if (_cycles[ci].MemberId != "brachiate") placedCount++;
        var extras = new List<(string, BambooIcdCell.StrutPlacement)>();
        for (var i = 0; i < _members.Count; i++)
        {
            var m = _members[i];
            // Waiting stock is not Motus-present yet (single feed). Only prior placements collide.
            if (i < placedCount)
                extras.Add(($"placed-{m.Id}", m.Place));
        }
        var track = mi >= 0 && mi < _tracks.Count ? _tracks[mi] : _tracks[0];
        return (track, extras, mi);
    }

    /// <summary>Viewer poses for every strut (active cane at the feed / held / released, plus placed members).</summary>
    public List<(string Name, double[] Pose)> StrutPoses(double t, int hold, IReadOnlyList<double> qDeg)
    {
        var (track, extras, mi) = WorldAt(t, hold);
        var list = new List<(string, double[])>();
        foreach (var (name, pose) in extras)
            list.Add((name, BambooIcdCell.ToViewerPose(BambooIcdCell.PlacementWorld(pose))));

        if (mi >= 0 && mi < _tracks.Count && mi < _cycles.Count && _cycles[mi].MemberId != "brachiate")
        {
            _cell.SetHang(HangAt(t));
            var world = _cell.StrutWorld(track, hold, qDeg);
            var name = hold == 2 ? $"placed-{_cycles[mi].MemberId}" : BambooIcdCell.Strut;
            list.Add((name, BambooIcdCell.ToViewerPose(world)));
        }
        return list;
    }

    void CheckNext()
    {
        var t = _ts[_next];
        var s = BambooIcdMotion.Sample(_keys, t);
        var (track, extras, mi) = WorldAt(t, s.Hold);
        if (mi >= 0 && mi < _cycles.Count)
            _cell.SetHang(HangAt(_cycles[mi], StripMember(s.Label), t));
        var contacts = _cell.Contacts(s.Q, s.GR, _gL, s.Hold, StripMember(s.Label), track, extras);
        if (contacts.Count > 0)
        {
            FirstHit = new Hit(t, LastClearT, s.Label, s.Q, contacts);
            return;
        }
        LastClearT = t;
        _next++;
    }

    /// <summary>Hang (L-grip mount) at motion time t — the viewer draws the mount here.</summary>
    public BambooIcdMotion.HangPose HangAt(double t)
    {
        if (_cycles.Count == 0) return BambooTrussLayout.StartDock;
        var mi = BambooTrussMotion.MemberIndexAt(_cycles, t);
        var label = StripMember(BambooIcdMotion.Sample(_keys, t).Label);
        return HangAt(_cycles[mi], label, t);
    }

    static BambooIcdMotion.HangPose HangAt(BambooTrussMotion.CycleBound cycle, string label, double t)
    {
        if (cycle.MemberId == "brachiate")
        {
            var path = BambooTrussLayout.BrachiateHangPath;
            if (path.Count == 0) return BambooTrussLayout.WalkHang;
            if (path.Count == 1) return path[0];
            var span = Math.Max(1e-9, cycle.T1 - cycle.T0);
            var u = Math.Clamp((t - cycle.T0) / span, 0, 1);
            var x = u * (path.Count - 1);
            var i = Math.Min((int)x, path.Count - 2);
            var f = x - i;
            var a = path[i]; var b = path[i + 1];
            return new BambooIcdMotion.HangPose(
                a.X + (b.X - a.X) * f,
                a.Y + (b.Y - a.Y) * f,
                a.Z + (b.Z - a.Z) * f);
        }
        if (label is "Transfer" or "Hold" or "Lower" or "Open" or "Depart" or "Retract"
            or "place:hold" or "place:release" or "Swing" or "Pitch")
            return cycle.PlaceHang;
        return cycle.PickHang;
    }

    public string? TrackMiss =>
        _tracks.Select((tr, i) => tr.Miss is { } m ? $"{_cycles[i].MemberId}: {m}" : null)
            .FirstOrDefault(x => x is not null);
}
