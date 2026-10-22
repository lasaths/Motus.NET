using Motus.Core;

namespace Motus.Viewer.Services;

/// <summary>
/// Incremental Motus collision scan along a sampled bamboo motion (play / scrub / tests share it).
/// Samples are spaced so no joint moves more than <c>maxStepDeg</c> and the R-grip no more than 0.25 mm
/// between checks (smoothstep easing peaks at 1.5× — accounted for). The scan stops at the first sample
/// Motus reports colliding; <see cref="LastClearT"/> is the last verified-clear sample before it.
/// </summary>
public sealed class BambooMotionScan
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
    public BambooIcdCell.StrutTrack Track { get; }
    private readonly double _gL;
    private readonly List<double> _ts = new();
    private int _next;

    public double LastClearT { get; private set; } = -1;
    public Hit? FirstHit { get; private set; }
    public bool Complete => FirstHit is not null || _next >= _ts.Count;
    public int SampleCount => _ts.Count;
    public int Checked => _next;

    public BambooMotionScan(
        BambooIcdCell cell,
        IReadOnlyList<BambooIcdMotion.MotionKey> keys,
        BambooIcdCell.StrutPlacement strutRest,
        double gLmm = 24,
        double maxStepDeg = 0.25)
    {
        _cell = cell;
        _keys = keys;
        Track = cell.TrackStrut(keys, strutRest);
        _gL = gLmm;
        BuildSamples(maxStepDeg);
    }

    private void BuildSamples(double maxStepDeg)
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

    /// <summary>Check every sample with t ≤ <paramref name="t"/> (stops at the first hit).</summary>
    public void AdvanceTo(double t)
    {
        while (FirstHit is null && _next < _ts.Count && _ts[_next] <= t + 1e-12)
            CheckNext();
    }

    /// <summary>Check up to <paramref name="budget"/> samples (for chunked background scanning).</summary>
    public void Step(int budget)
    {
        for (var i = 0; i < budget && FirstHit is null && _next < _ts.Count; i++)
            CheckNext();
    }

    public BambooMotionScan RunToEnd()
    {
        AdvanceTo(1.0);
        return this;
    }

    private void CheckNext()
    {
        var t = _ts[_next];
        var s = BambooIcdMotion.Sample(_keys, t);
        var contacts = _cell.Contacts(s.Q, s.GR, _gL, s.Hold, s.Label, Track);
        if (contacts.Count > 0)
        {
            FirstHit = new Hit(t, LastClearT, s.Label, s.Q, contacts);
            return;
        }
        LastClearT = t;
        _next++;
    }

    /// <summary>Clamp a requested t to the verified-clear prefix (after advancing the scan to it).</summary>
    public double ClampToClear(double t)
    {
        AdvanceTo(t);
        if (FirstHit is null) return t;
        return Math.Min(t, Math.Max(0, LastClearT));
    }
}
