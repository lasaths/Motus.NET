namespace Motus.Viewer.Services;

/// <summary>
/// Play clock for /bamboo-truss. Keeps motion time as a double (the page used to round it into the
/// 0..1000 slider each frame), never crosses more than one motion key per <see cref="Advance"/> call,
/// parks on every key for <see cref="MinDwellSec"/> so each phase is drawn on screen, and never runs
/// past the Motus-verified frontier. A stalled frame (large dt) can therefore never skip a block.
/// </summary>
public sealed class BambooTrussPlayback
{
    /// <summary>Wall-clock seconds for the full 0→1 motion (excluding dwells).</summary>
    public const double DurationSec = 45.0;
    /// <summary>Minimum on-screen time parked on each reached key (seconds).</summary>
    public const double MinDwellSec = 0.35;
    /// <summary>Largest dt applied to motion per frame (seconds) — smooth motion after a stall.</summary>
    public const double MaxStepSec = 0.05;

    private readonly IReadOnlyList<BambooIcdMotion.MotionKey> _keys;
    private int _next;
    private double _dwell;

    public BambooTrussPlayback(IReadOnlyList<BambooIcdMotion.MotionKey> keys, double startT = 0)
    {
        _keys = keys;
        T = Math.Clamp(startT, 0, 1);
        _next = 0;
        // Keys at or before the start time count as already reached (resume mid-motion).
        while (_next < _keys.Count && _keys[_next].T < T - 1e-12) _next++;
    }

    /// <summary>Current motion time (0..1).</summary>
    public double T { get; private set; }

    /// <summary>True when every key (incl. the last at T=1) was reached and its dwell elapsed.</summary>
    public bool Done => _next >= _keys.Count && _dwell <= 0 && T >= 1 - 1e-12;

    /// <summary>True while playback is held back by the Motus frontier (scan not yet at the next step).</summary>
    public bool WaitingForMotus { get; private set; }

    /// <summary>Labels of keys reached so far, in order (one entry per key).</summary>
    public List<string> Reached { get; } = new();

    /// <summary>
    /// Advance by <paramref name="dtSec"/> wall seconds, but not past <paramref name="frontier"/>
    /// (last Motus-verified time). Returns the new motion time.
    /// </summary>
    public double Advance(double dtSec, double frontier = 1.0)
    {
        if (_keys.Count == 0) { T = 1; return T; }
        dtSec = Math.Max(0, dtSec);
        if (_dwell > 0)
        {
            _dwell -= dtSec;
            WaitingForMotus = false;
            return T;
        }
        if (_next >= _keys.Count) { T = 1; WaitingForMotus = false; return T; }

        var keyT = _keys[_next].T;
        var target = Math.Min(T + Math.Min(dtSec, MaxStepSec) / DurationSec, keyT);
        var limit = Math.Clamp(frontier, 0, 1);
        WaitingForMotus = target > limit + 1e-12;
        if (WaitingForMotus) target = Math.Max(T, limit);
        T = target;
        if (T >= keyT - 1e-12)
        {
            T = keyT;
            Reached.Add(_keys[_next].Label);
            _next++;
            _dwell = MinDwellSec;
        }
        return T;
    }
}
