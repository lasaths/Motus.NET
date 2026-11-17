using Motus.Core;
using Motus.Construction.Search;

namespace Motus.Construction;

/// <summary>
/// Runs task blocks (pick / place / park / walk / land) on a <see cref="BambooTrussPlanner"/>, one at a time. The one
/// interpreter of a task sequence: the replay (<see cref="BambooTrussMotion.RunSequence"/>) and the build search
/// (<see cref="BambooBuildSearch"/>, to confirm a choice) both run their tasks through it, so a choice the search keeps
/// is planned by the same calls the replay makes.
/// </summary>
internal sealed class BambooMoveRunner(BambooTrussPlanner p, IEnumerable<string>? built = null, Action<string>? log = null) : ITaskRunner
{
    /// <summary>Outcome of one task block. Ok = false: the block was not planned (<see cref="Note"/> says why).</summary>
    public readonly record struct Step(bool Ok, bool Walked, string? Note, int WalkSteps = 0, int Tried = 0);

    /// <summary>Walk steps per walk block (the replay and the search must find the same walk).</summary>
    public int MaxStepsPerWalk { get; init; } = 150;
    /// <summary>Time cap (s) of a walk block (<see cref="BambooWalk.Explore"/>).</summary>
    public double WalkSeconds { get; init; } = 90;
    /// <summary>Per-walk-block outcomes (benchmark table), or null.</summary>
    public List<BambooTrussMotion.WalkLeg>? Legs { get; init; }

    /// <summary>Members built so far (relay parks excluded), in order.</summary>
    public List<string> Built { get; } = built?.ToList() ?? new();

    PickTask? _pick;

    TaskStep ITaskRunner.Run(TaskInstance task) { var s = Run(task); return new(s.Ok, s.Note); }
    IReadOnlyList<string> ITaskRunner.Built => Built;

    /// <summary>Run one core task. A pick is remembered for the place / park that follows; it plans nothing by itself.</summary>
    public Step Run(TaskInstance c)
    {
        if (CoreTasks.AsPick(c) is { } pk) { _pick = pk; return new Step(true, false, null); }
        if (CoreTasks.AsPlace(c) is { } pl && _pick is not null && _pick.Object == pl.Object)
        {
            _pick = null;
            var m = BambooTrussLayout.Def(pl.Object);
            if (m is null) return new Step(false, false, $"{pl.Object}: not a truss member — skipped");
            return Build(m, "built");
        }
        if (CoreTasks.AsPark(c) is { } park && _pick is not null && _pick.Object == park.Object)
        {
            _pick = null;
            var m = BambooTrussLayout.Def(BambooTaskModel.RelayIdOf(park));
            if (m is null) return new Step(false, false, $"{park.Object}: cannot park at {park.Connector}/{park.Direction} — skipped");
            return Build(m, $"parked at {park.Connector}/{park.Direction}");
        }
        if (CoreTasks.AsWalk(c) is not { } walk) return new Step(true, false, null);
        if (walk.Mode == WalkMode.Land)
        {
            var land = BambooTrussMotion.LandStep(p, walk.Target, log);
            return new Step(land.Ok, land.Walked, land.Note);
        }
        return Walk(walk.Target, BambooTrussMotion.WalkTargetF(walk), walk.Gait);
    }

    /// <summary>Walk L to <paramref name="target"/> (anchor, or "placed-" + member) at clamp parameter <paramref name="f"/>: anchor → anchor stake walk, else <see cref="BambooWalk.Explore"/>.</summary>
    public Step Walk(string target, double f, string? gaitName)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var gait = BambooWalk.GaitNamed(gaitName);
        if (BambooTrussLayout.IsSupport(target) && BambooTrussLayout.IsSupport(p.LStake))
        {
            if (p.LStake == target) return new Step(true, false, null);
            var ok = p.WalkToAnchor(target);
            return new Step(ok, ok, ok ? $"walk → {target}: stake walk" : $"walk → {target}: not planned — {p.Error}");
        }
        bool Goal(BambooWalk.Site s) => s.On == target && (s.IsStake || Math.Abs(s.F - f) < 1e-9);
        var label = BambooTrussLayout.IsSupport(target) ? target : $"{target["placed-".Length..]}@{f:+0.00;-0.00}";
        var res = BambooWalk.Explore(p, gait, BambooTrussLayout.Current.WalkRAnchor, Goal, MaxStepsPerWalk, log, WalkSeconds, $"walk ({gait.Id}) → {label}");
        var path = res.Reached is { } r ? res.PathTo(r) : new List<BambooWalk.Step>();
        Legs?.Add(new BambooTrussMotion.WalkLeg($"{target}@{f:+0.00;-0.00}", gait.Id, res.Reached is not null, path.Count, res.Tried,
            path.Count == 0 ? 0 : path.Average(x => x.StrideM), sw.Elapsed.TotalSeconds, path, res));
        var note = res.Reached is { } rr
            ? $"walk ({gait.Id}) → {rr}: {path.Count} steps — " + string.Join(" → ", path.Select(x => $"{x.Kind} (R on {x.RAnchor}) to {x.To}"))
            : $"walk ({gait.Id}) → {target}@{f:+0.00;-0.00}: not reachable with this gait ({res.Tried} steps tried, {res.Reachable.Count} sites reached)";
        log?.Invoke(note);
        return new Step(res.Reached is not null, res.Reached is not null, note, path.Count, res.Tried);
    }

    Step Build(BambooTrussLayout.MemberDef m, string what)
    {
        var fx = BambooTrussLayout.ExistingNodes(Built);
        if (!fx.Contains(m.NodeA) && !fx.Contains(m.NodeB)) return new Step(false, false, $"{m.Id}: not built — neither end node exists yet");
        if (p.BuildMember(m)) { if (!BambooTrussLayout.IsRelay(m.Id)) Built.Add(m.Id); log?.Invoke($"{m.Id}: {what}"); return new Step(true, false, null); }
        log?.Invoke($"{m.Id}: {p.Error}");
        return new Step(false, false, $"{m.Id}: not built — {p.Error}");
    }
}
