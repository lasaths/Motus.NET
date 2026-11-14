namespace Motus.Construction.Search;

/// <summary>
/// One way to extend a search state. <see cref="Key"/> tells candidates of one state apart (failure memory);
/// <see cref="Rank"/>: lower is tried first; <see cref="Pre"/>: its cheap precondition, evaluated just before it is
/// confirmed; <see cref="Resumed"/>: the recorded choice of a resumed run, tried first and at the full budget.
/// <see cref="Label"/> and <see cref="Backtrack"/> are the log names of a try and of a dead end.
/// </summary>
internal sealed record Candidate<T>(T Value, string Key, string Label, string Backtrack, double Rank, Func<bool> Pre, bool Resumed);

internal static class Precondition
{
    /// <summary>Memoise a lazy precondition: evaluated once, when first asked.</summary>
    public static Func<bool> Once(Func<bool> pre) { bool? memo = null; return () => memo ??= pre(); }
}

internal enum Verdict { Yes, No, NotYet }

/// <summary>
/// A confirmer's answer. Yes: <see cref="Choice"/> is kept; <see cref="Places"/> = false is a move that places nothing
/// (it is handed to the next confirmation as the pending move). NotYet: a short budget ran out, which proves nothing.
/// </summary>
internal readonly record struct Confirmation<TChoice>(Verdict Verdict, TChoice? Choice, string Note, bool Places = true);

/// <summary>The planner state the search branches on. <see cref="Save"/> includes the random stream position, so a
/// restored branch replays bit-identically.</summary>
internal interface IPlanningState<TChoice, TSave>
{
    /// <summary>Build state after <paramref name="order"/>, for failure and dead-end memory.</summary>
    string Key(IReadOnlyList<TChoice> order);
    TSave Save();
    void Restore(TSave saved);
}

internal interface ICandidateGenerator<T, TChoice>
{
    /// <summary>Candidates after <paramref name="order"/> (and <paramref name="pending"/>, a kept move that placed nothing);
    /// <paramref name="onDeepest"/>: the state is at the depth of the best prefix.</summary>
    IReadOnlyList<Candidate<T>> For(IReadOnlyList<TChoice> order, TChoice? pending, bool onDeepest);
}

internal interface IConfirmer<T, TChoice>
{
    /// <summary>Confirm with the real planner; <paramref name="shortStage"/>: the short budget.</summary>
    Confirmation<TChoice> Confirm(T candidate, IReadOnlyList<TChoice> order, TChoice? pending, bool shortStage);
}

/// <summary>
/// Depth-first backtracking over planner states with confirmation budgets, a best prefix, resume, failure and dead-end
/// memory, and staged confirmation: a "not yet" candidate gets the full budget only on the deepest branch, after the other
/// candidates of its state, and once more in a second pass if the search ends exhausted with some left off it.
/// </summary>
internal sealed class BacktrackSearch<T, TChoice, TSave>(IPlanningState<TChoice, TSave> state, ICandidateGenerator<T, TChoice> generator,
    IConfirmer<T, TChoice> confirmer, BacktrackSearch<T, TChoice, TSave>.Settings s) where TChoice : class
{
    internal sealed record Settings(int Total, int MaxRejected, int MaxConfirmations, Action<string> Log)
    {
        public bool Staged { get; init; }
        public bool FailureMemory { get; init; }
        public bool DeadEndMemory { get; init; }
        /// <summary>Every confirmation that is not "not yet": candidate, order before it, kept, seconds, node, position in the node's sorted order.</summary>
        public Action<Candidate<T>, IReadOnlyList<TChoice>, bool, double, int, int>? Outcome { get; init; }
        /// <summary>Every expansion, before any try: node id, its candidates in try order, order before it. Observes only.</summary>
        public Action<int, IReadOnlyList<Candidate<T>>, IReadOnlyList<TChoice>>? Sorted { get; init; }
        /// <summary>After every placement: the order and whether it is complete.</summary>
        public Action<IReadOnlyList<TChoice>, bool>? Placed { get; init; }
    }

    internal sealed record Result(bool Complete, List<TChoice> Order, int Confirmed, int Rejected, int Deferred, string StopReason);

    readonly List<TChoice> _order = new();
    List<TChoice> _best = new();
    readonly HashSet<string> _failed = new(StringComparer.Ordinal), _deadEnds = new(StringComparer.Ordinal);
    readonly System.Diagnostics.Stopwatch _clock = new();
    int _confirmed, _rejected, _deferred, _parked, _nodes;
    bool _escalateAll;
    string _stop = "search exhausted";

    public Result Run()
    {
        _clock.Start();
        var complete = Dfs(null);
        if (!complete && _parked > 0 && !BudgetReached())
        {
            s.Log($"build search: second pass — {_parked} \"not yet\" candidates off the deepest branch get the full budget");
            _escalateAll = true;
            complete = Dfs(null);
        }
        return new Result(complete, complete ? _order : _best, _confirmed, _rejected, _deferred, complete ? "complete" : _stop);
    }

    bool BudgetReached()
    {
        if (_rejected >= s.MaxRejected) { _stop = $"rejected confirmation limit ({s.MaxRejected})"; return true; }
        if (_confirmed + _rejected >= s.MaxConfirmations) { _stop = $"confirmation limit ({s.MaxConfirmations})"; return true; }
        return false;
    }

    bool Dfs(TChoice? pending)
    {
        if (_order.Count == s.Total) return true;
        if (BudgetReached()) return false;
        var key = s.FailureMemory || s.DeadEndMemory ? state.Key(_order) : "";
        if (s.DeadEndMemory && _deadEnds.Contains(key)) { s.Log("  tamp: build state already found to be a dead end — skipped"); return false; }
        // OrderBy is stable, so this is the same order the loop below used before it was materialised.
        var sorted = generator.For(_order, pending, onDeepest: _order.Count == _best.Count)
            .OrderBy(x => x.Resumed ? 0 : 1).ThenBy(x => x.Rank).ToList();
        var node = _nodes++;
        s.Sorted?.Invoke(node, sorted, _order);
        var notYet = new List<(Candidate<T> C, int Pos)>();

        // One confirmation; true when the search completes below it.
        bool Try(Candidate<T> c, int pos, bool shortStage)
        {
            var cp = state.Save();
            var t0 = _clock.Elapsed.TotalSeconds;
            var r = confirmer.Confirm(c.Value, _order, pending, shortStage);
            if (r.Verdict == Verdict.NotYet)
            {
                state.Restore(cp);
                _deferred++;
                notYet.Add((c, pos));
                s.Log($"  {c.Label}: not yet — {r.Note} after {_clock.Elapsed.TotalSeconds - t0:F1} s");
                return false;
            }
            s.Outcome?.Invoke(c, _order, r.Verdict == Verdict.Yes, _clock.Elapsed.TotalSeconds - t0, node, pos);
            if (r.Verdict == Verdict.No) { _rejected++; _failed.Add($"{key}#{c.Key}"); state.Restore(cp); return false; }
            _confirmed++;
            if (!r.Places)
            {
                s.Log($"  {r.Note}");
                if (Dfs(r.Choice)) return true;
                s.Log($"backtrack: {c.Backtrack} leads to a dead end");
                state.Restore(cp);
                return false;
            }
            _order.Add(r.Choice!);
            if (_order.Count > _best.Count) _best = _order.ToList();
            s.Log($"{_order.Count}/{s.Total} {r.Note}");
            s.Placed?.Invoke(_order, _order.Count == s.Total);
            if (Dfs(null)) return true;
            s.Log($"backtrack: {c.Backtrack} leads to a dead end");
            _order.RemoveAt(_order.Count - 1);
            state.Restore(cp);
            return false;
        }

        for (var pos = 0; pos < sorted.Count; pos++)
        {
            var c = sorted[pos];
            if (BudgetReached()) break;
            if (!c.Pre()) continue;
            if (s.FailureMemory && _failed.Contains($"{key}#{c.Key}")) { s.Log($"  {c.Label}: already rejected in this build state — skipped"); continue; }
            if (Try(c, pos, s.Staged && !c.Resumed)) return true;
        }
        // "Not yet" candidates: the full budget only on the deepest branch (LazySP: evaluate only the current best plan).
        var parkedHere = false;
        foreach (var (c, pos) in notYet.ToList())
        {
            if (!_escalateAll && _order.Count < _best.Count)
            { _parked++; parkedHere = true; s.Log($"  {c.Label}: not yet, off the deepest branch — left for a later pass"); continue; }
            if (BudgetReached()) break;
            s.Log($"  {c.Label}: retry at the full RRT budget");
            if (Try(c, pos, false)) return true;
        }
        if (s.DeadEndMemory && !parkedHere && !BudgetReached()) _deadEnds.Add(key);
        return false;
    }
}
