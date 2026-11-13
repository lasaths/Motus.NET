using Motus.Core;

namespace Motus.Construction.Search;

/// <summary>Outcome of one task. Ok = false: the task was not planned (<see cref="Note"/> says why).</summary>
internal readonly record struct TaskStep(bool Ok, string? Note);

/// <summary>
/// Runs core tasks (<see cref="TaskInstance"/>) one at a time on a planner. A search confirms its choices through one, and
/// a fresh one replays the result, so "the search result rebuilds in a fresh replay" is checked against this contract.
/// </summary>
internal interface ITaskRunner
{
    TaskStep Run(TaskInstance task);
    /// <summary>Objects built so far, in order.</summary>
    IReadOnlyList<string> Built { get; }
}

internal static class Replay
{
    /// <summary>
    /// Run <paramref name="tasks"/> on a fresh runner. Null when every task is planned and the built objects are
    /// <paramref name="expected"/> in order; otherwise why not.
    /// </summary>
    public static string? Mismatch(ITaskRunner fresh, IEnumerable<TaskInstance> tasks, IEnumerable<string> expected)
    {
        foreach (var t in tasks)
            if (fresh.Run(t) is { Ok: false } s) return s.Note ?? "a task was not planned";
        var want = expected.ToList();
        return fresh.Built.SequenceEqual(want) ? null : $"built [{string.Join(", ", fresh.Built)}], expected [{string.Join(", ", want)}]";
    }
}
