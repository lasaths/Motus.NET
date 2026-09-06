namespace Motus.Core;

/// <summary>
/// Opt-in counters for profiling a single Plan() call (collision checks, RRT iterations, LIN IK
/// attempts). Disabled by default — <see cref="Wrap"/> returns the checker unchanged and hot-path
/// increments are skipped, so there is no cost unless a caller explicitly enables this for
/// benchmarking. Not meant for concurrent multi-plan use; enable only around a single Plan() call.
/// </summary>
public static class PlanningDiagnostics
{
    public static bool Enabled { get; set; }
    public static long CollisionChecks;
    public static long SegmentChecks;
    public static long IkAttempts;
    public static long RrtIterations;

    public static void Reset()
    {
        CollisionChecks = 0;
        SegmentChecks = 0;
        IkAttempts = 0;
        RrtIterations = 0;
    }

    public static void RecordIkAttempt()
    {
        if (Enabled) IkAttempts++;
    }

    public static void RecordRrtIteration()
    {
        if (Enabled) RrtIterations++;
    }

    /// <summary>Wraps a checker with call counters when diagnostics are enabled; returns the same
    /// instance otherwise so normal planning has zero overhead.</summary>
    public static ICollisionChecker Wrap(ICollisionChecker checker) =>
        Enabled ? new CountingCollisionChecker(checker) : checker;

    private sealed class CountingCollisionChecker : ICollisionChecker
    {
        private readonly ICollisionChecker _inner;
        public CountingCollisionChecker(ICollisionChecker inner) => _inner = inner;

        public bool IsCollisionFree(JointState state, CollisionScene scene)
        {
            CollisionChecks++;
            return _inner.IsCollisionFree(state, scene);
        }

        public bool SegmentCollisionFree(JointState from, JointState to, CollisionScene scene, double configurationStep)
        {
            SegmentChecks++;
            return _inner.SegmentCollisionFree(from, to, scene, configurationStep);
        }
    }
}
