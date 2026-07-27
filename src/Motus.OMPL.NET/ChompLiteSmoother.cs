using Motus.Core;

namespace Motus.OMPL.NET;

/// <summary>
/// Lightweight geometric CHOMP-style smoother for managed paths.
/// </summary>
/// <remarks>
/// This is intentionally "lite": collision cost is sampled by finite differences against the existing
/// binary validity checker. Future richer signed-distance backends could replace the penalty function;
/// TrajOpt and GPMP2 are documented alternatives for optimization-based smoothing.
/// </remarks>
public static class ChompLiteSmoother
{
    public static IReadOnlyList<JointState> Smooth(
        IReadOnlyList<JointState> path,
        RobotModel robot,
        ICollisionChecker? checker,
        CollisionScene? scene,
        SamplingPlannerOptions? options = null)
    {
        options ??= new SamplingPlannerOptions();
        var constraints = new PlanningPipeline.ConstraintContext(false, null, robot.Preset.BaseFrame, robot.Preset.ToolFrame, null, null);
        var qPath = path.Select(p => p.Positions.ToArray()).ToList();
        var smoothed = SmoothInternal(
            qPath,
            robot.Preset.JointLimits,
            scene ?? new CollisionScene(),
            checker,
            constraints,
            q => new JointState(q),
            options);
        return smoothed.Select(q => new JointState(q)).ToList();
    }

    internal static List<double[]> SmoothInternal(
        IReadOnlyList<double[]> path,
        IReadOnlyList<JointLimit> limits,
        CollisionScene scene,
        ICollisionChecker? checker,
        PlanningPipeline.ConstraintContext constraints,
        Func<double[], JointState> toFull,
        SamplingPlannerOptions options)
    {
        if (path.Count <= 2 || options.ChompIterations <= 0)
            return path.Select(q => (double[])q.Clone()).ToList();

        var current = path.Select(q => (double[])q.Clone()).ToList();
        var lr = Math.Clamp(options.ChompLearningRate, 1e-4, 0.45);
        var eps = Math.Clamp(options.ChompFiniteDifferenceStep, 1e-4, 0.2);

        for (var iter = 0; iter < options.ChompIterations; iter++)
        {
            for (var i = 1; i < current.Count - 1; i++)
            {
                var prev = current[i - 1];
                var q = current[i];
                var next = current[i + 1];
                var candidate = new double[q.Length];

                for (var j = 0; j < q.Length; j++)
                {
                    var smoothGradient = 2.0 * q[j] - prev[j] - next[j];
                    var collisionGradient = CollisionPenaltyGradient(q, j, eps, scene, checker, constraints, toFull);
                    candidate[j] = q[j] - lr * (smoothGradient + collisionGradient);
                    candidate[j] = Math.Clamp(candidate[j], limits[j].MinRadians, limits[j].MaxRadians);
                }

                if (ManagedRrtConnect.SegmentValid(prev, candidate, scene, checker, constraints, toFull, options.StepRadians) &&
                    ManagedRrtConnect.SegmentValid(candidate, next, scene, checker, constraints, toFull, options.StepRadians))
                {
                    current[i] = candidate;
                }
            }
        }

        return current;
    }

    private static double CollisionPenaltyGradient(
        double[] q,
        int axis,
        double eps,
        CollisionScene scene,
        ICollisionChecker? checker,
        PlanningPipeline.ConstraintContext constraints,
        Func<double[], JointState> toFull)
    {
        // PONYTAIL: finite-difference binary validity until a signed-distance collision backend is available.
        var plus = (double[])q.Clone();
        var minus = (double[])q.Clone();
        plus[axis] += eps;
        minus[axis] -= eps;
        var cp = Penalty(plus, scene, checker, constraints, toFull);
        var cm = Penalty(minus, scene, checker, constraints, toFull);
        return (cp - cm) / (2.0 * eps);
    }

    private static double Penalty(
        double[] q,
        CollisionScene scene,
        ICollisionChecker? checker,
        PlanningPipeline.ConstraintContext constraints,
        Func<double[], JointState> toFull)
    {
        var full = toFull(q);
        var penalty = 0.0;
        if (checker is not null && !checker.IsCollisionFree(full, scene))
            penalty += 1.0;
        if (!PlanningPipeline.TryValidateConstraints(constraints, full, out _))
            penalty += 1.0;
        return penalty;
    }
}
