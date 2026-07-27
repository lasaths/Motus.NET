using Motus.Core;
using Motus.Geometry;

namespace Motus.OMPL.NET;

/// <summary>Managed RRT-Connect fallback (extracted from legacy RrtConnectPlanner).</summary>
internal static class ManagedRrtConnect
{
    internal static PlanningResult Plan(
        PlanningRequest request,
        SamplingPlannerOptions options,
        ICollisionChecker? defaultChecker,
        SerialJointChain? serialChain = null,
        string plannerLabel = "RRT-Connect")
    {
        var checker = PlanningPipeline.ResolveChecker(request, defaultChecker);
        var robot = request.Robot;
        var limits = robot.Preset.JointLimits;
        var scene = request.Options.CollisionScene ?? new CollisionScene();
        var rng = new Random(options.RandomSeed);
        var spaceFail = PlanningPipeline.TryBuildPlanSpace(request, out var space);
        if (spaceFail is not null) return spaceFail;
        var checkerAvailabilityFail = PlanningPipeline.ValidateCollisionCheckerAvailability(request.Options, scene, checker);
        if (checkerAvailabilityFail is not null) return checkerAvailabilityFail;
        var checkerFail = PlanningPipeline.ValidateMobileBaseChecker(space, checker);
        if (checkerFail is not null) return checkerFail;

        var startVal = request.Start.Validate(limits);
        var goalVal = request.Goal.Validate(limits);
        if (!startVal.IsValid) return PlanningResult.Failed(startVal.Errors.Select(e => $"Start: {e}"));
        if (!goalVal.IsValid) return PlanningResult.Failed(goalVal.Errors.Select(e => $"Goal: {e}"));
        var constraintFail = PlanningPipeline.TryBuildConstraintContext(request, serialChain, out var constraints);
        if (constraintFail is not null) return constraintFail;
        if (!PlanningPipeline.TryValidateConstraints(
                constraints, space.ToFull(space.Start), space.ToBaseFrame(space.Start), out var startReason))
            return PlanningPipeline.ConstraintFailure("start", startReason);
        if (!PlanningPipeline.TryValidateConstraints(
                constraints, space.ToFull(space.Goal), space.ToBaseFrame(space.Goal), out var goalReason))
            return PlanningPipeline.ConstraintFailure("goal", goalReason);
        var endpointFail = PlanningPipeline.ValidateEndpoints(
            space, scene, checker, request.Options.AttachedBodies is { Count: > 0 });
        if (endpointFail is not null)
            return endpointFail;

        var start = (double[])space.Start.Clone();
        var goal = (double[])space.Goal.Clone();
        var treeA = new RrtTree { Nodes = [new RrtNode(start, -1)] };
        var treeB = new RrtTree { Nodes = [new RrtNode(goal, -1)] };
        var started = Environment.TickCount64;

        for (var iter = 0; iter < options.MaxIterations; iter++)
        {
            if (options.ShouldCancel?.Invoke() == true)
                return PlanningResult.Failed(new[] { "Planning cancelled." });

            if (options.MaxPlanTimeSeconds > 0)
            {
                var elapsed = (Environment.TickCount64 - started) / 1000.0;
                if (elapsed >= options.MaxPlanTimeSeconds)
                {
                    return PlanningResult.Failed(new[]
                    {
                        $"Planning timed out after {options.MaxPlanTimeSeconds:F0}s. " +
                        "Increase TimeLimit on Motus RRT Settings or reduce MaxIter."
                    });
                }
            }

            if ((iter & 0xF) == 0)
                options.ReportIteration?.Invoke(iter, options.MaxIterations);

            var sample = Sample(rng, goal, space.Limits, options.GoalBias);
            var (extendedA, newIdxA) = Extend(treeA, sample, space.Limits, scene, checker, constraints, space, options);
            if (extendedA && Connect(treeB, treeA.Nodes[newIdxA].Q, space.Limits, scene, checker, constraints, space, options, out var connectIdxB))
            {
                var pathA = Reconstruct(treeA, newIdxA);
                var pathB = Reconstruct(treeB, connectIdxB);
                pathB.Reverse();
                // Copy each ToFull result — group embedding may reuse a scratch JointState.
                var rawPlan = pathA.Concat(pathB.Skip(1))
                    .Select(q => (double[])q.Clone())
                    .ToList();
                EnsurePathStartsAtStart(rawPlan, start, goal);
                options.ReportIteration?.Invoke(options.MaxIterations - 1, options.MaxIterations);
                if (space.HasMobility)
                    return PlanningPipeline.BuildTrajectoryFromPlanSpace(
                        robot, rawPlan, space, request.Options, checker, usedNative: false, plannerLabel);

                var raw = rawPlan
                    .Select(q => new JointState(space.ToFull(q).Positions.ToArray()))
                    .ToList();
                var simplified = PathSimplifier.Simplify(raw, robot, checker, scene, options.StepRadians * 0.5);
                return PlanningPipeline.BuildTrajectory(robot, simplified, request.Options, checker, usedNative: false, plannerLabel);
            }

            (treeA, treeB) = (treeB, treeA);
        }

        return PlanningResult.Failed(new[] { $"RRT-Connect failed after {options.MaxIterations} iterations." });
    }

    private static double[] Sample(Random rng, double[] goal, IReadOnlyList<JointLimit> limits, double goalBias)
    {
        if (rng.NextDouble() < goalBias) return (double[])goal.Clone();
        var q = new double[limits.Count];
        for (var i = 0; i < limits.Count; i++)
            q[i] = limits[i].MinRadians + rng.NextDouble() * (limits[i].MaxRadians - limits[i].MinRadians);
        return q;
    }

    private static (bool extended, int newIndex) Extend(
        RrtTree tree, double[] target, IReadOnlyList<JointLimit> limits, CollisionScene scene,
        ICollisionChecker? checker, PlanningPipeline.ConstraintContext constraints,
        PlanningPipeline.PlanSpace space, SamplingPlannerOptions options)
    {
        var nearest = Nearest(tree, target);
        var steered = Steer(tree.Nodes[nearest].Q, target, limits, options.StepRadians);
        if (ConfigurationDistanceSquared(tree.Nodes[nearest].Q, steered) < 1e-24) return (false, nearest);
        if (!SegmentValid(tree.Nodes[nearest].Q, steered, scene, checker, constraints, space, options.StepRadians)) return (false, nearest);
        tree.Nodes.Add(new RrtNode(steered, nearest));
        return (true, tree.Nodes.Count - 1);
    }

    private static bool Connect(
        RrtTree tree, double[] target, IReadOnlyList<JointLimit> limits, CollisionScene scene,
        ICollisionChecker? checker, PlanningPipeline.ConstraintContext constraints,
        PlanningPipeline.PlanSpace space, SamplingPlannerOptions options, out int connectIndex)
    {
        connectIndex = Nearest(tree, target);
        var thresholdSq = options.ConnectThresholdRadians * options.ConnectThresholdRadians;
        while (ConfigurationDistanceSquared(tree.Nodes[connectIndex].Q, target) > thresholdSq)
        {
            var steered = Steer(tree.Nodes[connectIndex].Q, target, limits, options.StepRadians);
            if (ConfigurationDistanceSquared(tree.Nodes[connectIndex].Q, steered) < 1e-24) break;
            if (!SegmentValid(tree.Nodes[connectIndex].Q, steered, scene, checker, constraints, space, options.StepRadians)) return false;
            tree.Nodes.Add(new RrtNode(steered, connectIndex));
            connectIndex = tree.Nodes.Count - 1;
        }
        return ConfigurationDistanceSquared(tree.Nodes[connectIndex].Q, target) <= thresholdSq;
    }

    internal static bool SegmentValid(
        double[] from, double[] to, CollisionScene scene, ICollisionChecker? checker,
        PlanningPipeline.ConstraintContext constraints, PlanningPipeline.PlanSpace space, double stepRadians)
    {
        if (!constraints.Enabled && checker is null) return true;
        if (!space.HasMobility && !constraints.Enabled && checker is SphereCollisionChecker sphere)
            return sphere.SegmentCollisionFree(from, to, scene, stepRadians);

        if (stepRadians <= 0) stepRadians = 1e-3;
        var n = from.Length;
        var maxDelta = 0.0;
        for (var i = 0; i < n; i++)
            maxDelta = Math.Max(maxDelta, Math.Abs(to[i] - from[i]));
        var steps = Math.Max(1, (int)Math.Ceiling(maxDelta / stepRadians));

        for (var s = 0; s <= steps; s++)
        {
            var alpha = (double)s / steps;
            var q = new double[n];
            for (var i = 0; i < n; i++)
                q[i] = from[i] + alpha * (to[i] - from[i]);
            var full = space.ToFull(q);
            var baseFrame = space.ToBaseFrame(q);
            if (!PlanningPipeline.StateCollisionFree(checker, full, scene, baseFrame))
                return false;
            if (!PlanningPipeline.TryValidateConstraints(constraints, full, baseFrame, out _))
                return false;
        }

        return true;
    }

    private static bool SegmentFree(
        double[] from, double[] to, CollisionScene scene, ICollisionChecker? checker,
        Func<double[], JointState> toFull, double stepRadians)
    {
        if (checker is null) return true;
        if (checker is SphereCollisionChecker sphere)
            return sphere.SegmentCollisionFree(from, to, scene, stepRadians);
        // JointIndexMap.EmbedGroupState reuses one scratch JointState — materialize copies
        // before the second toFull call or both ends alias the same buffer.
        var fromFull = toFull(from);
        var fromCopy = new JointState(fromFull.Positions.ToArray());
        var toCopy = toFull(to);
        return checker.SegmentCollisionFree(fromCopy, toCopy, scene, stepRadians);
    }

    private static int Nearest(RrtTree tree, double[] target)
    {
        var best = 0;
        var bestDistSq = double.MaxValue;
        for (var i = 0; i < tree.Nodes.Count; i++)
        {
            var d = ConfigurationDistanceSquared(tree.Nodes[i].Q, target);
            if (d < bestDistSq) { bestDistSq = d; best = i; }
        }
        return best;
    }

    private static double[] Steer(double[] from, double[] to, IReadOnlyList<JointLimit> limits, double stepRadians)
    {
        var dist = ConfigurationDistance(from, to);
        if (dist <= stepRadians) return Clamp((double[])to.Clone(), limits);
        var alpha = stepRadians / dist;
        var q = new double[from.Length];
        for (var i = 0; i < from.Length; i++)
            q[i] = from[i] + alpha * (to[i] - from[i]);
        return Clamp(q, limits);
    }

    private static double[] Clamp(double[] q, IReadOnlyList<JointLimit> limits)
    {
        for (var i = 0; i < q.Length; i++)
            q[i] = Math.Clamp(q[i], limits[i].MinRadians, limits[i].MaxRadians);
        return q;
    }

    private static double ConfigurationDistanceSquared(double[] a, double[] b)
    {
        var sum = 0.0;
        for (var i = 0; i < a.Length; i++)
        {
            var d = a[i] - b[i];
            sum += d * d;
        }
        return sum;
    }

    private static double ConfigurationDistance(double[] a, double[] b) =>
        Math.Sqrt(ConfigurationDistanceSquared(a, b));

    private static void EnsurePathStartsAtStart(List<double[]> path, double[] start, double[] goal)
    {
        if (path.Count < 2) return;
        if (ConfigurationDistanceSquared(path[0], start)
            > ConfigurationDistanceSquared(path[0], goal))
            path.Reverse();
    }

    private static List<double[]> Reconstruct(RrtTree tree, int index)
    {
        var path = new List<double[]>();
        var i = index;
        while (i >= 0)
        {
            path.Add(tree.Nodes[i].Q);
            i = tree.Nodes[i].Parent;
        }
        path.Reverse();
        return path;
    }

    private sealed class RrtTree
    {
        public List<RrtNode> Nodes { get; init; } = new();
    }

    private readonly record struct RrtNode(double[] Q, int Parent);
}
