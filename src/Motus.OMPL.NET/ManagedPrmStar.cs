using Motus.Core;
using Motus.Geometry;

namespace Motus.OMPL.NET;

internal static class ManagedPrmStar
{
    internal static PlanningResult Plan(
        PlanningRequest request,
        SamplingPlannerOptions options,
        ICollisionChecker? defaultChecker,
        SerialJointChain? serialChain)
    {
        var checker = PlanningPipeline.ResolveChecker(request, defaultChecker);
        var robot = request.Robot;
        var scene = request.Options.CollisionScene ?? new CollisionScene();
        var rng = new Random(options.RandomSeed);
        var spaceFail = PlanningPipeline.TryBuildPlanSpace(request, out var space);
        if (spaceFail is not null) return spaceFail;
        var checkerAvailabilityFail = PlanningPipeline.ValidateCollisionCheckerAvailability(request.Options, scene, checker);
        if (checkerAvailabilityFail is not null) return checkerAvailabilityFail;
        var checkerFail = PlanningPipeline.ValidateMobileBaseChecker(space, checker);
        if (checkerFail is not null) return checkerFail;

        var startVal = request.Start.Validate(robot.Preset.JointLimits);
        var goalVal = request.Goal.Validate(robot.Preset.JointLimits);
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

        var nodes = new List<double[]> { (double[])space.Start.Clone(), (double[])space.Goal.Clone() };
        var edges = new List<Edge>[Math.Max(16, Math.Max(options.MaxPathStates, 64))];
        for (var i = 0; i < edges.Length; i++) edges[i] = new List<Edge>();

        TryConnectNode(0, nodes, edges, space, scene, checker, constraints, options, goalOnly: true);
        TryConnectNode(1, nodes, edges, space, scene, checker, constraints, options, goalOnly: false);
        var best = Dijkstra(nodes, edges, 0, 1);
        if (best.Count > 0)
            return BuildResult(best, nodes, space, robot, checker, scene, request.Options, options, constraints);

        var started = Environment.TickCount64;
        var maxNodes = Math.Min(edges.Length, Math.Max(16, options.MaxIterations + 2));
        for (var iter = 0; iter < options.MaxIterations && nodes.Count < maxNodes; iter++)
        {
            if (options.ShouldCancel?.Invoke() == true)
                return PlanningResult.Failed(new[] { "Planning cancelled." });
            if (options.MaxPlanTimeSeconds > 0 &&
                (Environment.TickCount64 - started) / 1000.0 >= options.MaxPlanTimeSeconds)
                break;

            if ((iter & 0xF) == 0)
                options.ReportIteration?.Invoke(iter, options.MaxIterations);

            var sample = Sample(rng, space.Limits);
            if (!StateValid(sample, scene, checker, constraints, space))
                continue;

            nodes.Add(sample);
            TryConnectNode(nodes.Count - 1, nodes, edges, space, scene, checker, constraints, options, goalOnly: false);

            if ((iter % 25) == 0 || nodes.Count == maxNodes)
            {
                best = Dijkstra(nodes, edges, 0, 1);
                if (best.Count > 0)
                {
                    options.ReportIteration?.Invoke(options.MaxIterations - 1, options.MaxIterations);
                    return BuildResult(best, nodes, space, robot, checker, scene, request.Options, options, constraints);
                }
            }
        }

        best = Dijkstra(nodes, edges, 0, 1);
        if (best.Count > 0)
            return BuildResult(best, nodes, space, robot, checker, scene, request.Options, options, constraints);

        return PlanningResult.Failed(new[] { $"PRM* failed after building {nodes.Count} valid roadmap states." });
    }

    private static PlanningResult BuildResult(
        IReadOnlyList<int> nodePath,
        IReadOnlyList<double[]> nodes,
        PlanningPipeline.PlanSpace space,
        RobotModel robot,
        ICollisionChecker? checker,
        CollisionScene scene,
        PlanningOptions planningOptions,
        SamplingPlannerOptions options,
        PlanningPipeline.ConstraintContext constraints)
    {
        var groupPath = nodePath.Select(i => nodes[i]).ToList();
        var smoothed = ChompLiteSmoother.SmoothInternal(
            groupPath,
            space.Limits,
            scene,
            checker,
            constraints,
            space,
            options);
        var full = smoothed
            .Select(q => new JointState(space.ToFull(q).Positions.ToArray()))
            .ToList();
        if (space.HasMobility)
            return PlanningPipeline.BuildTrajectoryFromPlanSpace(
                robot, smoothed, space, planningOptions, checker, usedNative: false, "PRM*");
        return PlanningPipeline.BuildTrajectory(robot, full, planningOptions, checker, usedNative: false, "PRM*");
    }

    private static void TryConnectNode(
        int index,
        IReadOnlyList<double[]> nodes,
        List<Edge>[] edges,
        PlanningPipeline.PlanSpace space,
        CollisionScene scene,
        ICollisionChecker? checker,
        PlanningPipeline.ConstraintContext constraints,
        SamplingPlannerOptions options,
        bool goalOnly)
    {
        var radius = ConnectionRadius(nodes.Count, space.Dims, options);
        for (var other = 0; other < nodes.Count; other++)
        {
            if (other == index) continue;
            if (goalOnly && other != 1) continue;
            var distance = Distance(nodes[index], nodes[other]);
            if (distance > radius && !(index <= 1 || other <= 1)) continue;
            if (!ManagedRrtConnect.SegmentValid(nodes[index], nodes[other], scene, checker, constraints, space, options.StepRadians))
                continue;

            edges[index].Add(new Edge(other, distance));
            edges[other].Add(new Edge(index, distance));
        }
    }

    private static double ConnectionRadius(int n, int dims, SamplingPlannerOptions options)
    {
        var samples = Math.Max(2, n);
        var radius = options.PrmStarGamma * Math.Pow(Math.Log(samples) / samples, 1.0 / Math.Max(1, dims));
        return Math.Max(options.ConnectThresholdRadians, radius);
    }

    private static bool StateValid(
        double[] q,
        CollisionScene scene,
        ICollisionChecker? checker,
        PlanningPipeline.ConstraintContext constraints,
        PlanningPipeline.PlanSpace space)
    {
        var full = space.ToFull(q);
        var baseFrame = space.ToBaseFrame(q);
        if (!PlanningPipeline.StateCollisionFree(checker, full, scene, baseFrame))
            return false;
        return PlanningPipeline.TryValidateConstraints(constraints, full, baseFrame, out _);
    }

    private static double[] Sample(Random rng, IReadOnlyList<JointLimit> limits)
    {
        var q = new double[limits.Count];
        for (var i = 0; i < limits.Count; i++)
            q[i] = limits[i].MinRadians + rng.NextDouble() * (limits[i].MaxRadians - limits[i].MinRadians);
        return q;
    }

    private static List<int> Dijkstra(IReadOnlyList<double[]> nodes, IReadOnlyList<List<Edge>> edges, int start, int goal)
    {
        var dist = Enumerable.Repeat(double.PositiveInfinity, nodes.Count).ToArray();
        var prev = Enumerable.Repeat(-1, nodes.Count).ToArray();
        var used = new bool[nodes.Count];
        dist[start] = 0;

        for (var k = 0; k < nodes.Count; k++)
        {
            var u = -1;
            var best = double.PositiveInfinity;
            for (var i = 0; i < nodes.Count; i++)
            {
                if (!used[i] && dist[i] < best)
                {
                    best = dist[i];
                    u = i;
                }
            }

            if (u < 0 || u == goal) break;
            used[u] = true;
            foreach (var edge in edges[u])
            {
                var alt = dist[u] + edge.Cost;
                if (alt < dist[edge.To])
                {
                    dist[edge.To] = alt;
                    prev[edge.To] = u;
                }
            }
        }

        if (double.IsPositiveInfinity(dist[goal]))
            return new List<int>();

        var path = new List<int>();
        for (var at = goal; at >= 0; at = prev[at])
        {
            path.Add(at);
            if (at == start) break;
        }
        path.Reverse();
        return path.Count > 0 && path[0] == start ? path : new List<int>();
    }

    private static double Distance(IReadOnlyList<double> a, IReadOnlyList<double> b)
    {
        var sum = 0.0;
        for (var i = 0; i < a.Count; i++)
        {
            var d = a[i] - b[i];
            sum += d * d;
        }
        return Math.Sqrt(sum);
    }

    private readonly record struct Edge(int To, double Cost);
}
