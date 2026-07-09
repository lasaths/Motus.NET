using System.Runtime.InteropServices;
using Motus.Core;
using Motus.Geometry;
using Motus.Native;
using Motus.OMPL.Native;

namespace Motus.OMPL.NET;

/// <summary>Joint-space RRT-Connect via native OMPL when built; managed fallback otherwise.</summary>
public sealed class RrtConnectPlanner : IPlanner
{
    private readonly ICollisionChecker? _defaultChecker;
    private readonly RrtConnectOptions _options;

    public RrtConnectPlanner(RobotPreset preset, RrtConnectOptions? options = null)
        : this(preset, null, options) { }

    public RrtConnectPlanner(RobotPreset preset, SerialJointChain? serialChain, RrtConnectOptions? options = null)
    {
        _defaultChecker = KinematicsResolver.SupportsModel(preset, serialChain)
            ? serialChain is null
                ? new SphereCollisionChecker(preset)
                : new SphereCollisionChecker(preset, serialChain)
            : null;
        _options = options ?? new RrtConnectOptions();
    }

    public RrtConnectPlanner(ICollisionChecker checker, RrtConnectOptions? options = null)
    {
        _defaultChecker = checker;
        _options = options ?? new RrtConnectOptions();
    }

    public PlanningResult Plan(PlanningRequest request)
    {
        if (_options.StepRadians <= 0)
            return PlanningResult.Failed(new[] { "RrtConnectOptions.StepRadians must be positive." });

        if (NativeOmpl.IsAvailable)
        {
            var native = TryNativePlan(request);
            if (native is not null) return native;
        }

        return PlanManaged(request);
    }

    private static ICollisionChecker? ResolveChecker(PlanningRequest request, ICollisionChecker? defaultChecker)
        => request.Options.CollisionChecker
            ?? (request.Options.AttachedBodies is { Count: > 0 }
                ? CollisionCheckerFactory.Create(request.Robot, null, request.Options.AttachedBodies)
                : defaultChecker);

    private static PlanSpace BuildPlanSpace(PlanningRequest request)
    {
        var limits = request.Robot.Preset.JointLimits;
        var map = request.Options.GroupMap;
        if (map is null)
        {
            return new PlanSpace(
                request.Start,
                request.Start.Positions.ToArray(),
                request.Goal.Positions.ToArray(),
                limits,
                q => new JointState(q));
        }

        var groupLimits = map.GroupToFull.Select(i => limits[i]).ToList();
        return new PlanSpace(
            request.Start,
            map.ExtractGroupPositions(request.Start),
            map.ExtractGroupPositions(request.Goal),
            groupLimits,
            q => map.EmbedGroupState(request.Start, q));
    }

    private PlanningResult? TryNativePlan(PlanningRequest request)
    {
        var checker = ResolveChecker(request, _defaultChecker);
        var robot = request.Robot;
        var scene = request.Options.CollisionScene ?? new CollisionScene();
        var space = BuildPlanSpace(request);
        var n = space.Dims;
        var low = space.Limits.Select(l => l.MinRadians).ToArray();
        var high = space.Limits.Select(l => l.MaxRadians).ToArray();
        var maxStates = Math.Max(16, _options.MaxPathStates);
        var buffer = new double[n * maxStates];

        var ctx = new NativePlanContext(checker, scene, _options.StepRadians, space.ToFull);
        var handle = GCHandle.Alloc(ctx);
        try
        {
            NativeBindings.ValidityCallback stateCb = (statePtr, dims, user) =>
            {
                var context = (NativePlanContext)GCHandle.FromIntPtr(user).Target!;
                return context.ValidityCallback(statePtr, dims);
            };
            NativeBindings.MotionValidityCallback motionCb = (fromPtr, toPtr, dims, user) =>
            {
                var context = (NativePlanContext)GCHandle.FromIntPtr(user).Target!;
                return context.MotionValidityCallback(fromPtr, toPtr, dims);
            };

            var plannerId = _options.PlannerId switch
            {
                OmplPlannerId.RrtStar => NativeBindings.PlannerRrtStar,
                _ => NativeBindings.PlannerRrtConnect
            };

            var rc = NativeOmpl.motus_ompl_rrt_connect(
                n, low, high, space.Start, space.Goal,
                _options.MaxIterations, _options.MaxPlanTimeSeconds, _options.StepRadians, _options.GoalBias,
                plannerId,
                stateCb, motionCb, GCHandle.ToIntPtr(handle),
                buffer, maxStates, out var count);

            if (rc != NativeOmpl.Ok || count < 2) return null;

            var waypoints = new List<JointState>(count);
            for (var i = 0; i < count; i++)
            {
                var q = new double[n];
                Array.Copy(buffer, i * n, q, 0, n);
                waypoints.Add(space.ToFull(q));
            }

            var simplified = SimplifyNativePath(
                waypoints, n, space, request, robot, checker, scene,
                stateCb, motionCb, handle, maxStates);
            return BuildTrajectory(robot, simplified, request.Options, checker, usedNative: true);
        }
        finally
        {
            handle.Free();
        }
    }

    private IReadOnlyList<JointState> SimplifyNativePath(
        List<JointState> waypoints,
        int dims,
        PlanSpace space,
        PlanningRequest request,
        RobotModel robot,
        ICollisionChecker? checker,
        CollisionScene scene,
        NativeBindings.ValidityCallback stateCb,
        NativeBindings.MotionValidityCallback motionCb,
        GCHandle handle,
        int maxStates)
    {
        var pathCount = waypoints.Count;
        var flatPath = new double[pathCount * dims];
        for (var i = 0; i < pathCount; i++)
        {
            var groupQ = request.Options.GroupMap?.ExtractGroupPositions(waypoints[i])
                ?? waypoints[i].Positions.ToArray();
            Array.Copy(groupQ, 0, flatPath, i * dims, dims);
        }

        var simpBuf = new double[dims * maxStates];
        var simpRc = NativeOmpl.motus_ompl_simplify_path(
            dims, flatPath, pathCount, _options.StepRadians * 0.5,
            stateCb, motionCb, GCHandle.ToIntPtr(handle),
            simpBuf, maxStates, out var simpCount);

        if (simpRc != NativeOmpl.Ok || simpCount < 2)
            return PathSimplifier.Simplify(waypoints, robot, checker, scene, _options.StepRadians * 0.5);

        var simplified = new List<JointState>(simpCount);
        for (var i = 0; i < simpCount; i++)
        {
            var q = new double[dims];
            Array.Copy(simpBuf, i * dims, q, 0, dims);
            simplified.Add(space.ToFull(q));
        }
        return simplified;
    }

    private PlanningResult PlanManaged(PlanningRequest request)
    {
        var checker = ResolveChecker(request, _defaultChecker);
        var robot = request.Robot;
        var limits = robot.Preset.JointLimits;
        var scene = request.Options.CollisionScene ?? new CollisionScene();
        var rng = new Random(_options.RandomSeed);
        var space = BuildPlanSpace(request);

        var startVal = request.Start.Validate(limits);
        var goalVal = request.Goal.Validate(limits);
        if (!startVal.IsValid) return PlanningResult.Failed(startVal.Errors.Select(e => $"Start: {e}"));
        if (!goalVal.IsValid) return PlanningResult.Failed(goalVal.Errors.Select(e => $"Goal: {e}"));
        if (checker is not null)
        {
            if (!checker.IsCollisionFree(space.ToFull(space.Start), scene))
                return PlanningResult.Failed(new[] { "Start configuration is in collision." });
            if (!checker.IsCollisionFree(space.ToFull(space.Goal), scene))
                return PlanningResult.Failed(new[] { "Goal configuration is in collision." });
        }

        var start = (double[])space.Start.Clone();
        var goal = (double[])space.Goal.Clone();
        var treeA = new RrtTree { Nodes = [new RrtNode(start, -1)] };
        var treeB = new RrtTree { Nodes = [new RrtNode(goal, -1)] };

        for (var iter = 0; iter < _options.MaxIterations; iter++)
        {
            if (_options.ShouldCancel?.Invoke() == true)
                return PlanningResult.Failed(new[] { "Planning cancelled." });

            var sample = Sample(rng, goal, space.Limits);
            var (extendedA, newIdxA) = Extend(treeA, sample, space.Limits, scene, checker, space.ToFull);
            if (extendedA && Connect(treeB, treeA.Nodes[newIdxA].Q, space.Limits, scene, checker, space.ToFull, out var connectIdxB))
            {
                var pathA = Reconstruct(treeA, newIdxA);
                var pathB = Reconstruct(treeB, connectIdxB);
                pathB.Reverse();
                var raw = pathA.Concat(pathB.Skip(1)).Select(q => space.ToFull(q)).ToList();
                var simplified = PathSimplifier.Simplify(raw, robot, checker, scene, _options.StepRadians * 0.5);
                return BuildTrajectory(robot, simplified, request.Options, checker, usedNative: false);
            }

            (treeA, treeB) = (treeB, treeA);
        }

        return PlanningResult.Failed(new[] { $"RRT-Connect failed after {_options.MaxIterations} iterations." });
    }

    private double[] Sample(Random rng, double[] goal, IReadOnlyList<JointLimit> limits)
    {
        if (rng.NextDouble() < _options.GoalBias) return (double[])goal.Clone();
        var q = new double[limits.Count];
        for (var i = 0; i < limits.Count; i++)
            q[i] = limits[i].MinRadians + rng.NextDouble() * (limits[i].MaxRadians - limits[i].MinRadians);
        return q;
    }

    private (bool extended, int newIndex) Extend(
        RrtTree tree, double[] target, IReadOnlyList<JointLimit> limits, CollisionScene scene,
        ICollisionChecker? checker, Func<double[], JointState> toFull)
    {
        var nearest = Nearest(tree, target);
        var steered = Steer(tree.Nodes[nearest].Q, target, limits);
        if (ConfigurationDistance(tree.Nodes[nearest].Q, steered) < 1e-12) return (false, nearest);
        if (!SegmentFree(tree.Nodes[nearest].Q, steered, scene, checker, toFull)) return (false, nearest);
        tree.Nodes.Add(new RrtNode(steered, nearest));
        return (true, tree.Nodes.Count - 1);
    }

    private bool Connect(
        RrtTree tree, double[] target, IReadOnlyList<JointLimit> limits, CollisionScene scene,
        ICollisionChecker? checker, Func<double[], JointState> toFull, out int connectIndex)
    {
        connectIndex = Nearest(tree, target);
        while (ConfigurationDistance(tree.Nodes[connectIndex].Q, target) > _options.ConnectThresholdRadians)
        {
            var steered = Steer(tree.Nodes[connectIndex].Q, target, limits);
            if (ConfigurationDistance(tree.Nodes[connectIndex].Q, steered) < 1e-12) break;
            if (!SegmentFree(tree.Nodes[connectIndex].Q, steered, scene, checker, toFull)) return false;
            tree.Nodes.Add(new RrtNode(steered, connectIndex));
            connectIndex = tree.Nodes.Count - 1;
        }
        return ConfigurationDistance(tree.Nodes[connectIndex].Q, target) <= _options.ConnectThresholdRadians;
    }

    private bool SegmentFree(
        double[] from, double[] to, CollisionScene scene, ICollisionChecker? checker, Func<double[], JointState> toFull)
    {
        if (checker is null) return true;
        return checker.SegmentCollisionFree(toFull(from), toFull(to), scene, _options.StepRadians);
    }

    private static int Nearest(RrtTree tree, double[] target)
    {
        var best = 0;
        var bestDist = double.MaxValue;
        for (var i = 0; i < tree.Nodes.Count; i++)
        {
            var d = ConfigurationDistance(tree.Nodes[i].Q, target);
            if (d < bestDist) { bestDist = d; best = i; }
        }
        return best;
    }

    private double[] Steer(double[] from, double[] to, IReadOnlyList<JointLimit> limits)
    {
        var dist = ConfigurationDistance(from, to);
        if (dist <= _options.StepRadians) return Clamp((double[])to.Clone(), limits);
        var alpha = _options.StepRadians / dist;
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

    private static double ConfigurationDistance(double[] a, double[] b)
    {
        var sum = 0.0;
        for (var i = 0; i < a.Length; i++)
        {
            var d = a[i] - b[i];
            sum += d * d;
        }
        return Math.Sqrt(sum);
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

    private static PlanningResult BuildTrajectory(
        RobotModel robot, IReadOnlyList<JointState> waypoints, PlanningOptions opts,
        ICollisionChecker? checker, bool usedNative)
    {
        var segmentOpts = checker is not null && PlanningCollision.SceneHasObstacles(opts.CollisionScene)
            ? new PlanningOptions
            {
                MaxJointStepRadians = opts.MaxJointStepRadians,
                TimeStepSeconds = opts.TimeStepSeconds,
                MaxJointVelocityRadiansPerSecond = opts.MaxJointVelocityRadiansPerSecond,
                CollisionScene = opts.CollisionScene,
                CollisionChecker = checker
            }
            : opts;
        var planner = new JointLinearPlanner();
        var points = new List<TrajectoryPoint> { new(0, waypoints[0]) };
        var t = 0.0;
        for (var i = 1; i < waypoints.Count; i++)
        {
            var seg = planner.Plan(new PlanningRequest(robot, waypoints[i - 1], waypoints[i], segmentOpts));
            if (!seg.Success) return PlanningResult.Failed(seg.Errors);
            var segPts = seg.Trajectory!.Points;
            for (var j = 1; j < segPts.Count; j++)
            {
                t += segPts[j].TimeSeconds - segPts[j - 1].TimeSeconds;
                points.Add(new TrajectoryPoint(t, segPts[j].JointState));
            }
        }

        var warnings = new List<string> { "RrtConnectPlanner: joint-space RRT-Connect path." };
        warnings.Add(MotusCapabilities.Describe());
        if (!usedNative)
            warnings.Add(NativeOmpl.StatusMessage);
        if (checker is null)
            warnings.Add("RRT-Connect ran without collision checker (no kinematics chain).");
        return PlanningResult.Succeeded(new Trajectory(robot, points), warnings);
    }

    private sealed class RrtTree
    {
        public List<RrtNode> Nodes { get; init; } = new();
    }

    private readonly record struct RrtNode(double[] Q, int Parent);

    private readonly record struct PlanSpace(
        JointState Seed,
        double[] Start,
        double[] Goal,
        IReadOnlyList<JointLimit> Limits,
        Func<double[], JointState> ToFull)
    {
        public int Dims => Limits.Count;
    }
}
