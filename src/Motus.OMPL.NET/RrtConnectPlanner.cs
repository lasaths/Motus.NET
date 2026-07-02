using System.Runtime.InteropServices;
using Motus.Core;
using Motus.Geometry;
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

    public RrtConnectPlanner(SphereCollisionChecker collision, RrtConnectOptions? options = null)
        : this((ICollisionChecker)collision, options) { }

    public PlanningResult Plan(PlanningRequest request)
    {
        if (NativeOmpl.IsAvailable)
        {
            var native = TryNativePlan(request);
            if (native is not null) return native;
        }

        return PlanManaged(request);
    }

    private static ICollisionChecker? ResolveChecker(PlanningRequest request, ICollisionChecker? defaultChecker) =>
        request.Options.CollisionChecker ?? defaultChecker;

    private PlanningResult? TryNativePlan(PlanningRequest request)
    {
        var checker = ResolveChecker(request, _defaultChecker);
        var robot = request.Robot;
        var limits = robot.Preset.JointLimits;
        var scene = request.Options.CollisionScene ?? new CollisionScene();
        var n = limits.Count;
        var low = limits.Select(l => l.MinRadians).ToArray();
        var high = limits.Select(l => l.MaxRadians).ToArray();
        var start = request.Start.Positions.ToArray();
        var goal = request.Goal.Positions.ToArray();
        var maxStates = Math.Max(16, _options.MaxPathStates);
        var buffer = new double[n * maxStates];

        var ctx = new NativePlanContext(checker, scene);
        var handle = GCHandle.Alloc(ctx);
        try
        {
            NativeOmpl.ValidityCallback cb = (statePtr, dims, user) =>
            {
                var context = (NativePlanContext)GCHandle.FromIntPtr(user).Target!;
                return context.ValidityCallback(statePtr, dims);
            };

            var rc = NativeOmpl.motus_ompl_rrt_connect(
                n, low, high, start, goal,
                _options.MaxIterations, _options.StepRadians, _options.GoalBias,
                cb, GCHandle.ToIntPtr(handle),
                buffer, maxStates, out var count);

            if (rc != NativeOmpl.Ok || count < 2) return null;

            var waypoints = new List<JointState>(count);
            for (var i = 0; i < count; i++)
            {
                var q = new double[n];
                Array.Copy(buffer, i * n, q, 0, n);
                waypoints.Add(new JointState(q));
            }

            var simplified = PathSimplifier.Simplify(waypoints, robot, checker, scene, _options.StepRadians * 0.5);
            return BuildTrajectory(robot, simplified, request.Options, checker, usedNative: true);
        }
        finally
        {
            handle.Free();
        }
    }

    private PlanningResult PlanManaged(PlanningRequest request)
    {
        var checker = ResolveChecker(request, _defaultChecker);
        var robot = request.Robot;
        var limits = robot.Preset.JointLimits;
        var scene = request.Options.CollisionScene ?? new CollisionScene();
        var rng = new Random(_options.RandomSeed);

        var startVal = request.Start.Validate(limits);
        var goalVal = request.Goal.Validate(limits);
        if (!startVal.IsValid) return PlanningResult.Failed(startVal.Errors.Select(e => $"Start: {e}"));
        if (!goalVal.IsValid) return PlanningResult.Failed(goalVal.Errors.Select(e => $"Goal: {e}"));
        if (checker is not null)
        {
            if (!checker.IsCollisionFree(request.Start, scene))
                return PlanningResult.Failed(new[] { "Start configuration is in collision." });
            if (!checker.IsCollisionFree(request.Goal, scene))
                return PlanningResult.Failed(new[] { "Goal configuration is in collision." });
        }

        var start = request.Start.Positions.ToArray();
        var goal = request.Goal.Positions.ToArray();
        var treeA = new RrtTree { Nodes = [new RrtNode(start, -1)] };
        var treeB = new RrtTree { Nodes = [new RrtNode(goal, -1)] };

        for (var iter = 0; iter < _options.MaxIterations; iter++)
        {
            if (_options.ShouldCancel?.Invoke() == true)
                return PlanningResult.Failed(new[] { "Planning cancelled." });

            var sample = Sample(rng, goal, limits);
            var (extendedA, newIdxA) = Extend(treeA, sample, limits, scene, checker);
            if (extendedA && Connect(treeB, treeA.Nodes[newIdxA].Q, limits, scene, checker, out var connectIdxB))
            {
                var pathA = Reconstruct(treeA, newIdxA);
                var pathB = Reconstruct(treeB, connectIdxB);
                pathB.Reverse();
                var raw = pathA.Concat(pathB.Skip(1)).Select(q => new JointState(q)).ToList();
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
        RrtTree tree, double[] target, IReadOnlyList<JointLimit> limits, CollisionScene scene, ICollisionChecker? checker)
    {
        var nearest = Nearest(tree, target);
        var steered = Steer(tree.Nodes[nearest].Q, target, limits);
        if (ConfigurationDistance(tree.Nodes[nearest].Q, steered) < 1e-12) return (false, nearest);
        if (!SegmentFree(tree.Nodes[nearest].Q, steered, scene, checker)) return (false, nearest);
        tree.Nodes.Add(new RrtNode(steered, nearest));
        return (true, tree.Nodes.Count - 1);
    }

    private bool Connect(
        RrtTree tree, double[] target, IReadOnlyList<JointLimit> limits, CollisionScene scene,
        ICollisionChecker? checker, out int connectIndex)
    {
        connectIndex = Nearest(tree, target);
        while (ConfigurationDistance(tree.Nodes[connectIndex].Q, target) > _options.ConnectThresholdRadians)
        {
            var steered = Steer(tree.Nodes[connectIndex].Q, target, limits);
            if (ConfigurationDistance(tree.Nodes[connectIndex].Q, steered) < 1e-12) break;
            if (!SegmentFree(tree.Nodes[connectIndex].Q, steered, scene, checker)) return false;
            tree.Nodes.Add(new RrtNode(steered, connectIndex));
            connectIndex = tree.Nodes.Count - 1;
        }
        return ConfigurationDistance(tree.Nodes[connectIndex].Q, target) <= _options.ConnectThresholdRadians;
    }

    private bool SegmentFree(double[] from, double[] to, CollisionScene scene, ICollisionChecker? checker)
    {
        if (checker is null) return true;
        return checker.SegmentCollisionFree(new JointState(from), new JointState(to), scene, _options.StepRadians);
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
}
