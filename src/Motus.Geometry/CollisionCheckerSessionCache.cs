using Motus.Core;

namespace Motus.Geometry;

/// <summary>Reuses collision checkers for identical robot / attach / scene fingerprints within a process.</summary>
public static class CollisionCheckerSessionCache
{
    // ponytail: unbounded fingerprints (e.g. one per attach/detach cycle under Auto Plan) would
    // otherwise pile up WeakReference entries the GC has no pressure to collect; cap + FIFO evict.
    private const int MaxEntries = 64;
    private static readonly Dictionary<Key, WeakReference<ICollisionChecker>> Cache = new();
    private static readonly List<Key> Order = new();
    private static readonly object Gate = new();

    public static ICollisionChecker GetOrCreate(
        RobotModel robot,
        SerialJointChain? chain,
        IReadOnlyList<AttachedBody>? attached,
        CollisionScene? scene) =>
        GetOrCreate(robot, chain, attached, scene, tree: null, planJointNames: null, treeDriverHome: null);

    public static ICollisionChecker GetOrCreate(
        RobotModel robot,
        SerialJointChain? chain,
        IReadOnlyList<AttachedBody>? attached,
        CollisionScene? scene,
        KinematicTree? tree,
        IReadOnlyList<string>? planJointNames,
        IReadOnlyList<double>? treeDriverHome)
    {
        var key = new Key(Fingerprint(robot, chain, attached, scene, tree, planJointNames, treeDriverHome),
            robot, chain, attached, tree, planJointNames, treeDriverHome);
        lock (Gate)
        {
            if (Cache.TryGetValue(key, out var weak) && weak.TryGetTarget(out var existing))
                return existing;

            var checker = tree is not null
                && robot.CollisionModel is { Links.Count: > 0 }
                && robot.Preset.AxisCount > (chain?.Joints.Length ?? 0)
                ? CollisionCheckerFactory.Create(robot, tree, chain, planJointNames, treeDriverHome, attached)
                : CollisionCheckerFactory.Create(robot, chain, attached);
            Cache[key] = new WeakReference<ICollisionChecker>(checker);
            Order.Add(key);
            PruneDead();
            EvictOverflow();
            return checker;
        }
    }

    public static void Clear() { lock (Gate) { Cache.Clear(); Order.Clear(); } }

    private static void PruneDead()
    {
        foreach (var key in Cache.Keys.ToList())
        {
            if (!Cache[key].TryGetTarget(out _))
            {
                Cache.Remove(key);
                Order.Remove(key);
            }
        }
    }

    // Bookkeeping-only eviction: the value is a WeakReference, so dropping the dictionary entry
    // never frees memory a caller still holds — it only bounds the (key, WeakReference) overhead
    // itself, which would otherwise grow with every distinct attach/scene fingerprint over a long
    // session (e.g. one per pick-place attach/detach cycle under Auto Plan). Never Dispose here:
    // the checker may still be in active use by whoever is holding the strong reference.
    private static void EvictOverflow()
    {
        while (Order.Count > MaxEntries)
        {
            var oldest = Order[0];
            Order.RemoveAt(0);
            Cache.Remove(oldest);
        }
    }

    internal static int Fingerprint(
        RobotModel robot,
        SerialJointChain? chain,
        IReadOnlyList<AttachedBody>? attached,
        CollisionScene? scene,
        KinematicTree? tree = null,
        IReadOnlyList<string>? planJointNames = null,
        IReadOnlyList<double>? treeDriverHome = null)
    {
        var hash = new HashCode();
        hash.Add(robot.Preset.ModelName, StringComparer.Ordinal);
        hash.Add(robot.Preset.AxisCount);
        hash.Add(chain?.GetHashCode() ?? 0);
        hash.Add(tree?.Fingerprint ?? 0);
        hash.Add(robot.Preset.BaseFrame.Frame);
        hash.Add(robot.Preset.ToolFrame.Frame);
        hash.Add(robot.CollisionModel?.ToolGeometry?.ContentHash ?? 0);
        if (treeDriverHome is not null)
            foreach (var q in treeDriverHome)
                hash.Add(q);
        if (planJointNames is not null)
        {
            hash.Add(planJointNames.Count);
            foreach (var n in planJointNames)
                hash.Add(n, StringComparer.Ordinal);
        }
        if (attached is not null)
        {
            hash.Add(attached.Count);
            foreach (var body in attached.OrderBy(b => b.Name, StringComparer.Ordinal))
            {
                hash.Add(body.Name, StringComparer.Ordinal);
                hash.Add(CollisionMeshCache.GeometryFingerprint(body.Geometry));
            }
        }
        if (scene is not null)
        {
            hash.Add(scene.Objects.Count);
            foreach (var obj in scene.Objects.OrderBy(o => o.Name, StringComparer.Ordinal))
                hash.Add(CollisionMeshCache.GeometryFingerprint(obj));
            foreach (var pair in scene.AllowedPairs.OrderBy(p => p.A, StringComparer.Ordinal).ThenBy(p => p.B, StringComparer.Ordinal))
            {
                hash.Add(pair.A, StringComparer.Ordinal);
                hash.Add(pair.B, StringComparer.Ordinal);
            }
        }
        return hash.ToHashCode();
    }

    /// <summary>
    /// Full cache key: everything a checker bakes in at construction (FK model, base and tool frames, link and tool
    /// geometry, chain, tree, plan joints, driver home, attached bodies). The int fingerprint is only the bucket;
    /// a hit must also pass <see cref="Equals(Key?)"/>. The scene is not baked into a checker (it is passed on each
    /// query), so it stays in the fingerprint only, as before.
    /// </summary>
    private sealed class Key : IEquatable<Key>
    {
        private readonly int _hash;
        private readonly RobotModel _robot;
        private readonly SerialJointChain? _chain;
        private readonly IReadOnlyList<AttachedBody>? _attached;
        private readonly KinematicTree? _tree;
        private readonly IReadOnlyList<string>? _planJointNames;
        private readonly IReadOnlyList<double>? _treeDriverHome;

        public Key(int hash, RobotModel robot, SerialJointChain? chain, IReadOnlyList<AttachedBody>? attached,
            KinematicTree? tree, IReadOnlyList<string>? planJointNames, IReadOnlyList<double>? treeDriverHome)
        {
            _hash = hash; _robot = robot; _chain = chain; _attached = attached?.ToArray(); _tree = tree;
            _planJointNames = planJointNames?.ToArray(); _treeDriverHome = treeDriverHome?.ToArray();
        }

        public override int GetHashCode() => _hash;
        public override bool Equals(object? obj) => Equals(obj as Key);

        public bool Equals(Key? o) =>
            o is not null && _hash == o._hash
            && ReferenceEquals(_chain, o._chain)
            && (_tree?.Fingerprint ?? 0) == (o._tree?.Fingerprint ?? 0)
            && SamePreset(_robot.Preset, o._robot.Preset)
            && SameModel(_robot.CollisionModel, o._robot.CollisionModel)
            && SameList(_planJointNames, o._planJointNames, string.Equals)
            && SameList(_treeDriverHome, o._treeDriverHome, (a, b) => a.Equals(b))
            && SameList(_attached, o._attached, (a, b) =>
                a.Name == b.Name && a.TcpLocalPose.Equals(b.TcpLocalPose)
                && a.SourceSceneObjectName == b.SourceSceneObjectName && SameGeometry(a.Geometry, b.Geometry));

        private static bool SamePreset(RobotPreset a, RobotPreset b) =>
            ReferenceEquals(a, b)
            || (a.ModelName == b.ModelName && a.AxisCount == b.AxisCount
                && a.BaseFrame.Frame.Equals(b.BaseFrame.Frame) && a.ToolFrame.Frame.Equals(b.ToolFrame.Frame));

        private static bool SameModel(RobotCollisionModel? a, RobotCollisionModel? b) =>
            ReferenceEquals(a, b)
            || (a is not null && b is not null
                && a.ToolGeometryInFlangeFrame == b.ToolGeometryInFlangeFrame
                && Nullable.Equals(a.ToolGeometryAttachOffset, b.ToolGeometryAttachOffset)
                && SameGeometry(a.ToolGeometry, b.ToolGeometry)
                && SameList(a.Links, b.Links, (x, y) =>
                    x.LinkIndex == y.LinkIndex && x.LinkName == y.LinkName && SameGeometry(x.LocalGeometry, y.LocalGeometry)));

        private static bool SameGeometry(CollisionObject? a, CollisionObject? b) =>
            ReferenceEquals(a, b)
            || (a is not null && b is not null
                && a.ContentHash == b.ContentHash && a.Name == b.Name && a.Shape == b.Shape && a.Pose.Equals(b.Pose)
                && a.ExtentX.Equals(b.ExtentX) && a.ExtentY.Equals(b.ExtentY) && a.ExtentZ.Equals(b.ExtentZ)
                && SameList(a.MeshIndices, b.MeshIndices, (x, y) => x == y)
                && SameList(a.MeshVertices, b.MeshVertices, (x, y) => x.AsSpan().SequenceEqual(y)));

        private static bool SameList<T>(IReadOnlyList<T>? a, IReadOnlyList<T>? b, Func<T, T, bool> same)
        {
            if (ReferenceEquals(a, b)) return true;
            if (a is null || b is null || a.Count != b.Count) return false;
            for (var i = 0; i < a.Count; i++)
                if (!same(a[i], b[i])) return false;
            return true;
        }
    }
}
