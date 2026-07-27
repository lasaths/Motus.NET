using Motus.Core;

namespace Motus.Geometry;

/// <summary>Reuses collision checkers for identical robot / attach / scene fingerprints within a process.</summary>
public static class CollisionCheckerSessionCache
{
    private static readonly Dictionary<int, WeakReference<ICollisionChecker>> Cache = new();
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
        var key = Fingerprint(robot, chain, attached, scene, tree, planJointNames);
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
            PruneDead();
            return checker;
        }
    }

    public static void Clear() { lock (Gate) Cache.Clear(); }

    private static void PruneDead()
    {
        foreach (var key in Cache.Keys.ToList())
        {
            if (!Cache[key].TryGetTarget(out _))
                Cache.Remove(key);
        }
    }

    internal static int Fingerprint(
        RobotModel robot,
        SerialJointChain? chain,
        IReadOnlyList<AttachedBody>? attached,
        CollisionScene? scene,
        KinematicTree? tree = null,
        IReadOnlyList<string>? planJointNames = null)
    {
        var hash = new HashCode();
        hash.Add(robot.Preset.ModelName, StringComparer.Ordinal);
        hash.Add(robot.Preset.AxisCount);
        hash.Add(chain?.GetHashCode() ?? 0);
        hash.Add(tree?.Fingerprint ?? 0);
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
}
