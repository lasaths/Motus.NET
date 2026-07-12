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
        CollisionScene? scene)
    {
        var key = Fingerprint(robot, chain, attached, scene);
        lock (Gate)
        {
            if (Cache.TryGetValue(key, out var weak) && weak.TryGetTarget(out var existing))
                return existing;

            var checker = CollisionCheckerFactory.Create(robot, chain, attached);
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
        CollisionScene? scene)
    {
        var hash = new HashCode();
        hash.Add(robot.Preset.ModelName, StringComparer.Ordinal);
        hash.Add(chain?.GetHashCode() ?? 0);
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
