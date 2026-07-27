namespace Motus.Core;

/// <summary>Named collision bodies for allowed-pair filtering (SRDF-lite).</summary>
public static class CollisionBodies
{
    // PONYTAIL: cache link names — hot loops called IsPairAllowed with $"link:{i}" every check.
    private static readonly object LinkNameGate = new();
    private static string[] _linkNames = Array.Empty<string>();

    public static string RobotLink(int index)
    {
        // ponytail: negative indices are plane-proximal aliases (link:-1), not cache slots
        if (index < 0)
            return $"link:{index}";
        var names = _linkNames;
        if ((uint)index < (uint)names.Length)
            return names[index];
        return GrowAndGet(index);
    }

    public static string Attached(string name) => $"attached:{name}";

    private static string GrowAndGet(int index)
    {
        lock (LinkNameGate)
        {
            if ((uint)index >= (uint)_linkNames.Length)
                GrowLinkNames(index + 1);
            return _linkNames[index];
        }
    }

    private static void GrowLinkNames(int count)
    {
        var next = new string[count];
        Array.Copy(_linkNames, next, _linkNames.Length);
        for (var i = _linkNames.Length; i < count; i++)
            next[i] = $"link:{i}";
        _linkNames = next;
    }
}

public sealed class CollisionScene
{
    public IReadOnlyList<CollisionObject> Objects { get; }
    /// <summary>Skip checks between these named body pairs (order-independent).</summary>
    public IReadOnlyList<(string A, string B)> AllowedPairs { get; }

    private readonly HashSet<(string, string)>? _allowedSet;

    public CollisionScene(IReadOnlyList<CollisionObject>? objects = null, IReadOnlyList<(string, string)>? allowedPairs = null)
    {
        Objects = objects ?? Array.Empty<CollisionObject>();
        // ponytail: plane at robot origin clips proximal envelopes — skip link:-1..1 vs planes
        AllowedPairs = WithPlaneBasePairs(Objects, allowedPairs);
        if (AllowedPairs.Count > 0)
        {
            _allowedSet = new HashSet<(string, string)>();
            foreach (var (a, b) in AllowedPairs)
                _allowedSet.Add(NormalizePair(a, b));
        }
    }

    public bool IsPairAllowed(string bodyA, string bodyB)
    {
        if (_allowedSet is null) return false;
        return _allowedSet.Contains(NormalizePair(bodyA, bodyB));
    }

    private static (string, string) NormalizePair(string a, string b) =>
        string.CompareOrdinal(a, b) <= 0 ? (a, b) : (b, a);

    private static IReadOnlyList<(string A, string B)> WithPlaneBasePairs(
        IReadOnlyList<CollisionObject> objects,
        IReadOnlyList<(string, string)>? allowedPairs)
    {
        List<(string, string)>? merged = null;
        foreach (var obj in objects)
        {
            if (obj.Shape != CollisionShape.Plane) continue;
            merged ??= allowedPairs is null
                ? new List<(string, string)>()
                : new List<(string, string)>(allowedPairs);
            // UR/industrial base+shoulder sit on the mounting plane
            for (var i = -1; i <= 1; i++)
                EnsurePair(merged, CollisionBodies.RobotLink(i), obj.Name);
        }

        return merged ?? allowedPairs ?? Array.Empty<(string, string)>();
    }

    private static void EnsurePair(List<(string, string)> pairs, string a, string b)
    {
        foreach (var (x, y) in pairs)
        {
            if ((x == a && y == b) || (x == b && y == a))
                return;
        }
        pairs.Add((a, b));
    }
}
