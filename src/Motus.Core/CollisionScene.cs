namespace Motus.Core;

/// <summary>Named collision bodies for allowed-pair filtering (SRDF-lite).</summary>
public static class CollisionBodies
{
    // PONYTAIL: cache link names — hot loops called IsPairAllowed with $"link:{i}" every check.
    private static string[] _linkNames = Array.Empty<string>();

    public static string RobotLink(int index)
    {
        if ((uint)index >= (uint)_linkNames.Length)
            GrowLinkNames(index + 1);
        return _linkNames[index];
    }

    public static string Attached(string name) => $"attached:{name}";

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
        AllowedPairs = allowedPairs ?? Array.Empty<(string, string)>();
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
}
