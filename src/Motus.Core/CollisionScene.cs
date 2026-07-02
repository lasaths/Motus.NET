namespace Motus.Core;

/// <summary>Named collision bodies for allowed-pair filtering (SRDF-lite).</summary>
public static class CollisionBodies
{
    public static string RobotLink(int index) => $"link:{index}";
}

public sealed class CollisionScene
{
    public IReadOnlyList<CollisionObject> Objects { get; }
    /// <summary>Skip checks between these named body pairs (order-independent).</summary>
    public IReadOnlyList<(string A, string B)> AllowedPairs { get; }

    public CollisionScene(IReadOnlyList<CollisionObject>? objects = null, IReadOnlyList<(string, string)>? allowedPairs = null)
    {
        Objects = objects ?? Array.Empty<CollisionObject>();
        AllowedPairs = allowedPairs ?? Array.Empty<(string, string)>();
    }

    public bool IsPairAllowed(string bodyA, string bodyB)
    {
        foreach (var (a, b) in AllowedPairs)
        {
            if ((a == bodyA && b == bodyB) || (a == bodyB && b == bodyA))
                return true;
        }
        return false;
    }
}
