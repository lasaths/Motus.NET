namespace Motus.Geometry;

/// <summary>
/// Link-to-link check for <see cref="TreeFkCollisionChecker"/>. Every pair of geometry on different tree links is
/// tested in both directions: link A against link B grown by <paramref name="ClearanceM"/>, and B against A grown.
/// Pairs in <paramref name="AllowedPairs"/> (link names, either order) and pairs the scene allows are skipped.
/// Use <see cref="TreeAdjacency.AllowedPairs"/> for the adjacent links; add SRDF pairs as needed.
/// </summary>
public sealed record TreeSelfCollision(double ClearanceM, IReadOnlyCollection<(string A, string B)> AllowedPairs);
