using Motus.Core;

namespace Motus.Geometry;

public sealed record CartesianReachResult(bool Success, JointState? Solution, IReadOnlyList<string> Errors)
{
    public static CartesianReachResult Succeeded(JointState solution) =>
        new(true, solution, Array.Empty<string>());

    public static CartesianReachResult Failed(params string[] errors) =>
        new(false, null, errors);
}
