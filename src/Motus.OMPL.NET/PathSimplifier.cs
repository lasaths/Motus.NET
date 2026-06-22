using Motus.Core;
using Motus.Geometry;

namespace Motus.OMPL.NET;

public static class PathSimplifier
{
    public static IReadOnlyList<JointState> Simplify(
        IReadOnlyList<JointState> path,
        RobotModel robot,
        SphereCollisionChecker? collision,
        CollisionScene scene,
        double stepRadians)
    {
        if (path.Count <= 2) return path;
        var simplified = new List<JointState> { path[0] };
        var anchor = 0;
        while (anchor < path.Count - 1)
        {
            var farthest = anchor + 1;
            for (var i = path.Count - 1; i > anchor; i--)
            {
                if (collision is null || collision.SegmentCollisionFree(path[anchor], path[i], scene, stepRadians))
                {
                    farthest = i;
                    break;
                }
            }
            simplified.Add(path[farthest]);
            anchor = farthest;
        }
        return simplified;
    }
}
