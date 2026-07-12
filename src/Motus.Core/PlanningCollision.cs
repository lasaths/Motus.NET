namespace Motus.Core;

/// <summary>Validates planned trajectories against a collision scene.</summary>
public static class PlanningCollision
{
    public static bool SceneHasObstacles(CollisionScene? scene) =>
        scene is not null && scene.Objects.Count > 0;

    public static PlanningResult? ValidateTrajectory(
        Trajectory trajectory,
        CollisionScene scene,
        ICollisionChecker checker,
        double segmentStepRadians)
    {
        var errors = new List<string>();
        TrajectoryPoint? prev = null;
        foreach (var pt in trajectory.Points)
        {
            if (!checker.IsCollisionFree(pt.JointState, scene))
                errors.Add($"Collision at t={pt.TimeSeconds:F4}s.");
            if (prev is not null && !checker.SegmentCollisionFree(prev.JointState, pt.JointState, scene, segmentStepRadians))
                errors.Add($"Collision between t={prev.TimeSeconds:F4}s and t={pt.TimeSeconds:F4}s.");
            prev = pt;
        }
        return errors.Count == 0 ? null : PlanningResult.Failed(errors);
    }

    /// <summary>Fast-fail when start or goal already intersects the collision scene.</summary>
    public static PlanningResult? ValidateEndpoints(
        JointState start,
        JointState goal,
        CollisionScene? scene,
        ICollisionChecker? checker)
    {
        if (checker is null || !SceneHasObstacles(scene))
            return null;

        if (!checker.IsCollisionFree(start, scene!))
        {
            return PlanningResult.Failed(new[]
            {
                checker.IsCollisionFree(start, new CollisionScene())
                    ? "Start configuration is in collision with an obstacle."
                    : "Start configuration has self-collision."
            });
        }
        if (!checker.IsCollisionFree(goal, scene!))
        {
            return PlanningResult.Failed(new[]
            {
                checker.IsCollisionFree(goal, new CollisionScene())
                    ? "Goal configuration is in collision with an obstacle."
                    : "Goal configuration has self-collision."
            });
        }
        return null;
    }
}
