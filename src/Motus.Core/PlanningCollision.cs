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
            if (prev is not null &&
                !JointDeltaWithinStep(prev.JointState, pt.JointState, segmentStepRadians) &&
                !checker.SegmentCollisionFree(prev.JointState, pt.JointState, scene, segmentStepRadians))
                errors.Add($"Collision between t={prev.TimeSeconds:F4}s and t={pt.TimeSeconds:F4}s.");
            prev = pt;
        }
        if (errors.Count == 0) return null;
        return PlanningResult.Failed(errors.Select(e =>
            new PlanningMessage(PlanningMessageCodes.PathCollision, e, PlanningMessageSeverity.Error)));
    }

    /// <summary>True when every joint delta is ≤ step — point checks already cover the segment.</summary>
    public static bool JointDeltaWithinStep(JointState from, JointState to, double stepRadians)
    {
        if (stepRadians <= 0) return false;
        var n = Math.Min(from.AxisCount, to.AxisCount);
        for (var i = 0; i < n; i++)
        {
            if (Math.Abs(to.Positions[i] - from.Positions[i]) > stepRadians)
                return false;
        }
        return true;
    }

    /// <summary>Fast-fail when start or goal already intersects the collision scene.</summary>
    public static PlanningResult? ValidateEndpoints(
        JointState start,
        JointState goal,
        CollisionScene? scene,
        ICollisionChecker? checker,
        bool includeAttachedBodies = false)
    {
        // No obstacles → nothing to endpoint-check (attached-only empty scenes still trip SelfCollisionFree).
        if (checker is null || !SceneHasObstacles(scene))
            return null;
        scene ??= new CollisionScene();

        if (!checker.IsCollisionFree(start, scene))
        {
            return PlanningResult.Failed(new[]
            {
                new PlanningMessage(
                    PlanningMessageCodes.EndpointCollision,
                    checker.IsCollisionFree(start, new CollisionScene())
                        ? "Start configuration is in collision with an obstacle."
                        : "Start configuration has self-collision.",
                    PlanningMessageSeverity.Error)
            });
        }
        if (!checker.IsCollisionFree(goal, scene))
        {
            return PlanningResult.Failed(new[]
            {
                new PlanningMessage(
                    PlanningMessageCodes.EndpointCollision,
                    checker.IsCollisionFree(goal, new CollisionScene())
                        ? "Goal configuration is in collision with an obstacle."
                        : "Goal configuration has self-collision.",
                    PlanningMessageSeverity.Error)
            });
        }
        return null;
    }
}
