using Motus.Core;

namespace Motus.Geometry;

/// <summary>Collision checks using width-varying tool geometry from end-effector state.</summary>
public static class ToolStateCollision
{
    public static IReadOnlyList<string> ValidateTrajectory(
        Trajectory trajectory,
        ToolDefinition sessionTool,
        CollisionScene? scene,
        ICollisionChecker? checker)
    {
        if (sessionTool.Capabilities is null || !PlanningCollision.SceneHasObstacles(scene))
            return Array.Empty<string>();

        checker ??= CollisionCheckerFactory.Create(trajectory.Robot);
        if (checker is null)
            return new[] { "Tool-state collision check skipped: no collision checker available." };

        var warnings = new List<string>();
        foreach (var point in trajectory.Points)
        {
            var geom = sessionTool.GeometryForState(point.ToolState);
            if (geom is null) continue;

            var session = trajectory.Robot.WithTool(new ToolDefinition(
                sessionTool.Name, sessionTool.Tcp, geom, sessionTool.Capabilities));
            var stateChecker = CollisionCheckerFactory.Create(session) ?? checker;
            if (!stateChecker.IsCollisionFree(point.JointState, scene!))
            {
                var width = point.ToolState?.GetValueOrDefault("width");
                var widthText = width is { } w ? $"width={w:F4}m" : "static tool";
                warnings.Add($"Tool-state collision at t={point.TimeSeconds:F2}s ({widthText}).");
            }
        }

        return warnings;
    }
}
