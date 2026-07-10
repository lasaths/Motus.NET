using Motus.Core;

namespace Motus.Geometry;

public sealed record CartesianWorkspaceCheck(bool IsWithinReach, string? Reason)
{
    public static CartesianWorkspaceCheck Ok() => new(true, null);
    public static CartesianWorkspaceCheck Reject(string reason) => new(false, reason);
}

/// <summary>Fast geometric workspace checks using preset reach metadata.</summary>
public static class CartesianWorkspace
{
    private const double ReachMargin = 1.05;

    public static CartesianWorkspaceCheck CheckReach(
        RobotPreset preset,
        CartesianPose goal,
        CartesianPose? startPose = null)
    {
        if (preset.ReachMeters is not { } reach || reach <= 0)
            return CartesianWorkspaceCheck.Ok();

        var maxReach = reach * ReachMargin;
        var goalDist = DistanceFromBase(goal);
        if (goalDist > maxReach)
        {
            return CartesianWorkspaceCheck.Reject(
                $"Goal TCP is outside robot reach (~{reach:F2} m).");
        }

        if (startPose is null)
            return CartesianWorkspaceCheck.Ok();

        var tcpDist = TcpDistance(startPose, goal);
        if (tcpDist > 2 * maxReach)
        {
            return CartesianWorkspaceCheck.Reject(
                $"Goal TCP is too far from start for a straight-line move (~{tcpDist:F2} m).");
        }

        return CartesianWorkspaceCheck.Ok();
    }

    private static double DistanceFromBase(CartesianPose pose)
    {
        var x = pose.Tcp.X;
        var y = pose.Tcp.Y;
        var z = pose.Tcp.Z;
        return Math.Sqrt(x * x + y * y + z * z);
    }

    private static double TcpDistance(CartesianPose a, CartesianPose b)
    {
        var dx = b.Tcp.X - a.Tcp.X;
        var dy = b.Tcp.Y - a.Tcp.Y;
        var dz = b.Tcp.Z - a.Tcp.Z;
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }
}
