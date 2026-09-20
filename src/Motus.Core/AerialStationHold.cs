namespace Motus.Core;

/// <summary>
/// Offline aerial station-hold: duplicate the last HolonomicSE3 sample with a dwell
/// (Motus 2.1 ADR 0002). Not a flight controller hover loop.
/// </summary>
public static class AerialStationHold
{
    /// <summary>
    /// Appends a hold sample at the last point's base/joints for <paramref name="holdSeconds"/>.
    /// Returns <paramref name="trajectory"/> unchanged when hold ≤ 0 or the path is empty.
    /// Rejects non-finite hold with <see cref="ArgumentOutOfRangeException"/>.
    /// </summary>
    public static Trajectory AppendHold(Trajectory trajectory, double holdSeconds)
    {
        ArgumentNullException.ThrowIfNull(trajectory);
        if (!double.IsFinite(holdSeconds))
            throw new ArgumentOutOfRangeException(nameof(holdSeconds), "Hold duration must be finite.");
        if (holdSeconds <= 0 || trajectory.Points.Count == 0)
            return trajectory;

        var last = trajectory.Points[^1];
        var hold = new TrajectoryPoint(
            last.TimeSeconds + holdSeconds,
            last.JointState,
            last.MotionType,
            last.SegmentIndex,
            last.BlendRadiusMeters,
            last.ToolState,
            last.BaseFrameOverride);
        var points = new TrajectoryPoint[trajectory.Points.Count + 1];
        for (var i = 0; i < trajectory.Points.Count; i++)
            points[i] = trajectory.Points[i];
        points[^1] = hold;
        return new Trajectory(trajectory.Robot, points, trajectory.AttachSpans);
    }
}
