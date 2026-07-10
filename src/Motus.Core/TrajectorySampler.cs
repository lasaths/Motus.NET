namespace Motus.Core;

/// <summary>Sample joint state along a planned trajectory at a given time.</summary>
public static class TrajectorySampler
{
    public static JointState AtTime(Trajectory trajectory, double elapsedSeconds, out int segmentIndex)
    {
        var pts = trajectory.Points;
        if (pts.Count == 0) throw new InvalidOperationException("Empty trajectory.");
        if (pts.Count == 1 || elapsedSeconds <= pts[0].TimeSeconds)
        {
            segmentIndex = 0;
            return pts[0].JointState;
        }

        if (elapsedSeconds >= pts[^1].TimeSeconds)
        {
            segmentIndex = Math.Max(0, pts.Count - 2);
            return pts[^1].JointState;
        }

        for (var i = 0; i < pts.Count - 1; i++)
        {
            var t0 = pts[i].TimeSeconds;
            var t1 = pts[i + 1].TimeSeconds;
            if (elapsedSeconds < t1 || i == pts.Count - 2)
            {
                segmentIndex = i;
                if (t1 <= t0 + 1e-12) return pts[i].JointState;
                var alpha = (elapsedSeconds - t0) / (t1 - t0);
                return Lerp(pts[i].JointState, pts[i + 1].JointState, Math.Clamp(alpha, 0, 1));
            }
        }

        segmentIndex = pts.Count - 2;
        return pts[^1].JointState;
    }

    private static JointState Lerp(JointState a, JointState b, double alpha)
    {
        var n = a.AxisCount;
        var q = new double[n];
        for (var i = 0; i < n; i++)
        {
            var delta = b.Positions[i] - a.Positions[i];
            while (delta > Math.PI) delta -= 2 * Math.PI;
            while (delta < -Math.PI) delta += 2 * Math.PI;
            q[i] = a.Positions[i] + alpha * delta;
        }
        return new JointState(q);
    }
}
