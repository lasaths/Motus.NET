using System.Text;
using System.Text.Json;

namespace Motus.Core;

public static class TrajectoryExport
{
    public static string ToJson(Trajectory trajectory)
    {
        var obj = new
        {
            robot = trajectory.Robot.DisplayName,
            durationSeconds = trajectory.DurationSeconds,
            pointCount = trajectory.Points.Count,
            points = trajectory.Points.Select(p => new
            {
                timeSeconds = p.TimeSeconds,
                jointsRadians = p.JointState.Positions
            })
        };
        return JsonSerializer.Serialize(obj, new JsonSerializerOptions { WriteIndented = true });
    }

    public static string ToCsv(Trajectory trajectory)
    {
        var n = trajectory.Robot.Preset.AxisCount;
        var sb = new StringBuilder();
        sb.Append("time_seconds");
        for (var i = 1; i <= n; i++) sb.Append($",joint_{i}_rad");
        sb.AppendLine();
        foreach (var p in trajectory.Points)
        {
            sb.Append(p.TimeSeconds.ToString("F6"));
            foreach (var j in p.JointState.Positions)
                sb.Append(',').Append(j.ToString("F6"));
            sb.AppendLine();
        }
        return sb.ToString();
    }
}
