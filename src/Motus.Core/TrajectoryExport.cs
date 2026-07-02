using System.Text;
using System.Text.Json;

namespace Motus.Core;

public sealed class TrajectoryExportOptions
{
    public bool Retime { get; init; }
    public bool Validate { get; init; }
    public TrajectoryRetimerOptions? Retimer { get; init; }
    public TrajectoryValidationOptions? Validation { get; init; }
}

public sealed class TrajectoryExportResult
{
    public Trajectory Trajectory { get; }
    public string Json { get; }
    public string Csv { get; }
    public ValidationResult? Validation { get; }

    public TrajectoryExportResult(Trajectory trajectory, string json, string csv, ValidationResult? validation)
    {
        Trajectory = trajectory;
        Json = json;
        Csv = csv;
        Validation = validation;
    }
}

public static class TrajectoryExport
{
    public static Trajectory Prepare(Trajectory trajectory, TrajectoryExportOptions? options = null)
    {
        options ??= new TrajectoryExportOptions();
        if (!options.Retime) return trajectory;
        var retimer = options.Retimer ?? new TrajectoryRetimerOptions { Algorithm = RetimerAlgorithm.Bottleneck };
        return TrajectoryRetimer.Retime(trajectory, retimer);
    }

    public static TrajectoryExportResult Export(Trajectory trajectory, TrajectoryExportOptions? options = null)
    {
        options ??= new TrajectoryExportOptions();
        var prepared = Prepare(trajectory, options);
        ValidationResult? validation = null;
        if (options.Validate)
            validation = new TrajectoryValidator().Validate(prepared, options.Validation);

        return new TrajectoryExportResult(
            prepared,
            ToJson(prepared, options.Retime),
            ToCsv(prepared, options.Retime),
            validation);
    }

    public static string ToJson(Trajectory trajectory, bool retime = false) =>
        ToJson(trajectory, retime ? new TrajectoryExportOptions { Retime = true } : null);

    public static string ToJson(Trajectory trajectory, TrajectoryExportOptions? options)
    {
        options ??= new TrajectoryExportOptions();
        var traj = Prepare(trajectory, options);
        var obj = new
        {
            robot = traj.Robot.DisplayName,
            durationSeconds = traj.DurationSeconds,
            pointCount = traj.Points.Count,
            retimed = options.Retime,
            points = traj.Points.Select(p => new
            {
                timeSeconds = p.TimeSeconds,
                jointsRadians = p.JointState.Positions
            })
        };
        return JsonSerializer.Serialize(obj, new JsonSerializerOptions { WriteIndented = true });
    }

    public static string ToCsv(Trajectory trajectory, bool retime = false) =>
        ToCsv(trajectory, retime ? new TrajectoryExportOptions { Retime = true } : null);

    public static string ToCsv(Trajectory trajectory, TrajectoryExportOptions? options)
    {
        options ??= new TrajectoryExportOptions();
        var traj = Prepare(trajectory, options);
        var n = traj.Robot.Preset.AxisCount;
        var sb = new StringBuilder();
        sb.Append("time_seconds");
        for (var i = 1; i <= n; i++) sb.Append($",joint_{i}_rad");
        sb.AppendLine();
        foreach (var p in traj.Points)
        {
            sb.Append(p.TimeSeconds.ToString("F6"));
            foreach (var j in p.JointState.Positions)
                sb.Append(',').Append(j.ToString("F6"));
            sb.AppendLine();
        }
        return sb.ToString();
    }
}
