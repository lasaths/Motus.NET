using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Motus.Core;

public sealed class TrajectoryExportOptions
{
    public bool Retime { get; init; }
    public bool Validate { get; init; }
    public TrajectoryRetimerOptions? Retimer { get; init; }
    public TrajectoryValidationOptions? Validation { get; init; }
    /// <summary>When set, included in JSON export if it differs from the trajectory robot preset tool frame.</summary>
    public ToolFrame? SessionToolFrame { get; init; }
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
        var retimer = options.Retimer ?? new TrajectoryRetimerOptions { Algorithm = RetimerAlgorithm.TotgLite };
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
            ToJson(prepared, options),
            ToCsv(prepared, options.Retime),
            validation);
    }

    public static string ToJson(Trajectory trajectory, bool retime = false) =>
        ToJson(trajectory, retime ? new TrajectoryExportOptions { Retime = true } : null);

    public static string ToJson(Trajectory trajectory, TrajectoryExportOptions? options)
    {
        options ??= new TrajectoryExportOptions();
        var traj = Prepare(trajectory, options);
        var jointNames = traj.Robot.JointNames;
        var toolFrame = ResolveExportToolFrame(traj.Robot, options.SessionToolFrame);
        var obj = new
        {
            robot = traj.Robot.DisplayName,
            jointNames,
            durationSeconds = traj.DurationSeconds,
            pointCount = traj.Points.Count,
            retimed = options.Retime,
            toolFrame,
            points = traj.Points.Select(p =>
            {
                Dictionary<string, double>? joints = null;
                if (jointNames is not null)
                {
                    joints = new Dictionary<string, double>();
                    for (var i = 0; i < jointNames.Count; i++)
                        joints[jointNames[i]] = p.JointState.Positions[i];
                }
                return new
                {
                    timeSeconds = p.TimeSeconds,
                    jointsRadians = p.JointState.Positions,
                    joints,
                    motionType = p.MotionType?.ToString().ToLowerInvariant(),
                    segmentIndex = p.SegmentIndex,
                    blendRadiusMeters = p.BlendRadiusMeters
                };
            })
        };
        return JsonSerializer.Serialize(obj, new JsonSerializerOptions
        {
            WriteIndented = true,
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull
        });
    }

    public static string ToCsv(Trajectory trajectory, bool retime = false) =>
        ToCsv(trajectory, retime ? new TrajectoryExportOptions { Retime = true } : null);

    public static string ToCsv(Trajectory trajectory, TrajectoryExportOptions? options)
    {
        options ??= new TrajectoryExportOptions();
        var traj = Prepare(trajectory, options);
        var n = traj.Robot.Preset.AxisCount;
        var hasMotionMetadata = traj.Points.Any(p => p.MotionType is not null || p.SegmentIndex is not null || p.BlendRadiusMeters is not null);
        var sb = new StringBuilder();
        sb.Append("time_seconds");
        for (var i = 1; i <= n; i++) sb.Append($",joint_{i}_rad");
        if (hasMotionMetadata) sb.Append(",motion_type,segment_index,blend_radius_m");
        sb.AppendLine();
        foreach (var p in traj.Points)
        {
            sb.Append(p.TimeSeconds.ToString("F6"));
            foreach (var j in p.JointState.Positions)
                sb.Append(',').Append(j.ToString("F6"));
            if (hasMotionMetadata)
            {
                sb.Append(',').Append(p.MotionType?.ToString().ToLowerInvariant() ?? string.Empty);
                sb.Append(',').Append(p.SegmentIndex?.ToString() ?? string.Empty);
                sb.Append(',').Append(p.BlendRadiusMeters?.ToString("F6") ?? string.Empty);
            }
            sb.AppendLine();
        }
        return sb.ToString();
    }

    private static object? ResolveExportToolFrame(RobotModel robot, ToolFrame? sessionTool)
    {
        var presetTool = robot.Preset.ToolFrame;
        var tool = sessionTool ?? presetTool;
        if (sessionTool is null && FramesEqual(tool.Frame, presetTool.Frame) &&
            string.Equals(tool.Name, presetTool.Name, StringComparison.Ordinal))
            return null;

        return new
        {
            name = tool.Name,
            x = tool.Frame.X,
            y = tool.Frame.Y,
            z = tool.Frame.Z,
            qw = tool.Frame.Qw,
            qx = tool.Frame.Qx,
            qy = tool.Frame.Qy,
            qz = tool.Frame.Qz
        };
    }

    private static bool FramesEqual(Frame a, Frame b) =>
        Math.Abs(a.X - b.X) < 1e-9 && Math.Abs(a.Y - b.Y) < 1e-9 && Math.Abs(a.Z - b.Z) < 1e-9 &&
        Math.Abs(a.Qw - b.Qw) < 1e-9 && Math.Abs(a.Qx - b.Qx) < 1e-9 &&
        Math.Abs(a.Qy - b.Qy) < 1e-9 && Math.Abs(a.Qz - b.Qz) < 1e-9;
}
