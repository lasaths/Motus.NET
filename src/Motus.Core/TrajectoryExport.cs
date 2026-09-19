using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Globalization;

namespace Motus.Core;

public static class PlanBundleContract
{
    public const int ExportVersion = 1;
    public const string ContractVersion = "1.0.0";
}

public sealed class PlannerProvenance
{
    public string? PlannerId { get; init; }
    public int? RandomSeed { get; init; }
    public string? SettingsHash { get; init; }
    public string? RetimeAlgorithm { get; init; }
}

public sealed class TrajectoryExportOptions
{
    public bool Retime { get; init; }
    public bool Validate { get; init; }
    public TrajectoryRetimerOptions? Retimer { get; init; }
    public TrajectoryValidationOptions? Validation { get; init; }
    /// <summary>When set, included in JSON export if it differs from the trajectory robot preset tool frame.</summary>
    public ToolFrame? SessionToolFrame { get; init; }
    /// <summary>Tool parameter schema for export header.</summary>
    public ToolCapabilities? ToolCapabilities { get; init; }
    /// <summary>Stable diagnostics with machine-readable code and severity.</summary>
    public IReadOnlyList<PlanningMessage>? Diagnostics { get; init; }
    /// <summary>Planner provenance metadata for reproducibility/debugging.</summary>
    public PlannerProvenance? Provenance { get; init; }
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
            SerializeJson(prepared, options),
            SerializeCsv(prepared),
            validation);
    }

    public static string ToJson(Trajectory trajectory, bool retime = false) =>
        ToJson(trajectory, retime ? new TrajectoryExportOptions { Retime = true } : null);

    public static string ToJson(Trajectory trajectory, TrajectoryExportOptions? options)
    {
        options ??= new TrajectoryExportOptions();
        return SerializeJson(Prepare(trajectory, options), options);
    }

    private static string SerializeJson(Trajectory traj, TrajectoryExportOptions options)
    {
        var jointNames = traj.Robot.JointNames;
        var toolFrame = ResolveExportToolFrame(traj.Robot, options.SessionToolFrame);
        var toolCapabilities = options.ToolCapabilities;
        var diagnostics = options.Diagnostics;
        var provenance = ResolveProvenance(options);
        var jsonOptions = new JsonSerializerOptions
        {
            WriteIndented = true,
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull
        };

        // Stewart-only shape (meters). Serial keeps PlanBundle golden fixture shape below.
        if (Units.IsStewart(traj.Robot.Preset))
        {
            var stewartObj = new
            {
                exportVersion = PlanBundleContract.ExportVersion,
                contractVersion = PlanBundleContract.ContractVersion,
                robot = traj.Robot.DisplayName,
                family = traj.Robot.Preset.Family,
                jointNames,
                units = new
                {
                    jointCoordinates = "meters",
                    legLengths = "meters",
                    time = "seconds",
                    distance = "meters"
                },
                frameConvention = new
                {
                    baseFrame = "robot_base",
                    tcpFrame = "tool_center_point",
                    jointOrder = "robot.jointNames order"
                },
                durationSeconds = traj.DurationSeconds,
                pointCount = traj.Points.Count,
                retimed = options.Retime,
                provenance = provenance is null ? null : new
                {
                    plannerId = provenance.PlannerId,
                    randomSeed = provenance.RandomSeed,
                    settingsHash = provenance.SettingsHash,
                    retimeAlgorithm = provenance.RetimeAlgorithm
                },
                diagnostics = diagnostics?.Select(d => new
                {
                    code = d.Code,
                    severity = d.Severity.ToString().ToLowerInvariant(),
                    message = d.Message
                }),
                toolFrame,
                toolCapabilities = toolCapabilities is null ? null : toolCapabilities.Parameters.Select(p => new
                {
                    name = p.Name,
                    unit = p.Unit,
                    min = p.Min,
                    max = p.Max,
                    defaultValue = p.Default
                }),
                attachSpans = AttachmentData(traj),
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
                        jointCoordinates = p.JointState.Positions,
                        joints,
                        motionType = p.MotionType?.ToString().ToLowerInvariant(),
                        segmentIndex = p.SegmentIndex,
                        blendRadiusMeters = p.BlendRadiusMeters,
                        toolState = p.ToolState?.Values
                    };
                })
            };
            return JsonSerializer.Serialize(stewartObj, jsonOptions);
        }

        // Aerial / HolonomicSE3: body poses — not UR MoveJ radians.
        if (Units.IsAerial(traj.Robot.Preset))
        {
            var aerialObj = new
            {
                exportVersion = PlanBundleContract.ExportVersion,
                contractVersion = PlanBundleContract.ContractVersion,
                robot = traj.Robot.DisplayName,
                family = traj.Robot.Preset.Family,
                jointNames,
                units = new
                {
                    bodyPosition = "meters",
                    bodyOrientation = "radians_rpy_fixed_xyz",
                    time = "seconds",
                    distance = "meters",
                    // Explicit: empty/prop joints are not industrial MoveJ coordinates.
                    waypointsQ = "not_ur_movej"
                },
                frameConvention = new
                {
                    baseFrame = "free_flyer_body",
                    orientation = "rpy_fixed_axis_xyz_urdf",
                    tcpFrame = "tool_center_point",
                    jointOrder = "robot.jointNames order (often empty for pure free-flyer)"
                },
                durationSeconds = traj.DurationSeconds,
                pointCount = traj.Points.Count,
                retimed = options.Retime,
                provenance = provenance is null ? null : new
                {
                    plannerId = provenance.PlannerId,
                    randomSeed = provenance.RandomSeed,
                    settingsHash = provenance.SettingsHash,
                    retimeAlgorithm = provenance.RetimeAlgorithm
                },
                diagnostics = diagnostics?.Select(d => new
                {
                    code = d.Code,
                    severity = d.Severity.ToString().ToLowerInvariant(),
                    message = d.Message
                }),
                toolFrame,
                toolCapabilities = toolCapabilities is null ? null : toolCapabilities.Parameters.Select(p => new
                {
                    name = p.Name,
                    unit = p.Unit,
                    min = p.Min,
                    max = p.Max,
                    defaultValue = p.Default
                }),
                attachSpans = AttachmentData(traj),
                points = traj.Points.Select(p =>
                {
                    Dictionary<string, double>? joints = null;
                    if (jointNames is not null && jointNames.Count > 0)
                    {
                        joints = new Dictionary<string, double>();
                        for (var i = 0; i < jointNames.Count; i++)
                            joints[jointNames[i]] = p.JointState.Positions[i];
                    }

                    var body = ResolveAerialBodyPose(traj.Robot, p);
                    return new
                    {
                        timeSeconds = p.TimeSeconds,
                        bodyPose = body is null ? null : new
                        {
                            x = body.Value.X,
                            y = body.Value.Y,
                            z = body.Value.Z,
                            roll = body.Value.Roll,
                            pitch = body.Value.Pitch,
                            yaw = body.Value.Yaw
                        },
                        baseFrame = p.BaseFrameOverride is { } bf ? FrameData(bf.Frame) : null,
                        propJoints = joints,
                        motionType = p.MotionType?.ToString().ToLowerInvariant(),
                        segmentIndex = p.SegmentIndex,
                        blendRadiusMeters = p.BlendRadiusMeters,
                        toolState = p.ToolState?.Values
                    };
                })
            };
            return JsonSerializer.Serialize(aerialObj, jsonOptions);
        }

        var obj = new
        {
            exportVersion = PlanBundleContract.ExportVersion,
            contractVersion = PlanBundleContract.ContractVersion,
            robot = traj.Robot.DisplayName,
            jointNames,
            units = new
            {
                jointAngles = "radians",
                time = "seconds",
                distance = "meters"
            },
            frameConvention = new
            {
                baseFrame = "robot_base",
                tcpFrame = "tool_center_point",
                jointOrder = "robot.jointNames order"
            },
            durationSeconds = traj.DurationSeconds,
            pointCount = traj.Points.Count,
            retimed = options.Retime,
            provenance = provenance is null ? null : new
            {
                plannerId = provenance.PlannerId,
                randomSeed = provenance.RandomSeed,
                settingsHash = provenance.SettingsHash,
                retimeAlgorithm = provenance.RetimeAlgorithm
            },
            diagnostics = diagnostics?.Select(d => new
            {
                code = d.Code,
                severity = d.Severity.ToString().ToLowerInvariant(),
                message = d.Message
            }),
            toolFrame,
            toolCapabilities = toolCapabilities is null ? null : toolCapabilities.Parameters.Select(p => new
            {
                name = p.Name,
                unit = p.Unit,
                min = p.Min,
                max = p.Max,
                defaultValue = p.Default
            }),
            attachSpans = AttachmentData(traj),
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
                    blendRadiusMeters = p.BlendRadiusMeters,
                    toolState = p.ToolState?.Values
                };
            })
        };
        return JsonSerializer.Serialize(obj, jsonOptions);
    }

    public static string ToCsv(Trajectory trajectory, bool retime = false) =>
        ToCsv(trajectory, retime ? new TrajectoryExportOptions { Retime = true } : null);

    public static string ToCsv(Trajectory trajectory, TrajectoryExportOptions? options)
    {
        return SerializeCsv(Prepare(trajectory, options));
    }

    private static string SerializeCsv(Trajectory traj)
    {
        if (Units.IsAerial(traj.Robot.Preset))
            return SerializeAerialCsv(traj);

        var n = traj.Robot.Preset.AxisCount;
        var stewart = Units.IsStewart(traj.Robot.Preset);
        var jointSuffix = stewart ? "_m" : "_rad";
        var hasMotionMetadata = traj.Points.Any(p => p.MotionType is not null || p.SegmentIndex is not null || p.BlendRadiusMeters is not null);
        var hasToolState = traj.Points.Any(p => p.ToolState is not null);
        var sb = new StringBuilder();
        sb.Append("time_seconds");
        for (var i = 1; i <= n; i++) sb.Append($",joint_{i}{jointSuffix}");
        if (hasMotionMetadata) sb.Append(",motion_type,segment_index,blend_radius_m");
        if (hasToolState) sb.Append(",tool_state_json");
        if (traj.AttachSpans.Count > 0) sb.Append(",attachment_spans_json");
        sb.AppendLine();
        var firstPoint = true;
        foreach (var p in traj.Points)
        {
            sb.Append(p.TimeSeconds.ToString("F6", CultureInfo.InvariantCulture));
            foreach (var j in p.JointState.Positions)
                sb.Append(',').Append(j.ToString("F6", CultureInfo.InvariantCulture));
            if (hasMotionMetadata)
            {
                sb.Append(',').Append(p.MotionType?.ToString().ToLowerInvariant() ?? string.Empty);
                sb.Append(',').Append(p.SegmentIndex?.ToString() ?? string.Empty);
                sb.Append(',').Append(p.BlendRadiusMeters?.ToString("F6", CultureInfo.InvariantCulture) ?? string.Empty);
            }
            if (hasToolState)
            {
                sb.Append(',');
                if (p.ToolState is null)
                    sb.Append(string.Empty);
                else
                    AppendJsonCell(sb, p.ToolState.Values);
            }
            if (traj.AttachSpans.Count > 0)
            {
                sb.Append(',');
                if (firstPoint) AppendJsonCell(sb, AttachmentData(traj));
            }
            sb.AppendLine();
            firstPoint = false;
        }
        return sb.ToString();
    }

    /// <summary>
    /// Aerial CSV: body SE(3) columns — never <c>joint_N_rad</c> (that would imply MoveJ).
    /// </summary>
    private static string SerializeAerialCsv(Trajectory traj)
    {
        var hasMotionMetadata = traj.Points.Any(p => p.MotionType is not null || p.SegmentIndex is not null || p.BlendRadiusMeters is not null);
        var sb = new StringBuilder();
        sb.Append("time_seconds,body_x_m,body_y_m,body_z_m,body_roll_rad,body_pitch_rad,body_yaw_rad");
        if (hasMotionMetadata) sb.Append(",motion_type,segment_index,blend_radius_m");
        sb.AppendLine();
        foreach (var p in traj.Points)
        {
            var body = ResolveAerialBodyPose(traj.Robot, p);
            sb.Append(p.TimeSeconds.ToString("F6", CultureInfo.InvariantCulture));
            if (body is null)
            {
                sb.Append(",,,,,,");
            }
            else
            {
                sb.Append(',').Append(body.Value.X.ToString("F6", CultureInfo.InvariantCulture));
                sb.Append(',').Append(body.Value.Y.ToString("F6", CultureInfo.InvariantCulture));
                sb.Append(',').Append(body.Value.Z.ToString("F6", CultureInfo.InvariantCulture));
                sb.Append(',').Append(body.Value.Roll.ToString("F6", CultureInfo.InvariantCulture));
                sb.Append(',').Append(body.Value.Pitch.ToString("F6", CultureInfo.InvariantCulture));
                sb.Append(',').Append(body.Value.Yaw.ToString("F6", CultureInfo.InvariantCulture));
            }

            if (hasMotionMetadata)
            {
                sb.Append(',').Append(p.MotionType?.ToString().ToLowerInvariant() ?? string.Empty);
                sb.Append(',').Append(p.SegmentIndex?.ToString() ?? string.Empty);
                sb.Append(',').Append(p.BlendRadiusMeters?.ToString("F6", CultureInfo.InvariantCulture) ?? string.Empty);
            }

            sb.AppendLine();
        }

        return sb.ToString();
    }

    private static (double X, double Y, double Z, double Roll, double Pitch, double Yaw)? ResolveAerialBodyPose(
        RobotModel robot,
        TrajectoryPoint point)
    {
        var frame = point.BaseFrameOverride?.Frame ?? robot.Preset.BaseFrame.Frame;
        if (!MobilityModel.HolonomicSE3.TryFromFrame(frame, out var pose, out _))
            return null;
        return (pose.X, pose.Y, pose.Z, pose.RollRadians, pose.PitchRadians, pose.YawRadians);
    }

    private static void AppendJsonCell(StringBuilder sb, object? value) =>
        sb.Append('"').Append(JsonSerializer.Serialize(value).Replace("\"", "\"\"")).Append('"');

    private static object? AttachmentData(Trajectory trajectory) => trajectory.AttachSpans.Count == 0 ? null :
        trajectory.AttachSpans.Select(span => new
        {
            startSeconds = span.StartSeconds,
            endSeconds = span.EndSeconds,
            releaseWorldPose = span.ReleaseWorldPose is { } release ? FrameData(release) : null,
            bodies = span.Bodies.Select(body => new
            {
                name = body.Name,
                sourceSceneObjectName = body.SourceSceneObjectName,
                tcpLocalPose = FrameData(body.TcpLocalPose),
                geometry = new
                {
                    name = body.Geometry.Name,
                    shape = body.Geometry.Shape.ToString().ToLowerInvariant(),
                    pose = FrameData(body.Geometry.Pose),
                    extentX = body.Geometry.ExtentX,
                    extentY = body.Geometry.ExtentY,
                    extentZ = body.Geometry.ExtentZ,
                    meshVertices = body.Geometry.MeshVertices,
                    meshIndices = body.Geometry.MeshIndices
                }
            })
        });

    private static object FrameData(Frame frame) => new
    {
        x = frame.X, y = frame.Y, z = frame.Z,
        qw = frame.Qw, qx = frame.Qx, qy = frame.Qy, qz = frame.Qz
    };

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

    private static PlannerProvenance? ResolveProvenance(TrajectoryExportOptions options)
    {
        if (options.Provenance is not null)
            return options.Provenance;
        if (!options.Retime || options.Retimer?.Algorithm != RetimerAlgorithm.Totg)
            return null;
        return new PlannerProvenance
        {
            RetimeAlgorithm = nameof(RetimerAlgorithm.Totg),
            SettingsHash = TotgMethodRefs.DescribeStack()
        };
    }

    private static bool FramesEqual(Frame a, Frame b) =>
        Math.Abs(a.X - b.X) < 1e-9 && Math.Abs(a.Y - b.Y) < 1e-9 && Math.Abs(a.Z - b.Z) < 1e-9 &&
        Math.Abs(a.Qw - b.Qw) < 1e-9 && Math.Abs(a.Qx - b.Qx) < 1e-9 &&
        Math.Abs(a.Qy - b.Qy) < 1e-9 && Math.Abs(a.Qz - b.Qz) < 1e-9;
}
