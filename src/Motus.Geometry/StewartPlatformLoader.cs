using System.Text.Json;
using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Versioned Stewart platform JSON loader. Rejects non-finite values, wrong leg counts, and oversized payloads.
/// </summary>
public static class StewartPlatformLoader
{
    public const int SchemaVersion = 1;
    public const int MaxJsonBytes = 256 * 1024;

    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNameCaseInsensitive = true,
        ReadCommentHandling = JsonCommentHandling.Skip,
        AllowTrailingCommas = true
    };

    public static StewartPlatform LoadFile(string path)
    {
        if (string.IsNullOrWhiteSpace(path))
            throw new ArgumentException("Path is required.", nameof(path));
        if (!File.Exists(path))
            throw new FileNotFoundException("Stewart platform file not found.", path);
        var info = new FileInfo(path);
        if (info.Length > MaxJsonBytes)
            throw new InvalidDataException($"Stewart JSON exceeds {MaxJsonBytes} bytes.");
        var json = File.ReadAllText(path);
        return LoadJson(json);
    }

    public static StewartPlatform LoadJson(string json)
    {
        if (string.IsNullOrWhiteSpace(json))
            throw new ArgumentException("JSON is required.", nameof(json));
        if (json.Length > MaxJsonBytes)
            throw new InvalidDataException($"Stewart JSON exceeds {MaxJsonBytes} characters.");

        StewartDto? dto;
        try
        {
            dto = JsonSerializer.Deserialize<StewartDto>(json, JsonOptions);
        }
        catch (JsonException ex)
        {
            throw new InvalidDataException($"Stewart JSON parse failed: {ex.Message}", ex);
        }

        if (dto is null)
            throw new InvalidDataException("Stewart JSON deserialized to null.");
        if (dto.SchemaVersion != SchemaVersion)
            throw new InvalidDataException($"Unsupported Stewart schemaVersion {dto.SchemaVersion}; expected {SchemaVersion}.");
        if (string.IsNullOrWhiteSpace(dto.ModelName))
            throw new InvalidDataException("modelName is required.");
        if (dto.BaseAnchors is null || dto.BaseAnchors.Length != StewartPlatform.LegCount)
            throw new InvalidDataException($"baseAnchors must have {StewartPlatform.LegCount} entries.");
        if (dto.PlatformAnchors is null || dto.PlatformAnchors.Length != StewartPlatform.LegCount)
            throw new InvalidDataException($"platformAnchors must have {StewartPlatform.LegCount} entries.");

        var baseAnchors = dto.BaseAnchors.Select(ToVec3).ToArray();
        var platformAnchors = dto.PlatformAnchors.Select(ToVec3).ToArray();
        JointLimit[] limits;
        if (dto.StrokeMinMeters is not null && dto.StrokeMaxMeters is not null)
        {
            if (!double.IsFinite(dto.StrokeMinMeters.Value) || !double.IsFinite(dto.StrokeMaxMeters.Value))
                throw new InvalidDataException("strokeMinMeters/strokeMaxMeters must be finite.");
            limits = Enumerable.Range(0, StewartPlatform.LegCount)
                .Select(_ => JointLimit.Meters(dto.StrokeMinMeters.Value, dto.StrokeMaxMeters.Value))
                .ToArray();
        }
        else if (dto.StrokeLimitsMeters is { Length: StewartPlatform.LegCount })
        {
            limits = new JointLimit[StewartPlatform.LegCount];
            for (var i = 0; i < StewartPlatform.LegCount; i++)
            {
                var s = dto.StrokeLimitsMeters[i];
                if (s is null || !double.IsFinite(s.Min) || !double.IsFinite(s.Max))
                    throw new InvalidDataException($"strokeLimitsMeters[{i}] must be finite min/max.");
                limits[i] = JointLimit.Meters(s.Min, s.Max);
            }
        }
        else
            throw new InvalidDataException("Provide strokeMinMeters/strokeMaxMeters or strokeLimitsMeters[6].");

        Frame? tool = null;
        if (dto.ToolOffset is not null)
        {
            var t = dto.ToolOffset;
            if (!AllFinite(t.X, t.Y, t.Z, t.Qw, t.Qx, t.Qy, t.Qz))
                throw new InvalidDataException("toolOffset must be finite.");
            tool = new Frame(t.X, t.Y, t.Z, t.Qw, t.Qx, t.Qy, t.Qz);
        }

        return new StewartPlatform(dto.ModelName, baseAnchors, platformAnchors, limits, tool);
    }

    private static Vec3 ToVec3(Vec3Dto? v)
    {
        if (v is null || !AllFinite(v.X, v.Y, v.Z))
            throw new InvalidDataException("Anchor must be a finite [x,y,z].");
        return new Vec3(v.X, v.Y, v.Z);
    }

    private static bool AllFinite(params double[] values)
    {
        foreach (var v in values)
            if (!double.IsFinite(v)) return false;
        return true;
    }

    private sealed class StewartDto
    {
        public int SchemaVersion { get; set; }
        public string? ModelName { get; set; }
        public Vec3Dto[]? BaseAnchors { get; set; }
        public Vec3Dto[]? PlatformAnchors { get; set; }
        public double? StrokeMinMeters { get; set; }
        public double? StrokeMaxMeters { get; set; }
        public StrokeDto[]? StrokeLimitsMeters { get; set; }
        public FrameDto? ToolOffset { get; set; }
    }

    private sealed class Vec3Dto
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
    }

    private sealed class StrokeDto
    {
        public double Min { get; set; }
        public double Max { get; set; }
    }

    private sealed class FrameDto
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double Qw { get; set; } = 1;
        public double Qx { get; set; }
        public double Qy { get; set; }
        public double Qz { get; set; }
    }
}
