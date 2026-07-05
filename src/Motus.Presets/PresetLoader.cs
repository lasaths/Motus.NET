using System.Text.Json;
using System.Text.Json.Serialization;
using Motus.Core;

namespace Motus.Presets;

public static class PresetLoader
{
    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNameCaseInsensitive = true,
        ReadCommentHandling = JsonCommentHandling.Skip,
        AllowTrailingCommas = true
    };

  public static string DefaultResourcesPath =>
        Path.Combine(ResolvePluginRoot(), "resources", "robots");

    private static string ResolvePluginRoot()
    {
        var assemblyPath = typeof(PresetLoader).Assembly.Location;
        if (!string.IsNullOrEmpty(assemblyPath))
        {
            var dir = Path.GetDirectoryName(assemblyPath);
            if (!string.IsNullOrEmpty(dir))
                return dir;
        }
        return AppContext.BaseDirectory;
    }

    public static RobotPreset LoadFromFile(string path) => LoadRobotModelFromFile(path).Preset;

    public static RobotModel LoadRobotModelFromFile(string path)
    {
        var json = File.ReadAllText(path);
        return LoadRobotModelFromJson(json, path);
    }

    public static RobotPreset LoadFromJson(string json) =>
        LoadRobotModelFromJson(json, null).Preset;

    public static RobotModel LoadRobotModelFromJson(string json, string? sourcePath = null)
    {
        var dto = JsonSerializer.Deserialize<PresetDto>(json, JsonOptions)
            ?? throw new InvalidOperationException("Empty preset JSON.");
        var preset = dto.ToPreset();
        var collision = sourcePath is not null
            ? CollisionPresetLoader.LoadFromDto(dto.CollisionLinks, sourcePath)
            : null;
        return new RobotModel(preset, collision, BundledJointNames.TryGet(preset.ModelName));
    }

    public static RobotPreset LoadByModelName(string modelName, string? resourcesRoot = null) =>
        LoadRobotModelByName(modelName, resourcesRoot).Preset;

    public static RobotModel LoadRobotModelByName(string modelName, string? resourcesRoot = null)
    {
        var root = resourcesRoot ?? DefaultResourcesPath;
        foreach (var file in Directory.EnumerateFiles(root, "*.json", SearchOption.AllDirectories))
        {
            var model = LoadRobotModelFromFile(file);
            if (string.Equals(model.Preset.ModelName, modelName, StringComparison.OrdinalIgnoreCase))
                return model;
        }
        throw new FileNotFoundException($"No preset found for model '{modelName}' under {root}.");
    }

    public static IReadOnlyList<string> ListAvailableModels(string? resourcesRoot = null)
    {
        var root = resourcesRoot ?? DefaultResourcesPath;
        if (!Directory.Exists(root)) return Array.Empty<string>();
        return Directory.EnumerateFiles(root, "*.json", SearchOption.AllDirectories)
            .Select(f => LoadFromFile(f).ModelName)
            .OrderBy(n => n)
            .ToList();
    }

    private sealed class PresetDto
    {
        public string Manufacturer { get; set; } = "";
        public string ModelName { get; set; } = "";
        public string Family { get; set; } = "";
        public int AxisCount { get; set; }
        public List<JointLimitDto> JointLimits { get; set; } = new();
        public double? ReachMeters { get; set; }
        public double? PayloadKg { get; set; }
        public FrameDto? BaseFrame { get; set; }
        [JsonPropertyName("toolFrame")]
        public ToolFrameDto? ToolFrameData { get; set; }
        public string? Notes { get; set; }
        public string? SourceNote { get; set; }
        public string? Disclaimer { get; set; }
        public List<CollisionPresetLoader.CollisionLinkDto>? CollisionLinks { get; set; }

        public RobotPreset ToPreset()
        {
            if (AxisCount <= 0) throw new InvalidOperationException("axisCount must be positive.");
            if (JointLimits.Count != AxisCount)
                throw new InvalidOperationException($"Expected {AxisCount} joint limits, got {JointLimits.Count}.");

            if (!Enum.TryParse<RobotManufacturer>(Manufacturer, true, out var mfg))
                throw new InvalidOperationException($"Unknown manufacturer '{Manufacturer}'.");

            var limits = JointLimits.Select(j => new JointLimit(
                j.MinRadians, j.MaxRadians,
                j.MaxVelocityRadiansPerSecond,
                j.MaxAccelerationRadiansPerSecondSquared)).ToList();

            var baseF = BaseFrame?.ToFrame() ?? Frame.Identity;
            var toolF = ToolFrameData?.ToToolFrame() ?? ToolFrame.Identity;

            return new RobotPreset
            {
                Manufacturer = mfg,
                ModelName = ModelName,
                Family = Family,
                AxisCount = AxisCount,
                JointLimits = limits,
                ReachMeters = ReachMeters,
                PayloadKg = PayloadKg,
                BaseFrame = new BaseFrame(baseF),
                ToolFrame = toolF,
                Notes = Notes,
                SourceNote = SourceNote,
                Disclaimer = Disclaimer ?? "Preset values are planning/visualization defaults only, not physical compatibility guarantees."
            };
        }
    }

    private sealed class JointLimitDto
    {
        public double MinRadians { get; set; }
        public double MaxRadians { get; set; }
        public double? MaxVelocityRadiansPerSecond { get; set; }
        public double? MaxAccelerationRadiansPerSecondSquared { get; set; }
    }

    private class FrameDto
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double Qw { get; set; } = 1;
        public double Qx { get; set; }
        public double Qy { get; set; }
        public double Qz { get; set; }
        public Frame ToFrame() => new(X, Y, Z, Qw, Qx, Qy, Qz);
    }

    private sealed class ToolFrameDto : FrameDto
    {
        public string? Name { get; set; }
        public ToolFrame ToToolFrame() => new(ToFrame(), Name ?? "flange");
    }
}
