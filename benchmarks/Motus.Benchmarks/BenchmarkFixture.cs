using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Benchmarks;

internal static class BenchmarkFixture
{
    internal static string ResourcesRoot => FindResourcesRoot();

    internal static RobotPreset LoadUr5e() => PresetLoader.LoadByModelName("UR5e", ResourcesRoot);

    internal static JointState Home { get; } = new(new[] { 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0 });

    internal static JointState Goal { get; } = new(new[] { 0.5, -1.5, 2.0, -1.0, -1.0, 0.5 });

    internal static CartesianPose LinGoal(DhForwardKinematics fk, RobotPreset preset)
    {
        var startTcp = fk.ComputeTcp(Home, preset.BaseFrame, preset.ToolFrame);
        return new CartesianPose(new Frame(
            0.8, 0.5, 0.9,
            startTcp.Tcp.Qw, startTcp.Tcp.Qx, startTcp.Tcp.Qy, startTcp.Tcp.Qz));
    }

    private static string FindResourcesRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null)
        {
            var candidate = Path.Combine(dir.FullName, "resources", "robots");
            if (Directory.Exists(candidate)) return candidate;
            dir = dir.Parent;
        }
        throw new DirectoryNotFoundException($"Could not locate resources/robots from {AppContext.BaseDirectory}");
    }
}
