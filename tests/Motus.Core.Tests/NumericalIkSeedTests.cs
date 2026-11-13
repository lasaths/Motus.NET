using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>IK must converge from seeds that are near, but not at, the answer (G0).</summary>
public class NumericalIkSeedTests
{
    private static string Fixture(string name) =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", name));

    private static string Resource(string name) =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots", name));

    public static TheoryData<string> Robots => new()
    {
        "ur10e", "rail_ur10e", "panda", "kr210", "KUKA/KR_10_R1100", "KUKA/LBR_iiwa_7_R800"
    };

    private static (RobotPreset Preset, SerialJointChain? Chain) Load(string id) => id switch
    {
        "ur10e" => Urdf(UrdfRobotLoader.Load(Fixture("ur10e/ur10e.urdf"), new UrdfLoadOptions { ModelName = "UR10e" })),
        "rail_ur10e" => Urdf(UrdfRobotLoader.Load(Fixture("ur10e/rail_ur10e.urdf"), new UrdfLoadOptions { ModelName = "rail_ur10e" })),
        "panda" => Urdf(UrdfRobotLoader.Load(Fixture("panda/panda_minimal.urdf"),
            new UrdfLoadOptions { BaseLink = "panda_link0", TipLink = "panda_hand_tcp", ModelName = "Panda" })),
        "kr210" => Urdf(UrdfRobotLoader.Load(Fixture("kr210_r3100_ultra/kr210_r3100_ultra_minimal.urdf"),
            new UrdfLoadOptions { ModelName = "KR 210 R3100 ultra" })),
        _ => (PresetLoader.LoadFromFile(Resource(id + ".json")), null)
    };

    private static (RobotPreset, SerialJointChain?) Urdf(UrdfRobot r) => (r.Preset, r.Chain);

    [Theory]
    [MemberData(nameof(Robots))]
    public void OffAnswerSeeds_Converge_AndFkMatchesTarget(string id)
    {
        var (preset, chain) = Load(id);
        var ik = KinematicsResolver.CreateInverseKinematics(preset, chain);
        var fk = KinematicsResolver.CreateFkSolver(preset, chain);
        var lim = preset.JointLimits;
        var rng = new Random(3);
        var ok = 0;
        const int trials = 40;
        for (var t = 0; t < trials; t++)
        {
            var q = lim.Select(l =>
            {
                var lo = Math.Max(l.MinRadians, -2.5);
                var hi = Math.Min(l.MaxRadians, 2.5);
                return lo + (0.2 + 0.6 * rng.NextDouble()) * (hi - lo);
            }).ToArray();
            var goal = fk.ComputeTcpTransform(q, preset.BaseFrame.Frame, preset.ToolFrame.Frame);
            var seed = q.Select((v, i) => Math.Clamp(v + 0.2 * (rng.NextDouble() * 2 - 1), lim[i].MinRadians, lim[i].MaxRadians)).ToArray();
            if (!ik.TrySolve(new CartesianPose(Transforms.ToFrame(goal)), new JointState(seed), out var sol))
                continue;
            var reached = fk.ComputeTcpTransform(sol.Positions, preset.BaseFrame.Frame, preset.ToolFrame.Frame);
            // Default NumericalIkOptions contract: 1 mm, 0.01 rad.
            if (Transforms.TcpMatches(reached, Transforms.ToFrame(goal), 1e-3, 1e-2))
                ok++;
        }
        Assert.True(ok >= 38, $"{id}: {ok}/{trials} off-answer seeds converged");
    }

    [Fact]
    public void GoalOutOfReach_FailsWithNamedReason()
    {
        var preset = PresetLoader.LoadFromFile(Resource("KUKA/KR_10_R1100.json"));
        var ik = new NumericalInverseKinematics(preset);
        var r = ik.TrySolveDetailed(new CartesianPose(new Frame(50, 0, 0)), new JointState(new double[preset.JointLimits.Count]));
        Assert.False(r.Success);
        Assert.Equal(NumericalIkFailureReasons.NoConvergence, r.FailureReason);
    }
}
