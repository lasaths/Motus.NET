using System.Text.Json;
using Motus.Presets;

namespace Motus.Core.Tests;

public sealed class PlanBundleContractTests
{
    private static readonly JsonSerializerOptions CanonicalJson = new() { WriteIndented = false };

    private static string FixturePath(string name) =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", "exports", name));

    private static RobotModel CreateRobot() =>
        new(
            new RobotPreset
            {
                Manufacturer = RobotManufacturer.UniversalRobots,
                ModelName = "ContractFixtureBot",
                AxisCount = 2,
                JointLimits = new[] { new JointLimit(-6.28, 6.28), new JointLimit(-6.28, 6.28) }
            },
            jointNames: new[] { "joint_a", "joint_b" });

    [Theory]
    [InlineData("joint_plan.json", "joint_plan.csv", "joint-linear")]
    [InlineData("cartesian_plan.json", "cartesian_plan.csv", "cartesian-lin")]
    [InlineData("collision_sampling_plan.json", "collision_sampling_plan.csv", "sampling-rrtconnect")]
    [InlineData("motion_program_plan.json", "motion_program_plan.csv", "industrial-motion-program")]
    public void Export_MatchesGoldenContractFixtures(string jsonFixture, string csvFixture, string plannerId)
    {
        var robot = CreateRobot();
        var points = BuildScenarioPoints(plannerId);
        var traj = new Trajectory(robot, points);
        var options = new TrajectoryExportOptions
        {
            Provenance = new PlannerProvenance
            {
                PlannerId = plannerId,
                RandomSeed = plannerId == "sampling-rrtconnect" ? 42 : null,
                SettingsHash = $"fixture-{plannerId}",
                RetimeAlgorithm = "totg-lite"
            },
            Diagnostics = new[]
            {
                new PlanningMessage(PlanningMessageCodes.PlannerWarning, $"fixture warning for {plannerId}", PlanningMessageSeverity.Warning)
            }
        };

        var json = TrajectoryExport.ToJson(traj, options);
        var csv = TrajectoryExport.ToCsv(traj, options);

        Assert.Equal(ReadCanonicalJson(FixturePath(jsonFixture)), ReadCanonicalJsonText(json));
        Assert.Equal(File.ReadAllText(FixturePath(csvFixture)).Replace("\r\n", "\n"), csv.Replace("\r\n", "\n"));
    }

    private static IReadOnlyList<TrajectoryPoint> BuildScenarioPoints(string plannerId) =>
        plannerId switch
        {
            "joint-linear" => new[]
            {
                new TrajectoryPoint(0.0, new JointState(new[] { 0.0, 0.0 })),
                new TrajectoryPoint(0.2, new JointState(new[] { 0.25, 0.1 }))
            },
            "cartesian-lin" => new[]
            {
                new TrajectoryPoint(0.0, new JointState(new[] { 0.1, -0.2 }), MotionPrimitiveType.Lin, segmentIndex: 0),
                new TrajectoryPoint(0.3, new JointState(new[] { 0.2, -0.1 }), MotionPrimitiveType.Lin, segmentIndex: 0)
            },
            "sampling-rrtconnect" => new[]
            {
                new TrajectoryPoint(0.0, new JointState(new[] { 0.0, 0.0 })),
                new TrajectoryPoint(0.15, new JointState(new[] { 0.18, 0.04 })),
                new TrajectoryPoint(0.3, new JointState(new[] { 0.35, 0.12 }))
            },
            "industrial-motion-program" => new[]
            {
                new TrajectoryPoint(0.0, new JointState(new[] { 0.0, 0.0 }), MotionPrimitiveType.Ptp, segmentIndex: 0, blendRadiusMeters: 0.002),
                new TrajectoryPoint(
                    0.25,
                    new JointState(new[] { 0.2, 0.1 }),
                    MotionPrimitiveType.Lin,
                    segmentIndex: 1,
                    blendRadiusMeters: 0.001,
                    toolState: new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.04 }))
            },
            _ => throw new InvalidOperationException($"Unknown planner fixture '{plannerId}'.")
        };

    private static string ReadCanonicalJson(string path) =>
        JsonSerializer.Serialize(JsonDocument.Parse(File.ReadAllText(path)).RootElement, CanonicalJson);

    private static string ReadCanonicalJsonText(string json) =>
        JsonSerializer.Serialize(JsonDocument.Parse(json).RootElement, CanonicalJson);
}
