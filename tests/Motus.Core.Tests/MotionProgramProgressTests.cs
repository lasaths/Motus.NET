using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class MotionProgramProgressTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Progress_ReportsEverySegment_AndOnlyReportsDoneOnSuccess(bool duplicateAttach)
    {
        var robot = new RobotModel(PresetLoader.LoadByModelName("UR5e"));
        var start = new JointState(new[] { 0d, -Math.PI / 2, Math.PI / 2, 0, Math.PI / 2, 0 });
        var body = CollisionObject.Box("brick", Frame.Identity, 0.02, 0.01, 0.01);
        var segments = new MotionSegment[]
        {
            new WaitSegment(0.1),
            new SetToolStateSegment(new EndEffectorState(new Dictionary<string, double> { ["width"] = 0.04 })),
            new AttachSegment("brick", Frame.Identity, body),
            duplicateAttach ? new AttachSegment("brick", Frame.Identity, body) : new DetachSegment("brick", Frame.Identity),
            new WaitSegment(0.1)
        };
        var updates = new List<MotionProgramProgress>();
        var result = new IndustrialMotionPlanner(robot.Preset).Plan(new MotionProgramRequest(robot, start, segments)
        {
            ReportProgress = updates.Add
        });
        Assert.Equal(!duplicateAttach, result.Success);
        Assert.All(updates, p => Assert.Equal(segments.Length, p.TotalSegments));
        Assert.Equal("Checking start", updates[0].Phase);
        var started = updates.Where(p => p.CompletedSegments < segments.Length && p.Phase != "Checking start").ToArray();
        Assert.Equal(duplicateAttach ? 4 : 5, started.Length);
        for (var i = 0; i < started.Length; i++)
        {
            Assert.Equal(i, started[i].CompletedSegments);
            Assert.Equal(segments[i].Type.ToString(), started[i].Phase);
        }
        if (duplicateAttach)
            Assert.DoesNotContain(updates, p => p.Phase == "Done");
        else
        {
            Assert.Equal(new MotionProgramProgress(5, 5, "Timing and tool checks"), updates[^2]);
            Assert.Equal(new MotionProgramProgress(5, 5, "Done"), updates[^1]);
        }
    }
}
