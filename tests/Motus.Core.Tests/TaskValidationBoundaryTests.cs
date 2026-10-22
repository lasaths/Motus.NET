using Motus.Core;

namespace Motus.Core.Tests;

public class TaskValidationBoundaryTests
{
    private static RobotModel Robot(double min = -2, double max = 2) => new(new RobotPreset
    {
        AxisCount = 1,
        JointLimits = new[] { new JointLimit(min, max) }
    });

    private static TaskPath Path(params double[] positions) => new(new TaskInstance("move",
        positions.Select((x, i) => (x, i)).ToDictionary(p => p.i.ToString(), p => new Frame(p.x, 0, 0))));

    private sealed class Ik : IInverseKinematics
    {
        public int Calls { get; private set; }
        public JointState? LastSeed { get; private set; }
        public JointState? FixedSolution { get; init; }
        public bool ReturnNull { get; init; }
        public bool TrySolve(CartesianPose target, JointState seed, out JointState solution)
        {
            Calls++;
            LastSeed = seed;
            solution = ReturnNull ? null! : FixedSolution ?? new JointState(new[] { target.Tcp.X });
            return true;
        }
    }

    private sealed class Checker : ICollisionChecker
    {
        public int Calls { get; private set; }
        public bool BlockMiddle { get; init; }
        public bool IsCollisionFree(JointState state, CollisionScene scene)
        {
            Calls++;
            return !BlockMiddle || state.Positions[0] < 0.4 || state.Positions[0] > 0.6;
        }
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void TaskFramesRejectNonFinitePositionAndQuaternion(double invalid)
    {
        Assert.Throws<ArgumentException>(() => new TaskInstance("move", new Dictionary<string, Frame> { ["target"] = new(invalid, 0, 0) }));
        Assert.Throws<ArgumentException>(() => new TaskInstance("move", new Dictionary<string, Frame> { ["target"] = new(0, 0, 0, invalid) }));
    }

    [Fact]
    public void TaskFramesRejectZeroQuaternionAndMissingNames()
    {
        Assert.Throws<ArgumentException>(() => new TaskInstance("move", new Dictionary<string, Frame> { ["target"] = default }));
        Assert.Throws<ArgumentException>(() => new TaskInstance("move", new Dictionary<string, Frame> { [" "] = Frame.Identity }));
        Assert.Throws<ArgumentNullException>(() => new TaskInstance("move", null!));
        Assert.Throws<ArgumentException>(() => new TaskPath(new TaskInstance[] { null! }));
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void DifferingFrameOverridesFailBeforeSolvingOrChecking(bool changeBase)
    {
        var ik = new Ik(); var checker = new Checker();
        var result = TaskPathValidator.Validate(Path(0), Robot(), ik, checker, new CollisionScene(),
            baseFrame: changeBase ? new BaseFrame(new Frame(1, 0, 0)) : null,
            toolFrame: changeBase ? null : new ToolFrame(new Frame(0, 0, 0.1)));
        Assert.False(result.IsValid);
        Assert.Contains(result.Errors, e => e.Contains("Unsupported base/tool frame override"));
        Assert.Equal(0, ik.Calls);
        Assert.Equal(0, checker.Calls);
    }

    [Fact]
    public void MatchingFrameOverridesAreAccepted()
    {
        var robot = Robot();
        var result = TaskPathValidator.Validate(Path(0), robot, new Ik(), new Checker(), new CollisionScene(),
            new BaseFrame(robot.Preset.BaseFrame.Frame), new ToolFrame(robot.Preset.ToolFrame.Frame));
        Assert.True(result.IsValid);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(3.0)]
    public void InvalidSeedsAreRejectedBeforeIk(double value)
    {
        var ik = new Ik();
        var result = TaskPathValidator.Validate(Path(0), Robot(), ik, new Checker(), new CollisionScene(), seed: new JointState(new[] { value }));
        Assert.False(result.IsValid);
        Assert.Contains(result.Errors, e => e.Contains("seed"));
        Assert.Equal(0, ik.Calls);
    }

    [Fact]
    public void MissingSeedUsesAnInBoundsCoordinateForPrismaticLimits()
    {
        var ik = new Ik();
        var result = TaskPathValidator.Validate(Path(1), Robot(1, 2), ik, new Checker(), new CollisionScene());
        Assert.True(result.IsValid);
        Assert.Equal(1, ik.LastSeed!.Positions[0]);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(0.0)]
    [InlineData(-0.01)]
    public void InvalidMotionStepIsRejectedBeforeInvokingSolvers(double step)
    {
        var ik = new Ik(); var checker = new Checker();
        var result = TaskPathValidator.ValidateWithMotion(Path(0, 1), Robot(), ik, checker, new CollisionScene(), step);
        Assert.False(result.IsValid);
        Assert.Contains(result.Errors, e => e.Contains("configurationStep"));
        Assert.Equal(0, ik.Calls);
        Assert.Equal(0, checker.Calls);
    }

    [Fact]
    public void InvalidIkResultsNeverReachCollisionChecking()
    {
        foreach (var ik in new[] { new Ik { FixedSolution = new JointState(new[] { double.NaN }) }, new Ik { ReturnNull = true }, new Ik { FixedSolution = new JointState(Array.Empty<double>()) } })
        {
            var checker = new Checker();
            var result = TaskPathValidator.Validate(Path(0), Robot(), ik, checker, new CollisionScene());
            Assert.False(result.IsValid);
            Assert.Empty(result.ReachableStates);
            Assert.Equal(0, checker.Calls);
        }
    }

    [Fact]
    public void MotionValidationChecksInteriorWhenEndpointsAreClear()
    {
        var checker = new Checker { BlockMiddle = true };
        Assert.True(TaskPathValidator.Validate(Path(0, 1), Robot(), new Ik(), checker, new CollisionScene()).IsValid);
        var motion = TaskPathValidator.ValidateWithMotion(Path(0, 1), Robot(), new Ik(), checker, new CollisionScene(), 0.05);
        Assert.False(motion.IsValid);
        Assert.Contains(motion.Errors, e => e.Contains("Segment collision"));
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(-1.0)]
    public void TypedTaskFactoriesRejectInvalidJawOpening(double invalid)
    {
        Assert.Throws<ArgumentException>(() => CoreTasks.Pick("beam", invalid));
        Assert.Throws<ArgumentException>(() => CoreTasks.Place("beam", invalid));
        Assert.Throws<ArgumentException>(() => CoreTasks.Park("beam", "node", "up", invalid));
    }

    [Fact]
    public void MalformedTaskMetadataProducesSequenceProblems()
    {
        var pick = new TaskInstance("pick", new Dictionary<string, Frame>(), new Dictionary<string, object> { [TaskMetadataKeys.Object] = "beam", [TaskMetadataKeys.JawMm] = "not a number" });
        var walk = CoreTasks.Walk("beam").WithMetadataValue(TaskMetadataKeys.Mode, "99");
        Assert.Throws<ArgumentException>(() => CoreTasks.AsWalk(walk));
        var problems = TaskSequenceRules.Validate(new[] { pick, walk });
        Assert.Equal(2, problems.Count);
        Assert.All(problems, p => Assert.Contains("invalid", p.Message));
        Assert.Throws<ArgumentException>(() => CoreTasks.Pick(" ", 0));
        Assert.Throws<ArgumentException>(() => CoreTasks.Walk("beam", (WalkMode)99));
    }
}
