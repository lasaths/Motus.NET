using Motus.Core;

namespace Motus.Core.Tests;

public class PathConstraintsTests
{
  [Fact]
  public void PathConstraints_RejectsNamedPositionViolation()
  {
    var constraints = new PathConstraints
    {
      Name = "box",
      PositionConstraints = new[]
      {
        new PositionConstraint
        {
          Name = "tcp_box",
          Target = new Frame(0, 0, 0),
          ToleranceXMeters = 0.01,
          ToleranceYMeters = 0.01,
          ToleranceZMeters = 0.01
        }
      }
    };

    Assert.False(constraints.TryValidate(new Frame(0.02, 0, 0), out var reason));
    Assert.Contains("ConstraintViolation", reason);
    Assert.Contains("tcp_box", reason);
    Assert.Contains("meters", reason);
  }

  [Fact]
  public void OrientationConstraint_UsesRadiansTolerance()
  {
    var constraint = new OrientationConstraint
    {
      Name = "tcp_orientation",
      Target = Frame.Identity,
      AbsoluteXAxisToleranceRadians = 0.1,
      AbsoluteYAxisToleranceRadians = 0.1,
      AbsoluteZAxisToleranceRadians = 0.1
    };

    var halfAngle = 0.08;
    var tcp = new Frame(0, 0, 0, Math.Cos(halfAngle), 0, 0, Math.Sin(halfAngle));
    Assert.False(constraint.TryValidate(tcp, out var reason));
    Assert.Contains("radians", reason);
  }
}
