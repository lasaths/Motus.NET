using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class AllowedCollisionTests
{
  private static string ResourcesRoot =>
    Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "resources", "robots"));

  [Fact]
  public void AllowedPair_SkipsRobotLinkVsObject()
  {
    var preset = PresetLoader.LoadByModelName("UR5e", ResourcesRoot);
    var checker = new SphereCollisionChecker(preset);
    var start = new JointState(new double[6]);
    var table = CollisionObject.Box("table", new Frame(0.3, 0, 0.2), 0.5, 0.5, 0.05);
    var blocking = new CollisionScene(new[] { table });
    Assert.False(checker.IsCollisionFree(start, blocking));

    var pairs = Enumerable.Range(0, 6).Select(i => (CollisionBodies.RobotLink(i), "table")).ToList();
    var allowed = new CollisionScene(new[] { table }, pairs);
    Assert.True(checker.IsCollisionFree(start, allowed));
  }
}
