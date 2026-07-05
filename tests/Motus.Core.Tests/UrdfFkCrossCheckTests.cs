namespace Motus.Core.Tests;

public class UrdfFkCrossCheckTests
{
  private static string RepoRoot =>
    Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", ".."));

  [Fact]
  public void UrdfFk_MatchesUrdfLoader_Reference()
  {
    if (!NodeAvailable())
    {
      Assert.Fail("Node.js is required for URDF FK cross-check. Install Node 22+ and run: npm ci --prefix tools/urdf-viewer");
    }

    var report = ViewerReportGenerator.Build(RepoRoot);
    ViewerReportGenerator.Write(RepoRoot, report);

    Assert.Equal(0, report.Summary.Failed);
    Assert.Equal(0, report.Summary.PlanningFailed);
    Assert.True(report.Summary.Total > 0);
    Assert.True(report.Summary.ViewerCases >= 12, "Expected multi-point FK suites for viewer fixtures");
    Assert.True(report.Summary.PlanningScenarios >= 2, "Expected RRT planning scenarios for viewer fixtures");
  }

  private static bool NodeAvailable()
  {
    try
    {
      using var p = System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo("node", "--version")
      {
        RedirectStandardOutput = true,
        UseShellExecute = false,
        CreateNoWindow = true,
      });
      return p?.WaitForExit(5000) == true && p.ExitCode == 0;
    }
    catch
    {
      return false;
    }
  }
}
