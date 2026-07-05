using System.Diagnostics;
using System.Text.Json;
using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class UrdfFkCrossCheckTests
{
  private static readonly JsonSerializerOptions JsonOptions = new() { PropertyNameCaseInsensitive = true };
  private static readonly JsonSerializerOptions NodeJsonOptions = new()
  {
    PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
    PropertyNameCaseInsensitive = true
  };

  private static string RepoRoot =>
    Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", ".."));

  private static string FixturesDir =>
    Path.Combine(RepoRoot, "tests", "fixtures");

  private static string VerifyScript =>
    Path.Combine(RepoRoot, "tools", "urdf-fk-verify.mjs");

  [Fact]
  public void UrdfFk_MatchesUrdfLoader_Reference()
  {
    if (!NodeAvailable())
    {
      Assert.Fail("Node.js is required for URDF FK cross-check. Install Node 22+ and run: npm ci --prefix tools/urdf-viewer");
    }

    if (!File.Exists(VerifyScript))
      Assert.Fail($"Missing verify script: {VerifyScript}");

    var casesPath = Path.Combine(FixturesDir, "fk_cases.json");
    var cases = JsonSerializer.Deserialize<List<FkCase>>(File.ReadAllText(casesPath), JsonOptions)
      ?? throw new InvalidOperationException("Failed to load fk_cases.json");

    foreach (var group in cases.GroupBy(c => c.UrdfFile))
    {
      var first = group.First();
      var urdfPath = Path.Combine("tests", "fixtures", first.UrdfFile).Replace('\\', '/');
      var motusById = new Dictionary<string, Frame>();

      foreach (var c in group)
      {
        var robot = UrdfRobotLoader.Load(Path.Combine(FixturesDir, c.UrdfFile), new UrdfLoadOptions
        {
          BaseLink = c.BaseLink,
          TipLink = c.TipLink
        });
        var positions = robot.JointNames.Select(name => c.Joints[name]).ToArray();
        Assert.Equal(robot.JointNames.Count, positions.Length);

        var fk = new SerialForwardKinematics(robot.Chain);
        var pose = fk.ComputeTcp(new JointState(positions), robot.Preset.BaseFrame, robot.Preset.ToolFrame);
        motusById[c.Id] = pose.Tcp;
      }

      var request = new VerifyRequest
      {
        RepoRoot = RepoRoot,
        UrdfPath = urdfPath,
        TipLink = first.TipLink,
        Cases = group.Select(c => new VerifyCase { Id = c.Id, Joints = c.Joints }).ToList()
      };

      var response = RunVerifyScript(request);
      Assert.Equal(group.Count(), response.Results.Count);

      foreach (var result in response.Results)
      {
        Assert.NotNull(result.Id);
        var motus = motusById[result.Id!];
        var posErr = Math.Sqrt(
          Math.Pow(result.Position.X - motus.X, 2) +
          Math.Pow(result.Position.Y - motus.Y, 2) +
          Math.Pow(result.Position.Z - motus.Z, 2));
        Assert.True(posErr < 0.001, $"{result.Id}: position error {posErr:F4}m");

        var dot = Math.Abs(
          result.Quaternion.W * motus.Qw +
          result.Quaternion.X * motus.Qx +
          result.Quaternion.Y * motus.Qy +
          result.Quaternion.Z * motus.Qz);
        dot = Math.Min(1.0, dot);
        var angleErr = 2 * Math.Acos(dot);
        Assert.True(angleErr < 0.01, $"{result.Id}: orientation error {angleErr:F4} rad");
      }
    }
  }

  private static bool NodeAvailable()
  {
    try
    {
      using var p = Process.Start(new ProcessStartInfo("node", "--version")
      {
        RedirectStandardOutput = true,
        UseShellExecute = false,
        CreateNoWindow = true
      });
      return p?.WaitForExit(5000) == true && p.ExitCode == 0;
    }
    catch
    {
      return false;
    }
  }

  private static VerifyResponse RunVerifyScript(VerifyRequest request)
  {
    var input = JsonSerializer.Serialize(request, NodeJsonOptions);
    var psi = new ProcessStartInfo("node", VerifyScript)
    {
      RedirectStandardInput = true,
      RedirectStandardOutput = true,
      RedirectStandardError = true,
      UseShellExecute = false,
      CreateNoWindow = true,
      WorkingDirectory = RepoRoot
    };

    using var process = Process.Start(psi) ?? throw new InvalidOperationException("Failed to start node verify script.");
    process.StandardInput.Write(input);
    process.StandardInput.Close();
    var stdout = process.StandardOutput.ReadToEnd();
    var stderr = process.StandardError.ReadToEnd();
    process.WaitForExit(30_000);

    if (process.ExitCode != 0)
      throw new InvalidOperationException($"urdf-fk-verify failed (exit {process.ExitCode}): {stderr}");

    return JsonSerializer.Deserialize<VerifyResponse>(stdout, JsonOptions)
      ?? throw new InvalidOperationException("Invalid verify script output.");
  }

  private sealed class FkCase
  {
    public string Id { get; set; } = "";
    public string UrdfFile { get; set; } = "";
    public string BaseLink { get; set; } = "";
    public string TipLink { get; set; } = "";
    public Dictionary<string, double> Joints { get; set; } = new();
  }

  private sealed class VerifyRequest
  {
    public string RepoRoot { get; set; } = "";
    public string UrdfPath { get; set; } = "";
    public string TipLink { get; set; } = "";
    public List<VerifyCase> Cases { get; set; } = new();
  }

  private sealed class VerifyCase
  {
    public string Id { get; set; } = "";
    public Dictionary<string, double> Joints { get; set; } = new();
  }

  private sealed class VerifyResponse
  {
    public List<VerifyResult> Results { get; set; } = new();
  }

  private sealed class VerifyResult
  {
    public string? Id { get; set; }
    public Vec3 Position { get; set; } = new();
    public Quat Quaternion { get; set; } = new();
  }

  private sealed class Vec3
  {
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
  }

  private sealed class Quat
  {
    public double W { get; set; }
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
  }
}
