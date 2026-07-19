using System.Diagnostics;
using System.Text.Json;
using Motus.Core;
using Motus.Geometry;
using Motus.OMPL.NET;
using Motus.Presets;

namespace Motus.Core.Tests;

/// <summary>Builds viewer_report.json from fk_cases.json and planning scenarios.</summary>
internal static class ViewerReportGenerator
{
  private static readonly JsonSerializerOptions ReadOptions = new() { PropertyNameCaseInsensitive = true };
  private static readonly JsonSerializerOptions WriteOptions = new()
  {
    PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
    WriteIndented = true,
  };
  private static readonly JsonSerializerOptions NodeJsonOptions = new()
  {
    PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
    PropertyNameCaseInsensitive = true,
  };

  public static string ReportPath(string repoRoot) =>
    Path.Combine(repoRoot, "tests", "fixtures", "viewer_report.json");

  public static ViewerReport Build(string repoRoot)
  {
    var fixturesDir = Path.Combine(repoRoot, "tests", "fixtures");
    var verifyScript = Path.Combine(repoRoot, "tools", "urdf-fk-verify.mjs");
    if (!File.Exists(verifyScript))
      throw new InvalidOperationException($"Missing verify script: {verifyScript}");

    var casesPath = Path.Combine(fixturesDir, "fk_cases.json");
    var cases = JsonSerializer.Deserialize<List<FkCase>>(File.ReadAllText(casesPath), ReadOptions)
      ?? throw new InvalidOperationException("Failed to load fk_cases.json");

    var report = new ViewerReport
    {
      GeneratedUtc = DateTime.UtcNow.ToString("o"),
      Fixtures = new Dictionary<string, ViewerFixtureReport>(),
    };

    foreach (var group in cases.GroupBy(c => c.UrdfFile))
    {
      var first = group.First();
      var urdfPath = Path.Combine("tests", "fixtures", first.UrdfFile).Replace('\\', '/');
      var motusById = new Dictionary<string, Frame>();

      foreach (var c in group)
      {
        var robot = UrdfRobotLoader.Load(Path.Combine(fixturesDir, c.UrdfFile), new UrdfLoadOptions
        {
          BaseLink = c.BaseLink,
          TipLink = c.TipLink,
        });
        var positions = robot.JointNames.Select(name => c.Joints[name]).ToArray();
        if (robot.JointNames.Count != positions.Length)
          throw new InvalidOperationException($"{c.Id}: joint count mismatch");

        var fk = new SerialForwardKinematics(robot.Chain);
        var pose = fk.ComputeTcp(new JointState(positions), robot.Preset.BaseFrame, robot.Preset.ToolFrame);
        motusById[c.Id] = pose.Tcp;
      }

      var request = new VerifyRequest
      {
        RepoRoot = repoRoot,
        UrdfPath = urdfPath,
        TipLink = first.TipLink,
        Cases = group.Select(c => new VerifyCase { Id = c.Id, Joints = c.Joints }).ToList(),
      };

      var response = RunVerifyScript(repoRoot, verifyScript, request);
      if (response.Results.Count != group.Count())
        throw new InvalidOperationException($"Verify script returned {response.Results.Count} results, expected {group.Count()}");

      foreach (var result in response.Results)
      {
        if (result.Id is null)
          throw new InvalidOperationException("Verify result missing id");

        var motus = motusById[result.Id];
        var posErr = Math.Sqrt(
          Math.Pow(result.Position.X - motus.X, 2) +
          Math.Pow(result.Position.Y - motus.Y, 2) +
          Math.Pow(result.Position.Z - motus.Z, 2));

        var dot = Math.Abs(
          result.Quaternion.W * motus.Qw +
          result.Quaternion.X * motus.Qx +
          result.Quaternion.Y * motus.Qy +
          result.Quaternion.Z * motus.Qz);
        dot = Math.Min(1.0, dot);
        var angleErr = 2 * Math.Acos(dot);
        var passed = posErr < 0.001 && angleErr < 0.01;

        var fkCase = group.First(c => c.Id == result.Id);
        var caseReport = new ViewerCaseReport
        {
          Id = result.Id,
          UrdfFile = fkCase.UrdfFile,
          BaseLink = fkCase.BaseLink,
          TipLink = fkCase.TipLink,
          Joints = fkCase.Joints,
          MotusTcp = TcpPose.FromFrame(motus),
          ReferenceTcp = new TcpPose
          {
            Position = new Vec3Dto { X = result.Position.X, Y = result.Position.Y, Z = result.Position.Z },
            Quaternion = new QuatDto
            {
              W = result.Quaternion.W,
              X = result.Quaternion.X,
              Y = result.Quaternion.Y,
              Z = result.Quaternion.Z,
            },
          },
          PositionErrorM = posErr,
          OrientationErrorRad = angleErr,
          Passed = passed,
        };

        report.Summary.Total++;
        if (passed) report.Summary.Passed++;
        else report.Summary.Failed++;

        var fixtureId = MapViewerFixtureId(first.UrdfFile);
        if (fixtureId is null) continue;

        if (!report.Fixtures.TryGetValue(fixtureId, out var fixtureReport))
        {
          fixtureReport = new ViewerFixtureReport { TipLink = first.TipLink };
          report.Fixtures[fixtureId] = fixtureReport;
        }

        fixtureReport.Cases.Add(caseReport);
        report.Summary.ViewerCases++;
      }
    }

    AppendPlanningScenarios(report, repoRoot, fixturesDir);
    return report;
  }

  private static void AppendPlanningScenarios(ViewerReport report, string repoRoot, string fixturesDir)
  {
    var resources = Path.Combine(repoRoot, "resources", "robots");
    var ur10eModel = PresetLoader.LoadRobotModelByName("UR10e", resources);
    PlanScenario(
      report,
      "ur10e",
      ur10eModel.Preset,
      null,
      new double[] { 0, 0, 0, 0, 0, 0 },
      new double[] { 0.6, -0.6, 0.6, -0.6, -0.6, 0.3 },
      11,
      ur10eModel);

    var kr210 = UrdfRobotLoader.Load(Path.Combine(fixturesDir, "kr210_r3100_ultra/kr210_r3100_ultra_minimal.urdf"), new UrdfLoadOptions
    {
      BaseLink = "base_link",
      TipLink = "tool0",
    });
    PlanScenario(
      report,
      "kr210_r3100_ultra",
      kr210.Preset,
      kr210.Chain,
      new double[] { 0, -1.2, 1.5, 0, 0.5, 0 },
      new double[] { 0.4, -0.8, 2.0, -0.5, -0.3, 0.6 },
      17,
      kr210.ToModel());
  }

  private static void PlanScenario(
    ViewerReport report,
    string fixtureId,
    RobotPreset preset,
    SerialJointChain? chain,
    double[] startJoints,
    double[] goalJoints,
    int seed,
    RobotModel? model = null)
  {
    model ??= new RobotModel(preset);

    var start = new JointState(startJoints);
    var goal = new JointState(goalJoints);
    var fk = KinematicsResolver.CreateFkSolver(preset, chain);
    var checker = chain is null
      ? new SphereCollisionChecker(preset)
      : new SphereCollisionChecker(fk, preset.BaseFrame);
    var scene = RequireBlockingScene(checker, fk, preset.BaseFrame, preset.ToolFrame, start, goal, 0.08)
      ?? throw new InvalidOperationException($"Could not place blocking obstacle for {fixtureId}");

    var obstacle = scene.Objects[0];
    var opts = new PlanningOptions
    {
      CollisionScene = scene,
      CollisionChecker = checker,
      MaxJointStepRadians = 0.08,
    };
    var planner = chain is null
      ? new RrtConnectPlanner(preset, new RrtConnectOptions { MaxIterations = 10000, RandomSeed = seed, PreferManaged = true })
      : new RrtConnectPlanner(checker, new RrtConnectOptions { MaxIterations = 10000, RandomSeed = seed, PreferManaged = true });
    var result = planner.Plan(new PlanningRequest(model, start, goal, opts));
    var passed = result.Success;
    if (result.Trajectory is not null)
    {
      var validation = new TrajectoryValidator().Validate(result.Trajectory, new TrajectoryValidationOptions
      {
        CollisionChecker = checker,
        CollisionScene = scene,
        CheckAcceleration = false,
      });
      passed &= validation.IsValid;
    }

    if (!report.Fixtures.TryGetValue(fixtureId, out var fixture))
    {
      fixture = new ViewerFixtureReport { TipLink = "tool0" };
      report.Fixtures[fixtureId] = fixture;
    }

    var jointNames = model.JointNames;
    fixture.Scenarios.Add(new ViewerScenarioReport
    {
      Id = $"{fixtureId}_rrt_obstacle",
      Label = "RRT-Connect around sphere",
      Planner = "RrtConnect",
      Passed = passed,
      Obstacles = new List<ViewerObstacleReport> { ToObstacle(obstacle) },
      Points = result.Trajectory is null ? new() : SampleTrajectory(result.Trajectory, jointNames),
      Errors = passed ? new() : result.Errors.ToList(),
    });

    report.Summary.PlanningScenarios++;
    if (passed) report.Summary.PlanningPassed++;
    else report.Summary.PlanningFailed++;
  }

  private static CollisionScene? RequireBlockingScene(
    ICollisionChecker checker,
    IFkSolver fk,
    BaseFrame baseFrame,
    ToolFrame toolFrame,
    JointState start,
    JointState goal,
    double stepRadians)
  {
    var direct = FindBlockingScene(checker, fk, baseFrame, toolFrame, start, goal, stepRadians);
    if (direct is not null) return direct;

    var startTcp = fk.ComputeTcp(start, baseFrame, toolFrame).Tcp;
    var goalTcp = fk.ComputeTcp(goal, baseFrame, toolFrame).Tcp;
    var cartMid = new Frame(
      (startTcp.X + goalTcp.X) / 2,
      (startTcp.Y + goalTcp.Y) / 2,
      (startTcp.Z + goalTcp.Z) / 2);
    foreach (var radius in new[] { 0.08, 0.1, 0.12, 0.15, 0.2 })
    {
      var trial = new CollisionScene(new[] { CollisionObject.Sphere("block", cartMid, radius) });
      if (checker.IsCollisionFree(start, trial) && checker.IsCollisionFree(goal, trial)
          && !checker.SegmentCollisionFree(start, goal, trial, stepRadians))
        return trial;
    }

    for (var s = 1; s <= 15; s++)
    {
      var alpha = s / 16.0;
      var q = new double[start.AxisCount];
      for (var i = 0; i < q.Length; i++)
        q[i] = start.Positions[i] + alpha * (goal.Positions[i] - start.Positions[i]);
      var tcp = fk.ComputeTcp(new JointState(q), baseFrame, toolFrame).Tcp;
      foreach (var radius in new[] { 0.08, 0.1, 0.12, 0.15 })
      {
        var trial = new CollisionScene(new[] { CollisionObject.Sphere("block", tcp, radius) });
        if (checker.IsCollisionFree(start, trial) && checker.IsCollisionFree(goal, trial)
            && !checker.SegmentCollisionFree(start, goal, trial, stepRadians))
          return trial;
      }
    }

    return null;
  }

  private static CollisionScene? FindBlockingScene(
    ICollisionChecker checker,
    IFkSolver fk,
    BaseFrame baseFrame,
    ToolFrame toolFrame,
    JointState start,
    JointState goal,
    double stepRadians)
  {
    for (var s = 1; s <= 7; s++)
    {
      var alpha = s / 8.0;
      var q = new double[start.AxisCount];
      for (var i = 0; i < q.Length; i++)
        q[i] = start.Positions[i] + alpha * (goal.Positions[i] - start.Positions[i]);
      var mid = fk.ComputeTcp(new JointState(q), baseFrame, toolFrame);
      var trial = new CollisionScene(new[] { CollisionObject.Sphere("block", mid.Tcp, 0.08) });
      if (checker.IsCollisionFree(start, trial) && checker.IsCollisionFree(goal, trial)
          && !checker.SegmentCollisionFree(start, goal, trial, stepRadians))
        return trial;
    }
    return null;
  }

  private static ViewerObstacleReport ToObstacle(CollisionObject obj) => obj.Shape switch
  {
    CollisionShape.Sphere => new ViewerObstacleReport
    {
      Shape = "sphere",
      Name = obj.Name,
      X = obj.Pose.X,
      Y = obj.Pose.Y,
      Z = obj.Pose.Z,
      Radius = obj.ExtentX,
    },
    CollisionShape.Box => new ViewerObstacleReport
    {
      Shape = "box",
      Name = obj.Name,
      X = obj.Pose.X,
      Y = obj.Pose.Y,
      Z = obj.Pose.Z,
      HalfX = obj.ExtentX,
      HalfY = obj.ExtentY,
      HalfZ = obj.ExtentZ,
    },
    CollisionShape.Plane => new ViewerObstacleReport
    {
      Shape = "plane",
      Name = obj.Name,
      X = obj.Pose.X,
      Y = obj.Pose.Y,
      Z = obj.Pose.Z,
    },
    _ => throw new NotSupportedException($"Viewer export does not support {obj.Shape}"),
  };

  private static List<ViewerTrajectoryPointReport> SampleTrajectory(Trajectory trajectory, IReadOnlyList<string>? jointNames)
  {
    const int maxPoints = 48;
    var pts = trajectory.Points;
    if (pts.Count == 0) return new();

    IEnumerable<TrajectoryPoint> picked = pts;
    if (pts.Count > maxPoints)
    {
      picked = Enumerable.Range(0, maxPoints)
        .Select(i => pts[(int)Math.Round(i * (pts.Count - 1) / (double)(maxPoints - 1))]);
    }

    return picked.Select(p =>
    {
      var joints = new Dictionary<string, double>();
      if (jointNames is not null)
      {
        for (var i = 0; i < jointNames.Count; i++)
          joints[jointNames[i]] = p.JointState.Positions[i];
      }
      return new ViewerTrajectoryPointReport
      {
        TimeSeconds = p.TimeSeconds,
        Joints = joints,
      };
    }).ToList();
  }

  public static void Write(string repoRoot, ViewerReport report)
  {
    var json = JsonSerializer.Serialize(report, WriteOptions);
    File.WriteAllText(ReportPath(repoRoot), json);
  }

  public static string? MapViewerFixtureId(string urdfFile)
  {
    if (urdfFile.StartsWith("ur10e", StringComparison.OrdinalIgnoreCase)) return "ur10e";
    if (urdfFile.StartsWith("kr210_r3100_ultra", StringComparison.OrdinalIgnoreCase)) return "kr210_r3100_ultra";
    return null;
  }

  private static VerifyResponse RunVerifyScript(string repoRoot, string verifyScript, VerifyRequest request)
  {
    var input = JsonSerializer.Serialize(request, NodeJsonOptions);
    var psi = new ProcessStartInfo("node", verifyScript)
    {
      RedirectStandardInput = true,
      RedirectStandardOutput = true,
      RedirectStandardError = true,
      UseShellExecute = false,
      CreateNoWindow = true,
      WorkingDirectory = repoRoot,
    };

    using var process = Process.Start(psi) ?? throw new InvalidOperationException("Failed to start node verify script.");
    process.StandardInput.Write(input);
    process.StandardInput.Close();
    var stdout = process.StandardOutput.ReadToEnd();
    var stderr = process.StandardError.ReadToEnd();
    process.WaitForExit(30_000);

    if (process.ExitCode != 0)
      throw new InvalidOperationException($"urdf-fk-verify failed (exit {process.ExitCode}): {stderr}");

    return JsonSerializer.Deserialize<VerifyResponse>(stdout, ReadOptions)
      ?? throw new InvalidOperationException("Invalid verify script output.");
  }

  internal sealed class ViewerReport
  {
    public string GeneratedUtc { get; set; } = "";
    public ViewerSummary Summary { get; set; } = new();
    public Dictionary<string, ViewerFixtureReport> Fixtures { get; set; } = new();
  }

  internal sealed class ViewerSummary
  {
    public int Total { get; set; }
    public int Passed { get; set; }
    public int Failed { get; set; }
    public int ViewerCases { get; set; }
    public int PlanningScenarios { get; set; }
    public int PlanningPassed { get; set; }
    public int PlanningFailed { get; set; }
  }

  internal sealed class ViewerFixtureReport
  {
    public string TipLink { get; set; } = "tool0";
    public List<ViewerCaseReport> Cases { get; set; } = new();
    public List<ViewerScenarioReport> Scenarios { get; set; } = new();
  }

  internal sealed class ViewerCaseReport
  {
    public string Id { get; set; } = "";
    public string UrdfFile { get; set; } = "";
    public string BaseLink { get; set; } = "";
    public string TipLink { get; set; } = "";
    public Dictionary<string, double> Joints { get; set; } = new();
    public TcpPose MotusTcp { get; set; } = new();
    public TcpPose ReferenceTcp { get; set; } = new();
    public double PositionErrorM { get; set; }
    public double OrientationErrorRad { get; set; }
    public bool Passed { get; set; }
  }

  internal sealed class ViewerScenarioReport
  {
    public string Id { get; set; } = "";
    public string Label { get; set; } = "";
    public string Planner { get; set; } = "";
    public bool Passed { get; set; }
    public List<ViewerObstacleReport> Obstacles { get; set; } = new();
    public List<ViewerTrajectoryPointReport> Points { get; set; } = new();
    public List<string> Errors { get; set; } = new();
  }

  internal sealed class ViewerObstacleReport
  {
    public string Shape { get; set; } = "";
    public string Name { get; set; } = "";
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
    public double Radius { get; set; }
    public double HalfX { get; set; }
    public double HalfY { get; set; }
    public double HalfZ { get; set; }
  }

  internal sealed class ViewerTrajectoryPointReport
  {
    public double TimeSeconds { get; set; }
    public Dictionary<string, double> Joints { get; set; } = new();
  }

  internal sealed class TcpPose
  {
    public Vec3Dto Position { get; set; } = new();
    public QuatDto Quaternion { get; set; } = new();

    public static TcpPose FromFrame(Frame f) => new()
    {
      Position = new Vec3Dto { X = f.X, Y = f.Y, Z = f.Z },
      Quaternion = new QuatDto { W = f.Qw, X = f.Qx, Y = f.Qy, Z = f.Qz },
    };
  }

  internal sealed class Vec3Dto
  {
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
  }

  internal sealed class QuatDto
  {
    public double W { get; set; }
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
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
    public Vec3Dto Position { get; set; } = new();
    public QuatDto Quaternion { get; set; } = new();
  }
}
