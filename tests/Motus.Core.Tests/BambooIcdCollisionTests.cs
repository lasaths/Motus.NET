using Motus.Core;
using Motus.Geometry;
using Motus.Viewer.Services;

namespace Motus.Core.Tests;

/// <summary>
/// Full, physically-true collision for the ICD/LIS bamboo arm as /bamboo runs it: the hung URDF (link frames ==
/// drawn groups, collision boxes == drawn boxes) against pole, strut, ground, mount and itself, decided by
/// Motus.NET's <see cref="RobotMeshCollisionChecker"/>. The viewer services are compiled into this test
/// assembly verbatim (see csproj), so these tests exercise exactly the /bamboo code path.
/// </summary>
public class BambooIcdCollisionTests
{
    private static readonly Lazy<BambooIcdCell> LazyCell = new(() => BambooIcdCell.LoadFile(UrdfPath));

    private static string UrdfPath => Path.GetFullPath(Path.Combine(
        AppContext.BaseDirectory, "..", "..", "..", "..", "..", "tests", "fixtures", "bamboo_icd", "bamboo_icd.urdf"));

    private static BambooIcdCell Cell => LazyCell.Value;

    private static BambooMotionScan Scan(List<BambooIcdMotion.TaskEdit> tasks)
    {
        var built = BambooIcdMotion.BuildMotion(tasks);
        Assert.True(string.IsNullOrEmpty(built.Error), $"motion should build: {built.Error}");
        return new BambooMotionScan(Cell, built.Keys, BambooIcdCell.StrutRest(tasks), maxStepDeg: 0.5).RunToEnd();
    }

    private static List<BambooIcdMotion.TaskEdit> Nudged(string identity, string plane, double yMm)
    {
        var tasks = BambooIcdMotion.DefaultTasks();
        tasks.First(t => t.Identity == identity).Planes.First(p => p.Name == plane).YMm = yMm;
        return tasks;
    }

    private static bool Involves(BambooMotionScan.Hit hit, string a, string b) =>
        hit.Contacts.Any(c => (c.BodyA == a && c.BodyB == b) || (c.BodyA == b && c.BodyB == a));

    // ---- Frame alignment: Motus FK poses are the drawn poses -------------------------------------------

    [Theory]
    [InlineData(-90, -72, -80, -24, 0)]
    [InlineData(0, 0, 0, 0, 0)]
    [InlineData(-90, -24.1, -15.4, 39.5, 0)]
    [InlineData(30, 40, -60, 50, 90)]
    [InlineData(-150, 85, 120, -80, -160)]
    public void UrdfFk_MatchesDrawnHangKinematics(double q1, double q2, double q3, double q4, double q5)
    {
        var q = new[] { q1, q2, q3, q4, q5 };
        var drawn = BambooIcdMotion.FkJaw(q);
        var jaw = Cell.JawViewer(q);
        Assert.Equal(drawn.Pos[0], jaw.X, 9);
        Assert.Equal(drawn.Pos[1], jaw.Y, 9);
        Assert.Equal(drawn.Pos[2], jaw.Z, 9);

        // Every joint origin (θ1..θ5) also matches the drawn chain.
        var mats = new double[5][];
        for (var i = 0; i < 5; i++) mats[i] = new double[16];
        Cell.Fk.ComputeLinkTransformsInto(BambooIcdCell.RadFromDeg(q), mats);
        for (var i = 0; i < 5; i++)
        {
            var p = drawn.Pts[i + 1]; // Pts[0] = mount (pole), Pts[i+1] = joint i+1
            Assert.Equal(p[0], mats[i][3], 9);   // viewer x  = Motus X
            Assert.Equal(-p[2], mats[i][7], 9);  // viewer z  = -Motus Y
            Assert.Equal(p[1], mats[i][11], 9);  // viewer y  = Motus Z
        }
    }

    [Fact]
    public void Urdf_HangsUnderPole_AndCarriesOwnCollisionBoxes()
    {
        Assert.Equal(new[] { 0.030, 0.080, 0.088, 0.088, 0.088, 0.046 }, Cell.Seg);
        Assert.Equal(BambooIcdMotion.PoleY, Cell.MountWorld[11], 9); // mount at Motus Z = viewer y = 0.40
        var names = Cell.Parts.Select(p => p.Name).ToHashSet();
        foreach (var n in new[] { "base tube", "L-grip body", "θ1", "link θ2", "θ2", "link θ3", "θ3", "link θ4", "θ4",
                                  "link θ5", "θ5", "neck", "R-grip carriage", "R-grip rail", "R-grip jaw-", "R-grip jaw+" })
            Assert.Contains(n, names);
        Assert.All(Cell.Model.CollisionModel!.Links, l => Assert.Equal(CollisionShape.Box, l.LocalGeometry.Shape));
    }

    [Fact]
    public void Scene_ContainsPoleStrutGroundAndMount()
    {
        var keys = BambooIcdMotion.BuildMotion(BambooIcdMotion.DefaultTasks()).Keys;
        var track = Cell.TrackStrut(keys, BambooIcdCell.StrutRest(BambooIcdMotion.DefaultTasks()));
        var scene = Cell.Scene(track, hold: 0, gripping: false, gLmm: 24, BambooIcdMotion.HomeDeg);
        var objs = scene.Objects.Select(o => o.Name).ToList();
        Assert.Contains(BambooIcdCell.Pole, objs);
        Assert.Contains(BambooIcdCell.Strut, objs);
        Assert.Contains(BambooIcdCell.Ground, objs);
        Assert.Contains("base tube", objs);
        Assert.Contains("L-grip jaw-", objs);
        Assert.Equal(CollisionShape.Mesh, scene.Objects.First(o => o.Name == BambooIcdCell.Strut).Shape);
        Assert.Equal(CollisionShape.Mesh, scene.Objects.First(o => o.Name == BambooIcdCell.Pole).Shape);

        // Held: strut leaves the scene and rides on the checker as an attached body at its captured offset.
        Assert.True(track.Captured);
        var held = Cell.Scene(track, hold: 1, gripping: true, gLmm: 24, BambooIcdMotion.HomeDeg);
        Assert.DoesNotContain(BambooIcdCell.Strut, held.Objects.Select(o => o.Name));
    }

    // ---- Paths: PASS clear, FAIL on hit ----------------------------------------------------------------

    [Fact]
    public void DefaultPickPlace_IsCollisionFree_UnderMotus()
    {
        var scan = Scan(BambooIcdMotion.DefaultTasks());
        Assert.Null(scan.Track.Miss);
        Assert.True(scan.Track.Captured);
        Assert.True(scan.Complete);
        Assert.True(scan.FirstHit is null, $"default path should be clear, Motus hit: {scan.FirstHit?.Describe()} at {scan.FirstHit?.T:F3}");
        Assert.Equal(1.0, scan.LastClearT, 9);
    }

    [Fact]
    public void GraspNudgedDown_DrivesNeckThroughStrut_FailsWithNamedContact()
    {
        // The strut lies where it lies; aiming the grasp 1.5 mm lower drives the neck into it
        // (default clearance neck↔strut is 1.0 mm).
        var scan = Scan(Nudged("pick", "grasp", -1.5));
        var hit = Assert.IsType<BambooMotionScan.Hit>(scan.FirstHit);
        Assert.True(Involves(hit, "neck", BambooIcdCell.Strut), hit.Describe());
        Assert.Equal("Grasp", hit.Label);
        Assert.True(hit.LastClearT < hit.T);
        // Last clear sample really is clear.
        var s = BambooIcdMotion.Sample(BambooIcdMotion.BuildMotion(Nudged("pick", "grasp", -1.5)).Keys, hit.LastClearT);
        Assert.Empty(Cell.Contacts(s.Q, s.GR, 24, s.Hold, s.Label, scan.Track));
    }

    [Fact]
    public void ReleaseNudgedDown_PushesHeldStrutIntoGround_Fails()
    {
        var scan = Scan(Nudged("place", "release", -10));
        var hit = Assert.IsType<BambooMotionScan.Hit>(scan.FirstHit);
        Assert.True(Involves(hit, BambooIcdCell.Strut, BambooIcdCell.Ground), hit.Describe());
        Assert.Contains(hit.Contacts, c => c.Kind == CollisionContactKind.Attached);
    }

    [Fact]
    public void PathSwingingArmThroughPole_Fails()
    {
        // Home is folded up beside the pole; swinging θ1 from -90° to 0° sweeps the forearm through the pole.
        var keys = new List<BambooIcdMotion.MotionKey>
        {
            new() { T = 0, Label = "Home", Q = BambooIcdMotion.HomeDeg.ToArray(), GR = 40 },
            new() { T = 1, Label = "Swing", Q = new double[] { 0, -72, -80, -24, 0 }, GR = 40 }
        };
        var scan = new BambooMotionScan(Cell, keys, BambooIcdCell.StrutRest(BambooIcdMotion.DefaultTasks())).RunToEnd();
        var hit = Assert.IsType<BambooMotionScan.Hit>(scan.FirstHit);
        Assert.Contains(hit.Contacts, c => c.BodyB == BambooIcdCell.Pole && c.Kind == CollisionContactKind.Scene);
        Assert.True(hit.T > 0 && hit.T < 1);
    }

    [Fact]
    public void PathFoldingJawIntoMount_Fails()
    {
        // In the working plane the pole is shielded by the L-grip; folding the wrist up hits the L-grip rail.
        var keys = new List<BambooIcdMotion.MotionKey>
        {
            new() { T = 0, Label = "Home", Q = BambooIcdMotion.HomeDeg.ToArray(), GR = 40 },
            new() { T = 1, Label = "Fold", Q = new double[] { -90, -60, -135, -30, 0 }, GR = 40 }
        };
        var scan = new BambooMotionScan(Cell, keys, BambooIcdCell.StrutRest(BambooIcdMotion.DefaultTasks())).RunToEnd();
        var hit = Assert.IsType<BambooMotionScan.Hit>(scan.FirstHit);
        Assert.Contains(hit.Contacts, c => c.BodyB == "L-grip rail");
    }

    [Fact]
    public void PathDivingIntoGround_Fails()
    {
        var keys = new List<BambooIcdMotion.MotionKey>
        {
            new() { T = 0, Label = "Home", Q = BambooIcdMotion.HomeDeg.ToArray(), GR = 40 },
            new() { T = 1, Label = "Down", Q = new double[] { -90, 0, 0, 0, 0 }, GR = 40 } // straight down: reach 0.42 > 0.40
        };
        var scan = new BambooMotionScan(Cell, keys, BambooIcdCell.StrutRest(BambooIcdMotion.DefaultTasks())).RunToEnd();
        var hit = Assert.IsType<BambooMotionScan.Hit>(scan.FirstHit);
        Assert.Contains(hit.Contacts, c => c.BodyB == BambooIcdCell.Ground || c.BodyB == BambooIcdCell.Strut);
    }

    [Fact]
    public void FoldedArm_ReportsSelfCollisionByPart()
    {
        var track = Cell.TrackStrut(BambooIcdMotion.BuildMotion(BambooIcdMotion.DefaultTasks()).Keys,
            BambooIcdCell.StrutRest(BambooIcdMotion.DefaultTasks()));
        var contacts = Cell.Contacts(new double[] { -90, 90, 135, 90, 0 }, 40, 24, 0, "Manual", track);
        Assert.Contains(contacts, c => c.Kind == CollisionContactKind.Self);
        Assert.Contains(contacts, c => c.BodyA.StartsWith("link θ2") || c.BodyB.StartsWith("link θ2"));
        Assert.Empty(Cell.Contacts(BambooIcdMotion.HomeDeg, 40, 24, 0, "Home", track));
    }

    // ---- Motus TaskPathValidator on bamboo task frames -------------------------------------------------

    private static TaskPathValidationResult ValidateFrames(List<BambooIcdMotion.TaskEdit> tasks)
    {
        var keys = BambooIcdMotion.BuildMotion(tasks).Keys;
        var track = Cell.TrackStrut(keys, BambooIcdCell.StrutRest(tasks));
        var scene = Cell.Scene(track, hold: 0, gripping: false, gLmm: 24, BambooIcdMotion.HomeDeg);
        var checker = Cell.Checker(40, null); // jaws open while approaching the frames
        return TaskPathValidator.Validate(BambooIcdMotion.ToTaskPath(tasks), Cell.Model, new BambooIcdIk(), checker, scene,
            seed: new JointState(BambooIcdCell.RadFromDeg(BambooIcdMotion.HomeDeg)));
    }

    [Fact]
    public void TaskPathValidator_DefaultBambooFrames_Pass()
    {
        var result = ValidateFrames(BambooIcdMotion.DefaultTasks());
        Assert.True(result.IsValid, string.Join("\n", result.Errors));
        Assert.Equal(4, result.ReachableStates.Count);
        // IK adapter round-trips through Motus FK onto the task frame.
        var grasp = BambooIcdMotion.ToTaskPath(BambooIcdMotion.DefaultTasks()).Tasks[0].Frames["grasp"];
        var tcp = Cell.Fk.ComputeTcp(result.ReachableStates[1], Cell.Model.Preset.BaseFrame, Cell.Model.Preset.ToolFrame).Tcp;
        Assert.Equal(grasp.X, tcp.X, 6);
        Assert.Equal(grasp.Y, tcp.Y, 6);
        Assert.Equal(grasp.Z, tcp.Z, 6);
    }

    [Fact]
    public void TaskPathValidator_GraspIntoStrut_Fails()
    {
        var result = ValidateFrames(Nudged("pick", "grasp", -1.5));
        Assert.False(result.IsValid);
        Assert.Contains(result.Errors, e => e.Contains("'pick'") && e.Contains("'grasp'") && e.Contains("collides"));
    }
}
