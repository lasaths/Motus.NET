// tower harness: copy of gridshell harness10 (e3e87d0+), site from TowerInputs (1x2 bays, 3 stories); writes tasks.json for a separate rebuild.
// Usage: harness <urdf-file> <n> <plain|tamp|geom> <out-dir>
using System.Text.Json;
using Motus.Construction;

var (urdfPath, n, mode, outDir) = (args[0], int.Parse(args[1]), args[2], args[3]);
Directory.CreateDirectory(outDir);
var site = TowerInputs.Build();
var tag = $"tower1x2-{mode}";
var clock = System.Diagnostics.Stopwatch.StartNew();
using var logFile = new StreamWriter(Path.Combine(outDir, tag + ".log")) { AutoFlush = true };
void Log(string s) { var line = $"[{clock.Elapsed.TotalSeconds,7:F1}s] {s}"; Console.WriteLine(line); logFile.WriteLine(line); }

BambooTrussLayout.Use(site);
var clearanceMm = 2.0;
if (Environment.GetEnvironmentVariable("CLEARANCE_MM") is { Length: > 0 } raw)
{
    if (!double.TryParse(raw, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out clearanceMm))
        throw new ArgumentException($"CLEARANCE_MM must be a finite number in [0, 20], got '{raw}'");
    MotionConstraints.Current = (MotionConstraints.Current with { ClearanceMarginM = clearanceMm / 1000 }).Validate();
}
var st = site.Structure!;
Log($"input {tag}: {st.Nodes.Count} nodes, {st.Members.Count} members, {site.Stakes.Count} anchors, cane lengths " +
    $"{st.Members.Min(m => m.Length):F3}–{st.Members.Max(m => m.Length):F3} m, node Y {st.Nodes.Min(x => x.Y):F3}–{st.Nodes.Max(x => x.Y):F3} m");

// Geometry dump for the renderer (always).
File.WriteAllText(Path.Combine(outDir, "tower-input.json"), JsonSerializer.Serialize(new
{
    nodes = st.Nodes.Select(x => new { x.Name, x.X, x.Y, x.Z }),
    members = BambooTrussLayout.Members.Select(m => new { m.Id, m.NodeA, m.NodeB, m.Length, m.Place.X, m.Place.Y, m.Place.Z, m.Place.PitchDeg, m.Place.YawDeg }),
    anchors = site.Stakes.Select(s => new { s.Name, s.X, s.Z, s.ClampY, s.Top, s.HeadingDeg }),
    feeds = BambooTrussLayout.Members.Select(m => new { m.Id, m.FeedIndex, m.Store.X, m.Store.Z, m.Store.YawDeg }),
    stock = new { X = site.FeedX, Y = BambooTrussLayout.StrutY, Z = site.FeedZ, Yaw = site.FeedYawDeg, Length = site.Length },
    start = site.StartStake
}, new JsonSerializerOptions { WriteIndented = true }));
if (mode == "geom") return;

var cell = BambooIcdCell.Load(File.ReadAllText(urdfPath), BambooTrussLayout.StrutLength, RobotDefinition.BambooIcd);
BambooTrussMotion.UseCell(cell);
Log($"constraints: {JsonSerializer.Serialize(MotionConstraints.Current)}");

// Footholds rule first: the layout fingerprint includes it, so the build search's own scene check reuses this one's reach cache.
if (Environment.GetEnvironmentVariable("FOOTHOLDS") == "1") MotionConstraints.Current = MotionConstraints.Current with { FixedMemberRule = false };
var check = BambooSceneCheck.Run(null, BambooTrussLayout.Members);
Log(check.Summary);
foreach (var d in check.Details) Log("  scene check · " + d);

var opts = new BambooBuildSearch.Options
{
    CheckpointPath = Path.Combine(outDir, tag + ".checkpoint.json"),
    AnchorWalks = true,
    NearestFirst = Environment.GetEnvironmentVariable("NEAREST") == "1",
    PlacedFootholds = Environment.GetEnvironmentVariable("FOOTHOLDS") == "1",
    StagedConfirm = Environment.GetEnvironmentVariable("STAGED") == "1",
    Reposition = Environment.GetEnvironmentVariable("REPOSITION") == "1",
    LandPrecheck = Environment.GetEnvironmentVariable("LANDPRE") == "1",
    RelayPrecheck = Environment.GetEnvironmentVariable("RELAYPRE") == "1",
    ConstrainedFirst = Environment.GetEnvironmentVariable("CONSTRAINED") == "1",
    // RESUME=<checkpoint.json>: re-confirm an earlier run's prefix first (read directly: the fingerprint includes the clamp rule).
    ResumeAt = Environment.GetEnvironmentVariable("RESUME") is { Length: > 0 } rp
        ? JsonSerializer.Deserialize<BambooBuildSearch.Checkpoint>(File.ReadAllText(rp))!.Order : null,
    MaxRejected = int.TryParse(Environment.GetEnvironmentVariable("MAXREJ"), out var mr) ? mr : 40,
    Tamp = mode == "tamp" ? new BambooBuildSearch.TampSettings { OutcomeLogPath = Path.Combine(outDir, tag + ".outcomes.jsonl") } : null
};
var r = BambooBuildSearch.Run(cell, opts, s => Log("search · " + s));
Log($"RESULT {tag}: placed {r.Order.Count}/{BambooTrussLayout.Members.Count}, complete={r.Complete}, stop='{r.StopReason}', " +
    $"confirmed {r.Confirmed}, rejected {r.Rejected}, deferred {r.Deferred}, search {r.Seconds:F0} s");
Log("order: " + string.Join(" → ", r.Order.Select(c => $"{(c.RepositionTo is null ? "" : "[reposition " + c.RepositionTo + "] ")}{c.MemberId}({c.Mode}{(c.Relay is null ? "" : " " + c.Relay)})")));
if (r.Order.Count == 0) return;
File.WriteAllText(Path.Combine(outDir, tag + ".tasks.json"), JsonSerializer.Serialize(new { order = r.Order.Select(c => c.MemberId), tasks = r.Tasks }));

// Replay the confirmed prefix with the sequence runner, then the exact scan; clearance bound by a margin sweep.
var build = BambooTrussMotion.RunSequence(r.Tasks, s => logFile.WriteLine("run · " + s));
Log($"replay: built {build.Built.Count} [{string.Join(",", build.Built)}], keys {build.Keys.Count}, error '{build.Error}'");
if (build.Keys.Count == 0) return;
var members = BambooTrussLayout.Members.ToList();
var scan = new BambooTrussScan(cell, build.Keys, build.Cycles, members, BambooTrussLayout.JawClosedMm); scan.RunToEnd();
Log($"scan @{clearanceMm} mm: {(scan.FirstHit is { } h ? $"collision t={h.T:F4} {h.Label}: {h.Describe()}" : scan.TrackMiss is { } tm ? "tracking miss " + tm : "collision-free")}");
var passed = clearanceMm;
foreach (var mm in new[] { 3.0, 4, 5, 6, 8, 10, 12, 15, 20 }.Where(mm => mm > clearanceMm))
{
    var c = MotionConstraints.Current with { ClearanceMarginM = mm / 1000 };
    var s2 = new BambooTrussScan(cell, build.Keys, build.Cycles, members, BambooTrussLayout.JawClosedMm, constraints: c).RunToEnd();
    if (s2.FirstHit is not null) { Log($"margin sweep: fails at {mm} mm ({s2.FirstHit.Label}: {s2.FirstHit.Describe()})"); break; }
    passed = mm;
}
Log($"min clearance (non-exempt pairs, sampled scan) ≥ {passed} mm");
File.WriteAllText(Path.Combine(outDir, tag + ".keys.json"), JsonSerializer.Serialize(new
{
    keys = build.Keys, built = build.Built, members = members.Select(m => new { m.Id, m.Length, m.Place.X, m.Place.Y, m.Place.Z, m.Place.PitchDeg, m.Place.YawDeg })
}));

// Frames for the GIF: the same pose sources the Blazor viewer pushes to three.js (StrutPoses, GrippersAt, ClampsAt).
var frames = new List<object>();
for (var f = 0; f <= 300; f++)
{
    var t = f / 300.0;
    var s = BambooTrussMotion.SampleState(build.Keys, t);
    var (lw, rw) = scan.GrippersAt(t, s.Q);
    frames.Add(new
    {
        t, label = s.Label, hold = s.Hold, L = lw, R = rw,
        struts = scan.StrutPoses(t, s.Hold, s.Q).Select(p => new { name = p.Name, pose = p.Pose }),
        clamps = scan.ClampsAt(t, s.Q, s.GR, s.GL).Select(c => new { c.Gripper, c.On, c.IsBase })
    });
}
File.WriteAllText(Path.Combine(outDir, tag + ".frames.json"), JsonSerializer.Serialize(new { strutLength = BambooTrussLayout.StrutLength, frames }));
Log("frames written");
