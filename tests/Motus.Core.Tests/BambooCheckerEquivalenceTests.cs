using Motus.Core;
using Motus.Geometry;
using Motus.Viewer.Services;
using Xunit.Abstractions;

namespace Motus.Core.Tests;

/// <summary>
/// Collision speedups must not change a single verdict: every pose the planner checks on a fixed short case (start +
/// the first two members that build directly, 2 mm margin) is replayed through the reference (pre-speedup) path —
/// no broadphase prefilter, no allowed-name shortcut, no memo, two-pass margin — and must agree exactly.
/// Set MOTUS_PERF_OUT=&lt;dir&gt; to also write timings and the verdict log there.
/// </summary>
public class BambooCheckerEquivalenceTests
{
    readonly ITestOutputHelper _out;
    public BambooCheckerEquivalenceTests(ITestOutputHelper o) { _out = o; }

    static string UrdfPath => Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..",
        "tests", "fixtures", "bamboo_icd", "bamboo_icd.urdf"));

    static BambooIcdCell NewCell()
    {
        var cell = BambooIcdCell.LoadFile(UrdfPath, BambooTrussLayout.StrutLength);
        BambooTrussLayout.Configure(cell);
        return cell;
    }

    [Fact]
    public void ShortCase_PoseVerdicts_IdenticalToReferenceChecker()
    {
        var cell = NewCell();
        var p = new BambooTrussPlanner(cell) { VerdictLog = new() };
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var lines = new List<string> { $"start ok={p.Start()} {sw.Elapsed.TotalSeconds:F1}s" };
        var built = 0;
        foreach (var m in BambooTrussLayout.Members)
        {
            if (built >= 2) break;
            var t0 = sw.Elapsed.TotalSeconds;
            var cp = p.SaveDeep();
            var ok = p.BuildMember(m);
            lines.Add($"{m.Id}: ok={ok} {sw.Elapsed.TotalSeconds - t0:F1}s checks={p.Checks} pose={p.PoseWatch.Elapsed.TotalSeconds:F1}s");
            if (ok) built++; else p.RollbackDeep(cp);
        }
        Assert.Equal(2, built);
        lines.Add($"TOTAL {sw.Elapsed.TotalSeconds:F1}s checks={p.Checks} poseWatch={p.PoseWatch.Elapsed.TotalSeconds:F1}s poses/s={p.Checks / p.PoseWatch.Elapsed.TotalSeconds:F0}");
        var log = p.VerdictLog!;
        Assert.True(log.Count > 1000);
        Assert.Contains(log, v => !v.Ok);

        // Replay on a fresh cell through the reference path (no caches shared with the run above).
        var refCell = NewCell();
        var margin = p.Constraints.ClearanceMarginM;
        var mismatches = new List<string>();
        var rw = System.Diagnostics.Stopwatch.StartNew();
        RobotMeshCollisionChecker.ReferencePath = true;
        try
        {
            foreach (var v in log)
            {
                var refOk = BambooTrussCollision.Contacts(refCell, v.Base, v.BaseWorld, v.Q, v.GR, v.GL, v.Held, v.Env, v.LOn, v.ROn, 1, margin).Count == 0;
                if (refOk != v.Ok && mismatches.Count < 20)
                    mismatches.Add($"q=[{string.Join(",", v.Q.Select(x => x.ToString("R")))}] gR={v.GR:R} gL={v.GL:R} hold={v.Hold} new={v.Ok} ref={refOk}");
            }
        }
        finally { RobotMeshCollisionChecker.ReferencePath = false; }
        lines.Add($"reference replay: {log.Count} poses in {rw.Elapsed.TotalSeconds:F1}s ({log.Count / rw.Elapsed.TotalSeconds:F0} poses/s), {log.Count(v => !v.Ok)} colliding, {mismatches.Count} mismatches");
        foreach (var l in lines) _out.WriteLine(l);
        if (System.Environment.GetEnvironmentVariable("MOTUS_PERF_OUT") is { Length: > 0 } dir)
        {
            Directory.CreateDirectory(dir);
            var tag = System.Environment.GetEnvironmentVariable("MOTUS_PERF_TAG") ?? "run";
            File.WriteAllLines(Path.Combine(dir, $"short-{tag}.txt"), lines);
            File.WriteAllLines(Path.Combine(dir, $"verdicts-{tag}.txt"), log.Select(v =>
                $"{v.Hold}|{v.LOn}|{v.ROn}|{v.GR:R}|{v.GL:R}|{string.Join(",", v.Q.Select(x => x.ToString("R")))}|{v.Ok}"));
        }
        Assert.Empty(mismatches);
    }
}
