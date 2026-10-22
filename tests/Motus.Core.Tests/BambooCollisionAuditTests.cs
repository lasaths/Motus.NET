using Motus.Core;
using Motus.Geometry;
using Motus.Construction;

namespace Motus.Core.Tests;

/// <summary>
/// Independent dense collision audit of the Warren plan (shipped JSON and fresh planner output). Every key segment is
/// sampled in joint space 4× finer than the planner / viewer scan (joint step and jaw step ÷ 4, segment endpoints =
/// every key, so every swap / grasp / release pose), and each sample is checked with Motus collision: robot links and
/// self-collision, both grippers, mount parts and the carried cane vs ground pads, pit, anchor columns, connectors,
/// placed / parked canes and the stock. Clearance to the environment is estimated by re-checking with every obstacle
/// inflated by δ (2, 3, 5, 10 mm) with the shared check's exemptions: the reported minimum is the largest δ still clear
/// (a lower bound within one level). Hits are checked with the configured margin (2 mm) already applied.
/// Designed touches (each gripper's jaw plates on what it clamps) are the only allowed contacts; objects being clamped
/// are excluded from the clearance estimate (not from the hit check). Self-collision is checked as hit / no hit only.
/// </summary>
public partial class BambooTrussTests
{
    static readonly double[] AuditLevelsMm = { 10, 5, 3, 2 };

    sealed record AuditResult(int Segments, int Samples, int Hits, string? FirstHit, double MinClearMm, string MinWhere,
        int BelowMargin, double Seconds, Dictionary<double, int> SamplesBelow);

    AuditResult Audit(BambooTrussMotion.TrussBuild built, string name)
    {
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var c0 = MotionConstraints.Current;
        var dense = c0 with { ScanStepDeg = c0.ScanStepDeg / 4, JawStepMm = c0.JawStepMm / 4 };
        var scan = new BambooTrussScan(Cell, built.Keys, built.Cycles, BambooTrussLayout.Members, BambooTrussLayout.JawClosedMm, null, dense);
        var canonical = new BambooTrussScan(Cell, built.Keys, built.Cycles, BambooTrussLayout.Members, BambooTrussLayout.JawClosedMm);
        Assert.True(scan.SampleCount >= 3.9 * canonical.SampleCount, $"dense {scan.SampleCount} vs canonical {canonical.SampleCount}");
        int hits = 0, below = 0; string? first = null;
        var minClear = double.PositiveInfinity; var where = "";
        var belowCount = AuditLevelsMm.ToDictionary(l => l, _ => 0);
        var marginMm = c0.ClearanceMarginM * 1000;
        var near = new Dictionary<string, int>();
        foreach (var t in scan.SampleTimes)
        {
            var s = BambooTrussMotion.SampleState(built.Keys, t);
            var contacts = scan.ContactsAt(t, s.Q, s.GR, s.GL, s.Hold, 4);
            if (contacts.Count > 0)
            {
                hits++;
                first ??= $"{name} t={t:F6} {s.Label}: {BambooIcdCell.Describe(contacts)}";
                continue;
            }
            // Clearance: largest inflation level still clear (0 when even 1 mm touches).
            var env = scan.EnvironmentAt(t, s.Hold);
            var clear = 0.0;
            foreach (var lvl in AuditLevelsMm)
            {
                var ci = ContactsInflated(scan, built, t, s, env, lvl / 1000);
                if (ci.Count == 0) { clear = lvl; break; }
                belowCount[lvl]++;
                if (lvl == AuditLevelsMm[^1])
                {
                    var pair = BambooIcdCell.Describe(ci.Take(1).ToList()) + " @ " + s.Label.Split(':').Last();
                    near[pair] = near.GetValueOrDefault(pair) + 1;
                }
            }
            if (clear < marginMm) below++;
            if (clear < minClear)
            {
                minClear = clear;
                where = $"t={t:F6} {s.Label}" + (clear < 10 ? $" (closest at +{NextLevel(clear)} mm: {BambooIcdCell.Describe(ContactsInflated(scan, built, t, s, env, NextLevel(clear) / 1000))})" : "");
            }
        }
        var r = new AuditResult(built.Keys.Count - 1, scan.SampleCount, hits, first, minClear, where, below, clock.Elapsed.TotalSeconds, belowCount);
        _out.WriteLine($"{name}: {r.Segments} segments, {r.Samples} samples (canonical {canonical.SampleCount}), {r.Hits} hits, " +
            $"min clearance ≥ {r.MinClearMm} mm at {r.MinWhere}; samples below 10/5/3/2 mm: {string.Join("/", AuditLevelsMm.Select(l => belowCount[l]))}; " +
            $"below margin ({marginMm} mm): {below}; {r.Seconds:F0} s");
        if (first is not null) _out.WriteLine("first hit: " + first);
        foreach (var (pair, n) in near.OrderByDescending(x => x.Value).Take(12)) _out.WriteLine($"  < {AuditLevelsMm[^1]} mm: {n,6} × {pair}");
        return r;

        static double NextLevel(double clear) => AuditLevelsMm.Where(l => l > clear).DefaultIfEmpty(AuditLevelsMm[^1]).Min();
    }

    /// <summary>Contacts with every non-exempt obstacle grown by <paramref name="delta"/> — the shared check's margin logic.</summary>
    static IReadOnlyList<CollisionContact> ContactsInflated(BambooTrussScan scan, BambooTrussMotion.TrussBuild built, double t,
        BambooTrussMotion.State s, List<CollisionObject> env, double delta)
    {
        var k = built.Keys[s.SegEnd];
        var (g, w, _) = scan.BaseAt(t);
        var world = scan.World[BambooTrussMotion.CycleIndexAt(built.Keys, built.Cycles, t)];
        var held = s.Hold == 1 ? world.HeldOffset : null;
        return BambooTrussCollision.Contacts(Cell, g, w, s.Q, s.GR, s.GL, held, env, k.LOn, k.ROn, 4, delta);
    }

    /// <summary>
    /// Dense audit (4× finer than the planner) of the shipped plan and fresh planner output: fails on any Motus hit or a
    /// clearance below <see cref="MotionConstraints.ClearanceMarginM"/>. Also runs the viewer's hit-stop scan to the end.
    /// </summary>
    [Fact]
    [Trait("Category", "Slow")]
    public void CollisionAudit_Dense_ShippedAndFreshPlans_NoHit()
    {
        var fresh = Build();
        BambooTrussMotion.UseCell(Cell);
        var shipped = BambooTrussMotion.LoadCachedDefaultPlan();
        Assert.NotNull(shipped);
        Assert.Equal(7, fresh.Built.Count);
        Assert.Equal(fresh.Built, shipped!.Built);
        foreach (var (b, name) in new[] { (shipped, "shipped"), (fresh, "fresh") })
        {
            // Viewer path: the hit-stop scan the page runs during Play.
            var viewer = Scan(b).RunToEnd();
            Assert.True(viewer.FirstHit is null, $"{name} viewer scan hit: {viewer.FirstHit?.Label} {viewer.FirstHit?.Describe()}");
            Assert.Equal(1.0, viewer.LastClearT, 9);
            _out.WriteLine($"{name}: viewer scan {viewer.SampleCount} samples, clear to t=1");
            var r = Audit(b, name);
            Assert.True(r.Hits == 0, r.FirstHit);
            Assert.Equal(0, r.BelowMargin);
        }
    }
}
