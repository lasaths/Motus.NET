using Motus.Core;
using Motus.Geometry;
using Motus.Viewer.Services;

namespace Motus.Core.Tests;

/// <summary>
/// Independent dense collision audit of the Warren plan (shipped JSON and fresh planner output). Every key segment is
/// sampled in joint space 4× finer than the planner / viewer scan (joint step and jaw step ÷ 4, segment endpoints =
/// every key, so every swap / grasp / release pose), and each sample is checked with Motus collision: robot links and
/// self-collision, both grippers, mount parts and the carried cane vs ground pads, pit, anchor columns, connectors,
/// placed / parked canes and the stock. Clearance to the environment is estimated by re-checking with every obstacle
/// inflated by δ (1, 2, 5, 10 mm): the reported minimum is the largest δ still clear (a lower bound within one level).
/// Designed touches (each gripper's jaw plates on what it clamps) are the only allowed contacts; objects being clamped
/// are excluded from the clearance estimate (not from the hit check). Self-collision is checked as hit / no hit only.
/// </summary>
public partial class BambooTrussTests
{
    static readonly double[] AuditLevelsMm = { 10, 5, 2, 1 };

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
                if (lvl == 1)
                {
                    var pair = BambooIcdCell.Describe(ci.Take(1).ToList()) + " @ " + s.Label.Split(':').Last();
                    near[pair] = near.GetValueOrDefault(pair) + 1;
                }
            }
            if (clear < marginMm) below++;
            if (clear < minClear)
            {
                minClear = clear;
                where = $"t={t:F6} {s.Label}" + (clear < 10 ? $" (closest at +{(clear == 0 ? 1 : NextLevel(clear))} mm: {BambooIcdCell.Describe(ContactsInflated(scan, built, t, s, env, (clear == 0 ? 1 : NextLevel(clear)) / 1000))})" : "");
            }
        }
        var r = new AuditResult(built.Keys.Count - 1, scan.SampleCount, hits, first, minClear, where, below, clock.Elapsed.TotalSeconds, belowCount);
        _out.WriteLine($"{name}: {r.Segments} segments, {r.Samples} samples (canonical {canonical.SampleCount}), {r.Hits} hits, " +
            $"min clearance ≥ {r.MinClearMm} mm at {r.MinWhere}; samples below 10/5/2/1 mm: {string.Join("/", AuditLevelsMm.Select(l => belowCount[l]))}; " +
            $"below margin ({marginMm} mm): {below}; {r.Seconds:F0} s");
        if (first is not null) _out.WriteLine("first hit: " + first);
        foreach (var (pair, n) in near.OrderByDescending(x => x.Value).Take(12)) _out.WriteLine($"  < 1 mm: {n,6} × {pair}");
        return r;

        static double NextLevel(double clear) => AuditLevelsMm.Where(l => l > clear).DefaultIfEmpty(10).Min();
    }

    static IReadOnlyList<CollisionContact> ContactsInflated(BambooTrussScan scan, BambooTrussMotion.TrussBuild built, double t,
        BambooTrussMotion.State s, List<CollisionObject> env, double delta)
    {
        var k = built.Keys[s.SegEnd];
        var (g, w, _) = scan.BaseAt(t);
        var world = scan.World[BambooTrussMotion.CycleIndexAt(built.Keys, built.Cycles, t)];
        var held = s.Hold == 1 ? world.HeldOffset : null;
        // Objects a gripper clamps are hugged by design (jaw plates allowed to touch, the gripper body millimetres off):
        // they are not inflated, so the clearance is to everything else.
        var clamped = $"{k.LOn}|{k.ROn}".Split('|', StringSplitOptions.RemoveEmptyEntries).ToHashSet(StringComparer.Ordinal);
        return BambooTrussCollision.Contacts(Cell, g, w, s.Q, s.GR, s.GL, held, env.Select(o => clamped.Contains(o.Name) ? o : Inflate(o, delta)).ToList(), k.LOn, k.ROn, 4);
    }

    /// <summary>Obstacle grown by δ: primitives exactly; meshes by moving each vertex δ along its (area-weighted) normal.</summary>
    static CollisionObject Inflate(CollisionObject o, double d)
    {
        switch (o.Shape)
        {
            case CollisionShape.Sphere: return CollisionObject.Sphere(o.Name, o.Pose, o.ExtentX + d);
            case CollisionShape.Box: return CollisionObject.Box(o.Name, o.Pose, o.ExtentX + d, o.ExtentY + d, o.ExtentZ + d);
            case CollisionShape.Capsule: return CollisionObject.Capsule(o.Name, o.Pose, o.ExtentX + d, o.ExtentY);
            case CollisionShape.Plane:
            {
                var m = Transforms.FromFrame(o.Pose); // free side = local +X
                return CollisionObject.Plane(o.Name, new Frame(o.Pose.X + m[0] * d, o.Pose.Y + m[4] * d, o.Pose.Z + m[8] * d, o.Pose.Qw, o.Pose.Qx, o.Pose.Qy, o.Pose.Qz));
            }
            default:
            {
                var v = o.MeshVertices!; var idx = o.MeshIndices!;
                var n = v.Select(_ => new double[3]).ToList();
                for (var i = 0; i + 2 < idx.Count; i += 3)
                {
                    var a = v[idx[i]]; var b = v[idx[i + 1]]; var c = v[idx[i + 2]];
                    var u = new[] { b[0] - a[0], b[1] - a[1], b[2] - a[2] }; var w = new[] { c[0] - a[0], c[1] - a[1], c[2] - a[2] };
                    var f = new[] { u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0] };
                    foreach (var j in new[] { idx[i], idx[i + 1], idx[i + 2] }) for (var k = 0; k < 3; k++) n[j][k] += f[k];
                }
                var nv = new List<double[]>(v.Count);
                for (var i = 0; i < v.Count; i++)
                {
                    var l = Math.Sqrt(n[i][0] * n[i][0] + n[i][1] * n[i][1] + n[i][2] * n[i][2]);
                    nv.Add(l < 1e-15 ? v[i].ToArray() : new[] { v[i][0] + n[i][0] / l * d, v[i][1] + n[i][1] / l * d, v[i][2] + n[i][2] / l * d });
                }
                return CollisionObject.Mesh(o.Name, o.Pose, nv, idx);
            }
        }
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
