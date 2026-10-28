using Motus.Construction;
using Motus.Geometry;

namespace Motus.Core.Tests;

public partial class BambooTrussTests
{
    /// <summary>World points of every robot link part, mount part and the held cane at q (geometry origins, box corners, mesh vertices).</summary>
    private static List<double[]> RobotPoints(BambooIcdCell cell, string baseGripper, double[] baseWorld, double[] q, double gR, double gL, Frame? held)
    {
        cell.SetMountViewer(BambooTrussPlanner.LWorld(baseGripper, baseWorld, q));
        var pts = new List<double[]>();
        void Add(double[] m, CollisionObject g)
        {
            pts.Add(new[] { m[3], m[7], m[11] });
            if (g.Shape == CollisionShape.Box)
                foreach (var sx in new[] { -1, 1 }) foreach (var sy in new[] { -1, 1 }) foreach (var sz in new[] { -1, 1 })
                    pts.Add(Transforms.TransformPoint(m, sx * g.ExtentX, sy * g.ExtentY, sz * g.ExtentZ));
            if (g.MeshVertices is { } v)
                for (var i = 0; i < v.Count; i += 7) pts.Add(Transforms.TransformPoint(m, v[i][0], v[i][1], v[i][2]));
        }
        var baseM = Transforms.FromFrame(cell.HangBaseFrame.Frame);
        var mats = new double[q.Length][];
        for (var i = 0; i < mats.Length; i++) mats[i] = new double[16];
        cell.Fk.ComputeLinkTransformsInto(BambooIcdCell.RadFromDeg(q), mats);
        foreach (var l in cell.Model.CollisionModel!.Links)
        {
            if (l.LinkIndex < 0 || l.LinkIndex >= mats.Length) continue;
            var g = l.LocalGeometry;
            var pi = Array.IndexOf(BambooIcdCell.RGripPlates, g.Name);
            var p = pi < 0 ? g.Pose : new Frame((pi == 0 ? -1 : 1) * BambooIcdCell.PlateShift(gR), g.Pose.Y, g.Pose.Z, g.Pose.Qw, g.Pose.Qx, g.Pose.Qy, g.Pose.Qz);
            Add(Transforms.Multiply(Transforms.Multiply(baseM, mats[l.LinkIndex]), Transforms.FromFrame(p)), g);
        }
        foreach (var o in cell.MountObjects(gL)) Add(Transforms.FromFrame(o.Pose), o);
        if (held is { } off)
        {
            var body = cell.HeldStrut(off);
            Add(Transforms.Multiply(Transforms.Multiply(cell.JawWorld(q), Transforms.FromFrame(off)), Transforms.FromFrame(body.Geometry.Pose)), body.Geometry);
        }
        return pts;
    }

    /// <summary>
    /// The certifier's motion bound is a true bound: on random segments (both bases, jaws moving, cane held) no robot,
    /// mount or cane point moves more than ε·Δt between fractions.
    /// </summary>
    [Fact]
    public void SegmentCertifier_MotionBound_BoundsEveryPointsTravel()
    {
        var cell = Cell;
        var rng = new Random(7);
        var held = new Frame(0.0, 0.0, 0.03, Math.Cos(0.2), Math.Sin(0.2), 0, 0);
        var lim = cell.LimitsDeg;
        double Rand(double a, double b) => a + (b - a) * rng.NextDouble();
        var worstRatio = 0.0;
        for (var trial = 0; trial < 24; trial++)
        {
            var baseG = trial % 2 == 0 ? "L" : "R";
            var qa = lim.Select(l => Rand(l.Min, l.Max)).ToArray();
            var qb = qa.Select((x, i) => Math.Clamp(x + Rand(-40, 40), lim[i].Min, lim[i].Max)).ToArray();
            double gRa = Rand(5, 40), gRb = Rand(5, 40), gLa = Rand(5, 40), gLb = trial % 3 == 0 ? Rand(5, 40) : gLa;
            var h = trial % 4 < 2 ? held : (Frame?)null;
            var eps = BambooSegmentCertifier.SweepBound(cell, baseG, qa, qb, gRa, gRb, gLa, gLb, h);
            Assert.True(double.IsFinite(eps) && eps > 0, $"bound {eps}");
            const int steps = 64;
            List<double[]>? prev = null;
            for (var s = 0; s <= steps; s++)
            {
                var u = s / (double)steps;
                var q = qa.Select((x, i) => x + (qb[i] - x) * u).ToArray();
                var pts = RobotPoints(cell, baseG, BambooTrussLayout.StartBase, q, gRa + (gRb - gRa) * u, gLa + (gLb - gLa) * u, h);
                if (prev is not null)
                    for (var i = 0; i < pts.Count; i++)
                    {
                        var d = Math.Sqrt(Enumerable.Range(0, 3).Sum(k => (pts[i][k] - prev[i][k]) * (pts[i][k] - prev[i][k])));
                        worstRatio = Math.Max(worstRatio, d / (eps / steps));
                    }
                prev = pts;
            }
        }
        _out.WriteLine($"worst travel / bound = {worstRatio:F3}");
        Assert.True(worstRatio <= 1 + 1e-6, $"a point moved {worstRatio:F3}× the bound");
    }

    /// <summary>
    /// A small obstacle the jaw passes through mid-segment, clear at both ends: the samples at the ends miss it, the
    /// certifier does not; with the obstacle removed the same segment certifies.
    /// </summary>
    [Fact]
    public void SegmentCertifier_CatchesContactBetweenSamples_AndProvesAClearSegment()
    {
        var cell = Cell;
        var baseW = BambooTrussLayout.StartBase;
        var qa = new[] { 0.0, 20, 40, 20, 0 };
        var qb = new[] { 60.0, 20, 40, 20, 0 };
        var qm = qa.Select((x, i) => (x + qb[i]) / 2).ToArray();
        cell.SetMountViewer(BambooTrussPlanner.LWorld("L", baseW, qm));
        // Probe at the centre of a distal box part (inside the robot at qm).
        var part = cell.Model.CollisionModel!.Links.Last(l => l.LinkIndex >= 0 && l.LinkIndex < qm.Length && l.LocalGeometry.Shape == CollisionShape.Box
            && Array.IndexOf(BambooIcdCell.RGripPlates, l.LocalGeometry.Name) < 0);
        var mats = new double[qm.Length][];
        for (var i = 0; i < mats.Length; i++) mats[i] = new double[16];
        cell.Fk.ComputeLinkTransformsInto(BambooIcdCell.RadFromDeg(qm), mats);
        var jaw = Transforms.Multiply(Transforms.Multiply(Transforms.FromFrame(cell.HangBaseFrame.Frame), mats[part.LinkIndex]), Transforms.FromFrame(part.LocalGeometry.Pose));
        var ball = CollisionObject.Sphere("probe", new Frame(jaw[3], jaw[7], jaw[11], 1, 0, 0, 0), 0.004);
        var env = new List<CollisionObject> { ball };
        foreach (var q in new[] { qa, qb })
            Assert.Empty(BambooTrussCollision.Contacts(cell, "L", baseW, q, 30, 30, null, env, null, null, 8)
                .Where(c => c.BodyA == "probe" || c.BodyB == "probe"));
        Assert.NotEmpty(BambooTrussCollision.Contacts(cell, "L", baseW, qm, 30, 30, null, env, null, null, 8)
            .Where(c => c.BodyA == "probe" || c.BodyB == "probe"));
        Assert.False(BambooSegmentCertifier.Certify(cell, "L", baseW, qa, qb, 30, 30, 30, 30, null, env, null, null, null, 18, 20000, out var hitChecks));
        var far = new List<CollisionObject> { CollisionObject.Sphere("probe", new Frame(jaw[3], jaw[7], jaw[11] + 3.0, 1, 0, 0, 0), 0.004) };
        Assert.True(BambooSegmentCertifier.Certify(cell, "L", baseW, qa, qb, 30, 30, 30, 30, null, far, null, null, null, 18, 20000, out var clearChecks));
        _out.WriteLine($"checks: hit {hitChecks}, clear {clearChecks}");
    }

    /// <summary>Every key segment of the shipped Warren plan is proven contact-free between its samples.</summary>
    [Fact]
    public void FullPath_EverySegmentIsCertified()
    {
        var scan = Scan(Build());
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var bad = scan.UncertifiedSegments();
        _out.WriteLine($"uncertified {bad.Count} in {sw.Elapsed.TotalSeconds:F1} s: {string.Join(", ", bad)}");
        Assert.Empty(bad);
    }
}

public partial class BambooTrussTests
{
    /// <summary>
    /// With certified segments on, the planner still builds the Warren truss 7/7 and the scan finds it clear.
    /// About 16 min on an 8-core box, so it runs only when MOTUS_CERTIFY_BUILD=1.
    /// </summary>
    [Fact]
    [Trait("Category", "Slow")]
    public void Build_WithCertifiedSegments_IsCompleteAndClear()
    {
        if (Environment.GetEnvironmentVariable("MOTUS_CERTIFY_BUILD") != "1") return;
        var mc = MotionConstraints.Current;
        try
        {
            MotionConstraints.Current = mc with { CertifiedSegments = true };
            BambooTrussMotion.UseCell(Cell);
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var built = BambooTrussMotion.Build();
            _out.WriteLine($"build {sw.Elapsed.TotalSeconds:F0} s");
            Assert.True(string.IsNullOrEmpty(built.Error), built.Error);
            Assert.Equal(BambooTrussLayout.Members.Count, built.Cycles.Count(c => BambooTrussLayout.Members.Any(m => m.Id == c.MemberId)));
            var scan = new BambooTrussScan(Cell, built.Keys, built.Cycles, BambooTrussLayout.Members, BambooTrussLayout.JawClosedMm).RunToEnd();
            Assert.True(scan.FirstHit is null, scan.FirstHit?.Describe());
            Assert.Empty(scan.UncertifiedSegments());
        }
        finally { MotionConstraints.Current = mc; }
    }
}
