using Motus.Core;
using Motus.Geometry;

namespace Motus.Construction;

/// <summary>
/// Bamboo adapter of <see cref="SegmentCertifier"/> (opt-in, <see cref="MotionConstraints.CertifiedSegments"/>): proves a
/// joint-space PTP segment of the ICD cell contact-free between its samples.
/// <para>
/// The adapter gives the motion bound of the robot, its mount parts and the held cane (<see cref="SweepBound"/>) and the
/// clear check: the exact Motus test against obstacles grown by <see cref="BambooTrussCollision.Inflate"/>.
/// </para>
/// Scope (flagged): robot ↔ environment pairs only (self and robot ↔ own-mount pairs stay sample-checked); the clearance
/// margin stays enforced at the samples, between them only exact contact is proven; mesh growing is conservative only
/// at vertices whose adjacent faces stay within ~70° of the vertex normal (inherited from Inflate).
/// </summary>
public static class BambooSegmentCertifier
{
    /// <summary>Smallest obstacle growth step (m); see <see cref="SegmentCertifier.GrowQuantumM"/>.</summary>
    public const double GrowQuantumM = SegmentCertifier.GrowQuantumM;

    /// <summary>
    /// Motion bound ε (m) of the segment qa → qb (degrees) with jaw strokes gR, gL (mm): every robot, mount and held-cane
    /// point moves at most ε·|t − s| between fractions t and s. Per body b, joint j contributes |Δq_j|·D(j, b), where
    /// D is the farthest the body's geometry reaches from joint j's axis via the chain of axis points (distances between
    /// material points, so valid for every configuration); jaw plates add their stroke. +∞ for non-revolute joints.
    /// </summary>
    public static double SweepBound(BambooIcdCell cell, string? baseGripper, IReadOnlyList<double> qaDeg, IReadOnlyList<double> qbDeg,
        double gRa, double gRb, double gLa, double gLb, Frame? held, double? heldLength = null)
    {
        var n = qaDeg.Count;
        var rad = BambooIcdCell.RadFromDeg(qaDeg);
        var mats = NewMats(n);
        cell.Fk.ComputeLinkTransformsInto(rad, mats);
        if (AxisPoints(cell, rad, mats) is not { } axis) return double.PositiveInfinity;
        // Body b (−1 = mount, n − 1 = R-grip and held cane) is chain body b + 1. Base R: the chain is fixed at its tip.
        var spheres = Spheres(cell, qaDeg, gRa, gRb, gLa, gLb, held, heldLength, mats).Select(s => (s.Body + 1, s.C, s.R));
        var delta = new double[n];
        for (var j = 0; j < n; j++) delta[j] = (qbDeg[j] - qaDeg[j]) * Math.PI / 180;
        var worst = SegmentCertifier.ChainSweepBound(axis, spheres, delta, fixedAtTip: baseGripper == "R");
        return worst + (Math.Abs(gRb - gRa) + Math.Abs(gLb - gLa)) / 2000.0;
    }

    /// <summary>
    /// True when the segment is proven free of robot ↔ environment contact between its end poses (the ends themselves
    /// are the caller's samples). Interval checks stop at <paramref name="maxDepth"/> bisections or
    /// <paramref name="maxChecks"/> pose checks; hitting either leaves the segment uncertified (false), never accepted.
    /// Parts that do not move during the segment (the clamped base gripper, its fixed jaw, a cane held by a fixed
    /// R-grip) are skipped: their contacts are the same as at the samples.
    /// </summary>
    public static bool Certify(BambooIcdCell cell, string? baseGripper, double[] baseWorld,
        IReadOnlyList<double> qaDeg, IReadOnlyList<double> qbDeg, double gRa, double gRb, double gLa, double gLb,
        Frame? held, IReadOnlyList<CollisionObject> env, string? lOn, string? rOn, double? heldLength,
        int maxDepth, int maxChecks, out int checks) =>
        SegmentCertifier.Certify(new Segment(cell, baseGripper, baseWorld, qaDeg, qbDeg, gRa, gRb, gLa, gLb, held, env, lOn, rOn, heldLength),
            maxDepth, maxChecks, out checks);

    /// <summary>Growth rounded up to 1e-4·2^k m; see <see cref="SegmentCertifier.Quantize"/>.</summary>
    public static double Quantize(double d) => SegmentCertifier.Quantize(d);

    sealed class Segment(BambooIcdCell cell, string? baseGripper, double[] baseWorld,
        IReadOnlyList<double> qaDeg, IReadOnlyList<double> qbDeg, double gRa, double gRb, double gLa, double gLb,
        Frame? held, IReadOnlyList<CollisionObject> env, string? lOn, string? rOn, double? heldLength) : ICertifiableSegment
    {
        HashSet<string>? _envNames;
        List<(string A, string B)>? _allowed;
        readonly RobotDefinition _def = RobotDefinition.Current;
        double[]? _q;

        public double SweepBound() => BambooSegmentCertifier.SweepBound(cell, baseGripper, qaDeg, qbDeg, gRa, gRb, gLa, gLb, held, heldLength);

        public bool IsClear(double m, double grow)
        {
            _envNames ??= new HashSet<string>(env.Select(o => o.Name), StringComparer.Ordinal);
            _allowed ??= BambooTrussCollision.Allowed(lOn, rOn);
            var n = qaDeg.Count;
            var q = _q ??= new double[n];
            for (var k = 0; k < n; k++) q[k] = qaDeg[k] + (qbDeg[k] - qaDeg[k]) * m;
            var grown = env.Select(o => BambooTrussCollision.Inflate(o, grow)).ToList();
            cell.SetMountViewer(BambooTrussPlanner.LWorld(baseGripper, baseWorld, q));
            return !cell.PoseContacts(q, gRa + (gRb - gRa) * m, gLa + (gLb - gLa) * m, held, grown, _allowed, 64, heldLength).Any(Counts);
        }

        bool Counts(CollisionContact c)
        {
            var aEnv = _envNames!.Contains(c.BodyA); var bEnv = _envNames.Contains(c.BodyB);
            if (aEnv == bEnv) return false;
            return !Fixed(aEnv ? c.BodyB : c.BodyA);
        }

        bool Fixed(string part)
        {
            return baseGripper == "R"
                ? gRa == gRb && (_def.R.Owns(part) || part == BambooIcdCell.Strut)
                : gLa == gLb && _def.L.Owns(part);
        }
    }

    // ---- bound internals (chain-base frame; all distances are between points fixed in one body) ----

    /// <summary>Bounding spheres (chain-base frame) tagged with their body (−1 = mount); jaw plates at both strokes.</summary>
    static List<(int Body, double[] C, double R)> Spheres(BambooIcdCell cell, IReadOnlyList<double> qDeg,
        double gRa, double gRb, double gLa, double gLb, Frame? held, double? heldLength, double[][] mats)
    {
        var n = qDeg.Count;
        var list = new List<(int, double[], double)>();
        var invBase = Transforms.Inverse(Transforms.FromFrame(cell.HangBaseFrame.Frame));
        foreach (var l in cell.Model.CollisionModel!.Links)
        {
            if (l.LinkIndex < 0) continue;
            var k = Math.Min(l.LinkIndex, n - 1);
            var g = l.LocalGeometry;
            var pi = Array.IndexOf(BambooIcdCell.RGripPlates, g.Name);
            foreach (var gr in pi < 0 ? new[] { gRa } : new[] { gRa, gRb })
            {
                var p = g.Pose;
                if (pi >= 0) p = new Frame((pi == 0 ? -1 : 1) * BambooIcdCell.PlateShift(gr), p.Y, p.Z, p.Qw, p.Qx, p.Qy, p.Qz);
                list.Add((k, Origin(Transforms.Multiply(mats[l.LinkIndex], Transforms.FromFrame(p))), LocalRadius(g)));
            }
        }
        foreach (var gl in new[] { gLa, gLb })
            foreach (var o in cell.MountObjects(gl))
                list.Add((-1, Origin(Transforms.Multiply(invBase, Transforms.FromFrame(o.Pose))), LocalRadius(o)));
        if (held is { } off)
        {
            var body = cell.HeldStrut(off, heldLength);
            var tcp = Transforms.Multiply(invBase, cell.JawWorld(qDeg));
            var m = Transforms.Multiply(Transforms.Multiply(tcp, Transforms.FromFrame(off)), Transforms.FromFrame(body.Geometry.Pose));
            list.Add((n - 1, Origin(m), LocalRadius(body.Geometry)));
        }
        return list;
    }

    /// <summary>A point on each joint axis (chain-base frame): joint j turns link j rigidly about it.</summary>
    static double[][]? AxisPoints(BambooIcdCell cell, double[] rad, double[][] mats)
    {
        var n = rad.Length;
        var axis = new double[n][];
        var pert = NewMats(n);
        const double h = 1e-3;
        for (var j = 0; j < n; j++)
        {
            var q2 = rad.ToArray(); q2[j] += h;
            cell.Fk.ComputeLinkTransformsInto(q2, pert);
            var d = Transforms.Multiply(pert[j], Transforms.Inverse(mats[j]));
            double ax = d[9] - d[6], ay = d[2] - d[8], az = d[4] - d[1];
            var s = Math.Sqrt(ax * ax + ay * ay + az * az);
            if (s < h) return null; // not a revolute joint
            ax /= s; ay /= s; az /= s;
            // ((I − R) + a aᵀ) p = t picks the axis point closest to the origin.
            var m = new double[3, 3];
            var a = new[] { ax, ay, az };
            for (var r = 0; r < 3; r++)
                for (var c = 0; c < 3; c++)
                    m[r, c] = (r == c ? 1 : 0) - d[r * 4 + c] + a[r] * a[c];
            axis[j] = Solve3(m, new[] { d[3], d[7], d[11] });
        }
        return axis;
    }

    static double[] Solve3(double[,] m, double[] t)
    {
        double Det(double[,] x) =>
            x[0, 0] * (x[1, 1] * x[2, 2] - x[1, 2] * x[2, 1]) - x[0, 1] * (x[1, 0] * x[2, 2] - x[1, 2] * x[2, 0]) + x[0, 2] * (x[1, 0] * x[2, 1] - x[1, 1] * x[2, 0]);
        var det = Det(m);
        var p = new double[3];
        for (var c = 0; c < 3; c++)
        {
            var x = (double[,])m.Clone();
            for (var r = 0; r < 3; r++) x[r, c] = t[r];
            p[c] = Det(x) / det;
        }
        return p;
    }

    /// <summary>Radius around the geometry's own origin that contains it.</summary>
    static double LocalRadius(CollisionObject g) => g.Shape switch
    {
        CollisionShape.Sphere => g.ExtentX,
        CollisionShape.Box => Math.Sqrt(g.ExtentX * g.ExtentX + g.ExtentY * g.ExtentY + g.ExtentZ * g.ExtentZ),
        CollisionShape.Capsule => g.ExtentX + g.ExtentY,
        CollisionShape.Mesh => g.MeshVertices is { Count: > 0 } v ? v.Max(p => Math.Sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])) : 0,
        _ => double.PositiveInfinity,
    };

    static double[][] NewMats(int n) { var m = new double[n][]; for (var i = 0; i < n; i++) m[i] = new double[16]; return m; }
    static double[] Origin(double[] m) => new[] { m[3], m[7], m[11] };
}
