namespace Motus.Construction;

/// <summary>
/// Full-pose kinematics of the two-gripper bamboo arm for brachiation. Either gripper can be the base:
/// <list type="bullet">
/// <item><b>L-base</b>: L-grip (URDF <c>mount</c>) clamped at world frame <c>W_L</c>; R-grip = <c>W_L · T_LR(q)</c>.</item>
/// <item><b>R-base</b>: R-grip (URDF <c>tool0</c>) clamped at world frame <c>W_R</c>; L-grip = <c>W_R · T_LR(q)⁻¹</c>.</item>
/// </list>
/// <c>T_LR(q)</c> is the URDF chain mount → tool0 (joint axes Y, X, X, X, Y; offsets along −Y), identical to
/// Motus FK and <see cref="BambooIcdMotion.FkJaw"/>. Matrices are 4×4 row-major in <b>viewer</b> coordinates (Y-up).
/// <para>
/// Grip frame of either gripper: origin = grasp centre (between the jaw plates), local Z = jaw slot axis (the cane
/// axis when clamped), local X = jaw opening direction, local +Y = towards the arm (the gripper backs off along +Y).
/// A true clamp on a cane means: origin on the cane axis and local Z ∥ cane axis. Rotation about the cane is free,
/// so a grip target is 5 constraints for 5 joints (discrete IK solutions).
/// </para>
/// </summary>
public static class BambooGripKinematics
{
    /// <summary>Grip position tolerance at every clamp / grasp (metres).</summary>
    public const double GripPosTolM = 0.0005;
    /// <summary>Grip axis tolerance at every clamp / grasp (degrees): slot axis vs cane axis.</summary>
    public const double GripAxisTolDeg = 0.5;

    const double Deg = Math.PI / 180.0;
    static readonly double[] Seg = BambooIcdMotion.Seg;
    static readonly char[] Axes = { 'Y', 'X', 'X', 'X', 'Y' };

    public enum Gripper { L, R }

    // ---------- 4×4 helpers (row-major) ----------
    public static double[] Identity() => new double[] { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

    public static double[] Mul(double[] a, double[] b)
    {
        var c = new double[16];
        for (var r = 0; r < 4; r++)
            for (var k = 0; k < 4; k++)
            {
                var s = 0.0;
                for (var j = 0; j < 4; j++) s += a[r * 4 + j] * b[j * 4 + k];
                c[r * 4 + k] = s;
            }
        return c;
    }

    /// <summary>Rigid inverse.</summary>
    public static double[] Inv(double[] m)
    {
        var r = new double[16];
        for (var i = 0; i < 3; i++)
            for (var j = 0; j < 3; j++) r[i * 4 + j] = m[j * 4 + i];
        for (var i = 0; i < 3; i++)
            r[i * 4 + 3] = -(r[i * 4] * m[3] + r[i * 4 + 1] * m[7] + r[i * 4 + 2] * m[11]);
        r[15] = 1;
        return r;
    }

    public static double[] Trans(double x, double y, double z)
    {
        var m = Identity(); m[3] = x; m[7] = y; m[11] = z; return m;
    }

    public static double[] Rot(char axis, double rad)
    {
        var c = Math.Cos(rad); var s = Math.Sin(rad);
        var m = Identity();
        switch (axis)
        {
            case 'X': m[5] = c; m[6] = -s; m[9] = s; m[10] = c; break;
            case 'Y': m[0] = c; m[2] = s; m[8] = -s; m[10] = c; break;
            default: m[0] = c; m[1] = -s; m[4] = s; m[5] = c; break;
        }
        return m;
    }

    public static (double X, double Y, double Z) Pos(double[] m) => (m[3], m[7], m[11]);
    public static (double X, double Y, double Z) ColX(double[] m) => (m[0], m[4], m[8]);
    public static (double X, double Y, double Z) ColY(double[] m) => (m[1], m[5], m[9]);
    public static (double X, double Y, double Z) ColZ(double[] m) => (m[2], m[6], m[10]);

    /// <summary>Frame with origin <paramref name="p"/> and the given (orthonormal) columns.</summary>
    public static double[] FromCols((double X, double Y, double Z) x, (double X, double Y, double Z) y, (double X, double Y, double Z) z, (double X, double Y, double Z) p) =>
        new[] { x.X, y.X, z.X, p.X, x.Y, y.Y, z.Y, p.Y, x.Z, y.Z, z.Z, p.Z, 0, 0, 0, 1 };

    /// <summary>Mount (L-grip) → tool0 (R-grip) for joint angles in degrees.</summary>
    public static double[] Tlr(IReadOnlyList<double> qDeg)
    {
        // Columns of R (c0, c1, c2) and origin p; R·Rx / R·Ry update two columns in place.
        double c0x = 1, c0y = 0, c0z = 0, c1x = 0, c1y = 1, c1z = 0, c2x = 0, c2y = 0, c2z = 1;
        double px = 0, py = 0, pz = 0;
        for (var i = 0; i < 5; i++)
        {
            px -= Seg[i] * c1x; py -= Seg[i] * c1y; pz -= Seg[i] * c1z;
            var a = qDeg[i] * Deg; var c = Math.Cos(a); var s = Math.Sin(a);
            if (Axes[i] == 'X')
            {
                double n1x = c * c1x + s * c2x, n1y = c * c1y + s * c2y, n1z = c * c1z + s * c2z;
                double n2x = -s * c1x + c * c2x, n2y = -s * c1y + c * c2y, n2z = -s * c1z + c * c2z;
                c1x = n1x; c1y = n1y; c1z = n1z; c2x = n2x; c2y = n2y; c2z = n2z;
            }
            else
            {
                double n0x = c * c0x - s * c2x, n0y = c * c0y - s * c2y, n0z = c * c0z - s * c2z;
                double n2x = s * c0x + c * c2x, n2y = s * c0y + c * c2y, n2z = s * c0z + c * c2z;
                c0x = n0x; c0y = n0y; c0z = n0z; c2x = n2x; c2y = n2y; c2z = n2z;
            }
        }
        px -= Seg[5] * c1x; py -= Seg[5] * c1y; pz -= Seg[5] * c1z;
        return new[] { c0x, c1x, c2x, px, c0y, c1y, c2y, py, c0z, c1z, c2z, pz, 0, 0, 0, 1 };
    }

    /// <summary>World poses of both grippers given which one is the clamped base and its world frame.</summary>
    public static (double[] L, double[] R) Grippers(Gripper baseGripper, double[] baseWorld, IReadOnlyList<double> qDeg)
    {
        var t = Tlr(qDeg);
        return baseGripper == Gripper.L
            ? (baseWorld, Mul(baseWorld, t))
            : (Mul(baseWorld, Inv(t)), baseWorld);
    }

    /// <summary>World pose of the free (moving) gripper.</summary>
    public static double[] Free(Gripper baseGripper, double[] baseWorld, IReadOnlyList<double> qDeg)
    {
        var (l, r) = Grippers(baseGripper, baseWorld, qDeg);
        return baseGripper == Gripper.L ? r : l;
    }

    /// <summary>A cane grip target: point on the cane axis and the (unit) cane axis.</summary>
    public readonly record struct GripTarget(double X, double Y, double Z, double Ax, double Ay, double Az, string On)
    {
        public (double X, double Y, double Z) P => (X, Y, Z);
        public (double X, double Y, double Z) Axis => (Ax, Ay, Az);
        public GripTarget Offset(double dx, double dy, double dz) => this with { X = X + dx, Y = Y + dy, Z = Z + dz };
    }

    /// <summary>Grip target on a strut placement at signed distance <paramref name="s"/> from its centre along its axis.</summary>
    public static GripTarget OnStrut(BambooIcdCell.StrutPlacement p, double s, string on)
    {
        var a = p.Axis;
        return new GripTarget(p.X + a.X * s, p.Y + a.Y * s, p.Z + a.Z * s, a.X, a.Y, a.Z, on);
    }

    /// <summary>Position error (m) and slot-axis error (deg, sign-free) of a gripper frame vs a target.</summary>
    public static (double PosM, double AxisDeg) Residual(double[] grip, GripTarget t)
    {
        var dx = grip[3] - t.X; var dy = grip[7] - t.Y; var dz = grip[11] - t.Z;
        var dot = Math.Abs(grip[2] * t.Ax + grip[6] * t.Ay + grip[10] * t.Az);
        return (Math.Sqrt(dx * dx + dy * dy + dz * dz), Math.Acos(Math.Min(1, dot)) / Deg);
    }

    public static bool WithinLimits(IReadOnlyList<double> q)
    {
        for (var i = 0; i < 5; i++)
            if (q[i] < BambooIcdMotion.LimitsDeg[i].Min - 1e-6 || q[i] > BambooIcdMotion.LimitsDeg[i].Max + 1e-6) return false;
        return true;
    }

    /// <summary>
    /// Numerical grip IK (damped least squares, joint limits enforced, multi-seed). Drives the free gripper's origin
    /// onto <paramref name="target"/> and its slot axis parallel to the cane axis. Returns distinct solutions within
    /// <see cref="GripPosTolM"/> / <see cref="GripAxisTolDeg"/> (converged far tighter in practice), nearest-seed first.
    /// <paramref name="backoff"/> &gt; 0 places the grasp centre that far along the gripper's own +Y (pre-grasp /
    /// retreat pose: jaws just off the cane, slot still aligned with it).
    /// </summary>
    /// <summary>Multiplier on the random IK seeds (probe experiments).</summary>
    public static int SeedScale { get; set; } = 1;

    public static List<double[]> SolveGrip(Gripper baseGripper, double[] baseWorld, GripTarget target,
        IEnumerable<double[]>? seeds = null, int randomSeeds = 24, double backoff = 0, int rngSeed = 7, int axisSign = 0)
    {
        var all = new List<double[]>();
        if (seeds is not null) all.AddRange(seeds.Select(s => s.ToArray()));
        var rng = new Random(rngSeed);
        for (var i = 0; i < randomSeeds * SeedScale; i++)
        {
            var q = new double[5];
            for (var k = 0; k < 5; k++)
            {
                var (lo, hi) = BambooIcdMotion.LimitsDeg[k];
                q[k] = lo + (hi - lo) * rng.NextDouble();
            }
            all.Add(q);
        }
        var sols = new List<double[]>();
        foreach (var seed in all)
        {
            var q = Refine(baseGripper, baseWorld, target, seed, backoff, axisSign);
            if (q is null) continue;
            if (sols.Any(s => JointDist(s, q) < 2.0)) continue;
            sols.Add(q);
        }
        return sols;
    }

    public static double JointDist(IReadOnlyList<double> a, IReadOnlyList<double> b)
    {
        var m = 0.0;
        for (var k = 0; k < 5; k++) m = Math.Max(m, Math.Abs(a[k] - b[k]));
        return m;
    }

    /// <summary>Residual of the free gripper (with optional back-off along its +Y) against the target.</summary>
    public static (double PosM, double AxisDeg) GripResidual(Gripper baseGripper, double[] baseWorld, GripTarget target, IReadOnlyList<double> q, double backoff = 0)
    {
        var g = Free(baseGripper, baseWorld, q);
        if (backoff != 0)
            g = Mul(g, Trans(0, -backoff, 0));
        return Residual(g, target);
    }

    static double[] ErrorVec(Gripper bg, double[] bw, GripTarget t, double[] q, double backoff, int axisSign)
    {
        var g = Free(bg, bw, q);
        if (backoff != 0) g = Mul(g, Trans(0, -backoff, 0));
        var zx = g[2]; var zy = g[6]; var zz = g[10];
        var dot = zx * t.Ax + zy * t.Ay + zz * t.Az;
        var sgn = axisSign != 0 ? axisSign : dot >= 0 ? 1.0 : -1.0;
        var ux = t.Ax * sgn; var uy = t.Ay * sgn; var uz = t.Az * sgn;
        const double w = 0.1; // metres per radian
        return new[]
        {
            t.X - g[3], t.Y - g[7], t.Z - g[11],
            w * (zy * uz - zz * uy), w * (zz * ux - zx * uz), w * (zx * uy - zy * ux)
        };
    }

    static double[]? Refine(Gripper bg, double[] bw, GripTarget t, double[] seed, double backoff, int axisSign)
    {
        var q = seed.ToArray();
        for (var k = 0; k < 5; k++) q[k] = Math.Clamp(q[k], BambooIcdMotion.LimitsDeg[k].Min, BambooIcdMotion.LimitsDeg[k].Max);
        var lambda = 0.01;
        var e = ErrorVec(bg, bw, t, q, backoff, axisSign);
        var en = Norm(e);
        for (var it = 0; it < 120; it++)
        {
            var (pe, ae) = GripResidual(bg, bw, t, q, backoff);
            if (pe < 1e-7 && ae < 1e-5) break;
            // Numerical Jacobian (6×5) of the error w.r.t. q (radians).
            var J = new double[6, 5];
            const double h = 1e-6;
            for (var k = 0; k < 5; k++)
            {
                var qp = q.ToArray(); qp[k] += h / Deg;
                var ep = ErrorVec(bg, bw, t, qp, backoff, axisSign);
                for (var r = 0; r < 6; r++) J[r, k] = -(ep[r] - e[r]) / h;
            }
            // (JᵀJ + λ²I) dq = Jᵀ e
            var A = new double[5, 5]; var b = new double[5];
            for (var i = 0; i < 5; i++)
            {
                for (var j = 0; j < 5; j++)
                {
                    var s = 0.0;
                    for (var r = 0; r < 6; r++) s += J[r, i] * J[r, j];
                    A[i, j] = s + (i == j ? lambda * lambda : 0);
                }
                var sb = 0.0;
                for (var r = 0; r < 6; r++) sb += J[r, i] * e[r];
                b[i] = sb;
            }
            var dq = Solve5(A, b);
            if (dq is null) return null;
            var maxStep = 0.0;
            for (var k = 0; k < 5; k++) maxStep = Math.Max(maxStep, Math.Abs(dq[k]));
            var scale = maxStep > 0.35 ? 0.35 / maxStep : 1.0;
            var qn = q.ToArray();
            for (var k = 0; k < 5; k++)
                qn[k] = Math.Clamp(q[k] + dq[k] * scale / Deg, BambooIcdMotion.LimitsDeg[k].Min, BambooIcdMotion.LimitsDeg[k].Max);
            var en2 = ErrorVec(bg, bw, t, qn, backoff, axisSign);
            var nn = Norm(en2);
            if (nn < en)
            {
                q = qn; e = en2; en = nn;
                lambda = Math.Max(1e-5, lambda * 0.5);
            }
            else
            {
                lambda *= 4;
                if (lambda > 1e3) break;
            }
        }
        var (p, a) = GripResidual(bg, bw, t, q, backoff);
        if (p > GripPosTolM || a > GripAxisTolDeg) return null;
        if (axisSign != 0)
        {
            var g = Free(bg, bw, q);
            if ((g[2] * t.Ax + g[6] * t.Ay + g[10] * t.Az) * axisSign <= 0) return null;
        }
        return WithinLimits(q) ? q : null;
    }

    static double Norm(double[] v) { var s = 0.0; foreach (var x in v) s += x * x; return Math.Sqrt(s); }

    static double[]? Solve5(double[,] A, double[] b)
    {
        const int n = 5;
        var M = new double[n, n + 1];
        for (var i = 0; i < n; i++) { for (var j = 0; j < n; j++) M[i, j] = A[i, j]; M[i, n] = b[i]; }
        for (var c = 0; c < n; c++)
        {
            var piv = c;
            for (var r = c + 1; r < n; r++) if (Math.Abs(M[r, c]) > Math.Abs(M[piv, c])) piv = r;
            if (Math.Abs(M[piv, c]) < 1e-14) return null;
            if (piv != c) for (var j = 0; j <= n; j++) (M[c, j], M[piv, j]) = (M[piv, j], M[c, j]);
            for (var r = 0; r < n; r++)
            {
                if (r == c) continue;
                var f = M[r, c] / M[c, c];
                for (var j = c; j <= n; j++) M[r, j] -= f * M[c, j];
            }
        }
        var x = new double[n];
        for (var i = 0; i < n; i++) x[i] = M[i, n] / M[i, i];
        return x;
    }

    /// <summary>
    /// Clamp frame of the L-grip on a vertical anchor stake: slot axis (local Z) along world +Y, arm (local −Y)
    /// pointing horizontally at heading <paramref name="headingDeg"/> (0 = +Z, 90 = +X).
    /// </summary>
    public static double[] StakeClampFrame(double x, double y, double z, double headingDeg)
    {
        var h = headingDeg * Deg;
        // local −Y = arm direction (horizontal) ⇒ local +Y = −arm.
        var arm = (Math.Sin(h), 0.0, Math.Cos(h));
        var ly = (-arm.Item1, 0.0, -arm.Item3);
        var lz = (0.0, 1.0, 0.0);
        // X = Y × Z
        var lx = (ly.Item2 * lz.Item3 - ly.Item3 * lz.Item2, ly.Item3 * lz.Item1 - ly.Item1 * lz.Item3, ly.Item1 * lz.Item2 - ly.Item2 * lz.Item1);
        return FromCols(lx, ly, lz, (x, y, z));
    }
}
