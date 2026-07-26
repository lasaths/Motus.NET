namespace Motus.Geometry;

/// <summary>
/// Modern Robotics SO(3)/SE(3) Lie-group primitives (Lynch &amp; Park §3.2–3.3).
/// Matrices match Motus <see cref="Transforms"/>: row-major 4×4, translation at [3],[7],[11].
/// Twist / screw 6-vectors are (ωx,ωy,ωz, vx,vy,vz).
/// </summary>
public static class ScrewMath
{
    private const double NearZero = 1e-10;
    private const double NearPiTrace = 1e-6;

    public static double[,] VecToSo3(double wx, double wy, double wz) => new[,]
    {
        { 0, -wz, wy },
        { wz, 0, -wx },
        { -wy, wx, 0 }
    };

    public static void So3ToVec(double[,] so3, out double wx, out double wy, out double wz)
    {
        wx = so3[2, 1];
        wy = so3[0, 2];
        wz = so3[1, 0];
    }

    /// <summary>Rodrigues: R = I + sinθ[ω̂] + (1−cosθ)[ω̂]² with ‖ω̂‖=1, θ in radians.</summary>
    public static double[,] MatrixExp3(double wx, double wy, double wz, double theta)
    {
        var n = Math.Sqrt(wx * wx + wy * wy + wz * wz);
        if (n < NearZero || Math.Abs(theta) < NearZero)
            return Identity3();

        wx /= n; wy /= n; wz /= n;
        var w = VecToSo3(wx, wy, wz);
        var w2 = Multiply3(w, w);
        var s = Math.Sin(theta);
        var c = 1.0 - Math.Cos(theta);
        var r = Identity3();
        for (var i = 0; i < 3; i++)
        for (var j = 0; j < 3; j++)
            r[i, j] += s * w[i, j] + c * w2[i, j];
        return r;
    }

    /// <summary>Try SO(3) matrix logarithm. Fails on non-finite / non-orthogonal input (no silent NaN).</summary>
    public static bool TryMatrixLog3(double[,] r, out double wx, out double wy, out double wz, out double theta)
    {
        wx = wy = wz = theta = 0;
        if (!IsFinite3(r) || !IsRotationMatrix(r))
            return false;

        var tr = r[0, 0] + r[1, 1] + r[2, 2];
        var cos = Math.Clamp((tr - 1.0) * 0.5, -1.0, 1.0);
        theta = Math.Acos(cos);

        if (theta < NearZero)
        {
            // Identity: axis undefined; return zero rotation vector.
            return true;
        }

        if (Math.Abs(Math.PI - theta) < NearPiTrace || tr < -1.0 + NearPiTrace)
        {
            // θ ≈ π: (R − Rᵀ)/(2 sinθ) is ill-conditioned; use diagonal/off-diagonal (MR Eqs 3.56–3.60).
            theta = Math.PI;
            // Find largest diagonal for stable axis.
            var xx = (r[0, 0] + 1) * 0.5;
            var yy = (r[1, 1] + 1) * 0.5;
            var zz = (r[2, 2] + 1) * 0.5;
            if (xx >= yy && xx >= zz)
            {
                wx = Math.Sqrt(Math.Max(0, xx));
                wy = wx > NearZero ? r[0, 1] / (2 * wx) : 0;
                wz = wx > NearZero ? r[0, 2] / (2 * wx) : 0;
            }
            else if (yy >= zz)
            {
                wy = Math.Sqrt(Math.Max(0, yy));
                wx = wy > NearZero ? r[0, 1] / (2 * wy) : 0;
                wz = wy > NearZero ? r[1, 2] / (2 * wy) : 0;
            }
            else
            {
                wz = Math.Sqrt(Math.Max(0, zz));
                wx = wz > NearZero ? r[0, 2] / (2 * wz) : 0;
                wy = wz > NearZero ? r[1, 2] / (2 * wz) : 0;
            }
            Normalize3(ref wx, ref wy, ref wz);
            return true;
        }

        var denom = 2.0 * Math.Sin(theta);
        if (Math.Abs(denom) < NearZero)
            return false;

        wx = (r[2, 1] - r[1, 2]) / denom;
        wy = (r[0, 2] - r[2, 0]) / denom;
        wz = (r[1, 0] - r[0, 1]) / denom;
        Normalize3(ref wx, ref wy, ref wz);
        return true;
    }

    public static double[,] VecToSe3(double wx, double wy, double wz, double vx, double vy, double vz)
    {
        var se = new double[4, 4];
        se[0, 1] = -wz; se[0, 2] = wy; se[0, 3] = vx;
        se[1, 0] = wz; se[1, 2] = -wx; se[1, 3] = vy;
        se[2, 0] = -wy; se[2, 1] = wx; se[2, 3] = vz;
        return se;
    }

    public static void Se3ToVec(double[,] se, out double wx, out double wy, out double wz, out double vx, out double vy, out double vz)
    {
        wx = se[2, 1];
        wy = se[0, 2];
        wz = se[1, 0];
        vx = se[0, 3];
        vy = se[1, 3];
        vz = se[2, 3];
    }

    /// <summary>SE(3) exponential: e^[S]θ with screw S=(ω,v). If ‖ω‖≈0, pure translation with ‖v‖=1 and θ = distance.</summary>
    public static double[] MatrixExp6(double[] screw6, double theta)
    {
        if (screw6.Length != 6)
            throw new ArgumentException("Screw must be length 6 (ω,v).", nameof(screw6));

        var wx = screw6[0]; var wy = screw6[1]; var wz = screw6[2];
        var vx = screw6[3]; var vy = screw6[4]; var vz = screw6[5];
        var wNorm = Math.Sqrt(wx * wx + wy * wy + wz * wz);

        if (wNorm < NearZero)
        {
            // Pure translation: S = (0, v̂), θ = distance.
            var vNorm = Math.Sqrt(vx * vx + vy * vy + vz * vz);
            var t = Transforms.Identity();
            if (vNorm < NearZero || Math.Abs(theta) < NearZero)
                return t;
            t[3] = vx / vNorm * theta;
            t[7] = vy / vNorm * theta;
            t[11] = vz / vNorm * theta;
            return t;
        }

        // Normalize ω; if ‖ω‖≠1 treat θ_eff = θ·‖ω‖ (MR AxisAng packing).
        wx /= wNorm; wy /= wNorm; wz /= wNorm;
        var th = theta * wNorm;
        var r = MatrixExp3(wx, wy, wz, th);
        var wHat = VecToSo3(wx, wy, wz);
        var wHat2 = Multiply3(wHat, wHat);
        // G(θ) = Iθ + (1−cosθ)[ω] + (θ−sinθ)[ω]²
        var g = new double[3, 3];
        var c = 1.0 - Math.Cos(th);
        var ts = th - Math.Sin(th);
        for (var i = 0; i < 3; i++)
        for (var j = 0; j < 3; j++)
            g[i, j] = (i == j ? th : 0.0) + c * wHat[i, j] + ts * wHat2[i, j];

        var px = g[0, 0] * vx + g[0, 1] * vy + g[0, 2] * vz;
        var py = g[1, 0] * vx + g[1, 1] * vy + g[1, 2] * vz;
        var pz = g[2, 0] * vx + g[2, 1] * vy + g[2, 2] * vz;
        return RpToTrans(r, px, py, pz);
    }

    public static bool TryMatrixLog6(double[] t, out double[] screw6, out double theta)
    {
        screw6 = new double[6];
        theta = 0;
        if (t.Length != 16 || !IsFinite16(t))
            return false;

        var r = ExtractR(t);
        if (!IsRotationMatrix(r))
            return false;

        var px = t[3]; var py = t[7]; var pz = t[11];

        if (!TryMatrixLog3(r, out var wx, out var wy, out var wz, out theta))
            return false;

        if (theta < NearZero)
        {
            // Pure translation.
            var pNorm = Math.Sqrt(px * px + py * py + pz * pz);
            if (pNorm < NearZero)
                return true;
            theta = pNorm;
            screw6[3] = px / pNorm;
            screw6[4] = py / pNorm;
            screw6[5] = pz / pNorm;
            return true;
        }

        // v = G⁻¹(θ) p ; G⁻¹ = (1/θ)I − ½[ω] + (1/θ − ½ cot(θ/2))[ω]²  (MR Eq 3.92)
        var wHat = VecToSo3(wx, wy, wz);
        var wHat2 = Multiply3(wHat, wHat);
        var half = theta * 0.5;
        var cot = Math.Abs(half) < NearZero
            ? 0 // series: cot(x)≈1/x − x/3 → (1/θ − ½ cot(θ/2)) → θ/12
            : 1.0 / Math.Tan(half);
        var beta = Math.Abs(half) < NearZero
            ? theta / 12.0
            : 1.0 / theta - 0.5 * cot;

        var gInv = new double[3, 3];
        for (var i = 0; i < 3; i++)
        for (var j = 0; j < 3; j++)
            gInv[i, j] = (i == j ? 1.0 / theta : 0) - 0.5 * wHat[i, j] + beta * wHat2[i, j];

        var vx = gInv[0, 0] * px + gInv[0, 1] * py + gInv[0, 2] * pz;
        var vy = gInv[1, 0] * px + gInv[1, 1] * py + gInv[1, 2] * pz;
        var vz = gInv[2, 0] * px + gInv[2, 1] * py + gInv[2, 2] * pz;

        screw6[0] = wx; screw6[1] = wy; screw6[2] = wz;
        screw6[3] = vx; screw6[4] = vy; screw6[5] = vz;
        return true;
    }

    /// <summary>6×6 adjoint [Ad_T] = [[R, 0], [[p]R, R]].</summary>
    public static double[,] Adjoint(double[] t)
    {
        if (t.Length != 16)
            throw new ArgumentException("Expected 4×4 transform.", nameof(t));

        var r = ExtractR(t);
        var px = t[3]; var py = t[7]; var pz = t[11];
        var pR = Multiply3(VecToSo3(px, py, pz), r);

        var ad = new double[6, 6];
        for (var i = 0; i < 3; i++)
        for (var j = 0; j < 3; j++)
        {
            ad[i, j] = r[i, j];
            ad[i + 3, j + 3] = r[i, j];
            ad[i + 3, j] = pR[i, j];
        }
        return ad;
    }

    public static double[] AdjointMultiply(double[,] ad, double[] v6)
    {
        var r = new double[6];
        for (var i = 0; i < 6; i++)
        {
            var sum = 0.0;
            for (var j = 0; j < 6; j++)
                sum += ad[i, j] * v6[j];
            r[i] = sum;
        }
        return r;
    }

    public static double[] RpToTrans(double[,] r, double px, double py, double pz) =>
    [
        r[0, 0], r[0, 1], r[0, 2], px,
        r[1, 0], r[1, 1], r[1, 2], py,
        r[2, 0], r[2, 1], r[2, 2], pz,
        0, 0, 0, 1
    ];

    public static double[,] ExtractR(double[] t) => new[,]
    {
        { t[0], t[1], t[2] },
        { t[4], t[5], t[6] },
        { t[8], t[9], t[10] }
    };

    public static double[,] Identity3() => new[,]
    {
        { 1.0, 0.0, 0.0 },
        { 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 1.0 }
    };

    public static double[,] Multiply3(double[,] a, double[,] b)
    {
        var r = new double[3, 3];
        for (var i = 0; i < 3; i++)
        for (var j = 0; j < 3; j++)
        {
            var sum = 0.0;
            for (var k = 0; k < 3; k++)
                sum += a[i, k] * b[k, j];
            r[i, j] = sum;
        }
        return r;
    }

    public static bool IsRotationMatrix(double[,] r, double tol = 1e-6)
    {
        var rtR = Multiply3(Transpose3(r), r);
        for (var i = 0; i < 3; i++)
        for (var j = 0; j < 3; j++)
        {
            var expected = i == j ? 1.0 : 0.0;
            if (Math.Abs(rtR[i, j] - expected) > tol)
                return false;
        }
        var det =
            r[0, 0] * (r[1, 1] * r[2, 2] - r[1, 2] * r[2, 1]) -
            r[0, 1] * (r[1, 0] * r[2, 2] - r[1, 2] * r[2, 0]) +
            r[0, 2] * (r[1, 0] * r[2, 1] - r[1, 1] * r[2, 0]);
        return Math.Abs(det - 1.0) < tol * 10;
    }

    private static double[,] Transpose3(double[,] m) => new[,]
    {
        { m[0, 0], m[1, 0], m[2, 0] },
        { m[0, 1], m[1, 1], m[2, 1] },
        { m[0, 2], m[1, 2], m[2, 2] }
    };

    private static void Normalize3(ref double x, ref double y, ref double z)
    {
        var n = Math.Sqrt(x * x + y * y + z * z);
        if (n < NearZero) { x = 1; y = 0; z = 0; return; }
        x /= n; y /= n; z /= n;
    }

    private static bool IsFinite3(double[,] r)
    {
        for (var i = 0; i < 3; i++)
        for (var j = 0; j < 3; j++)
            if (!double.IsFinite(r[i, j])) return false;
        return true;
    }

    private static bool IsFinite16(double[] t)
    {
        for (var i = 0; i < 16; i++)
            if (!double.IsFinite(t[i])) return false;
        return true;
    }
}
