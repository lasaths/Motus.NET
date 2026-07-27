using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Immutable body-pose policy for Walk. Call <see cref="CreateSession"/> per Walk build so EMA state
/// is not shared under GH fan-out.
/// </summary>
public interface IBodyPoseSolver
{
    string MethodId { get; }
    IBodyPoseSession CreateSession();
}

/// <summary>Mutable per-Walk session (prev frame / blend state).</summary>
public interface IBodyPoseSession
{
    bool TryPose(
        double pathX,
        double pathY,
        double pathYawRad,
        IReadOnlyList<Vec3> nominalFootBody,
        IReadOnlyList<Vec3>? hipBody,
        Func<double, double, double> terrain,
        bool isFirstSample,
        out Frame frame,
        out string error);
}

/// <summary>Body XY/yaw follow the path; Z = terrain(path) + clearance (m).</summary>
public sealed class PathFollowBodyPose : IBodyPoseSolver
{
    public PathFollowBodyPose(double clearanceMeters = 0)
    {
        if (!double.IsFinite(clearanceMeters))
            throw new ArgumentException("Clearance must be finite (m).", nameof(clearanceMeters));
        ClearanceMeters = clearanceMeters;
    }

    public double ClearanceMeters { get; }
    public string MethodId => $"PathFollow(clearance={ClearanceMeters:F4}m)";

    public IBodyPoseSession CreateSession() => new Session(ClearanceMeters);

    private sealed class Session(double clearance) : IBodyPoseSession
    {
        public bool TryPose(
            double pathX, double pathY, double pathYawRad,
            IReadOnlyList<Vec3> nominalFootBody,
            IReadOnlyList<Vec3>? hipBody,
            Func<double, double, double> terrain,
            bool isFirstSample,
            out Frame frame,
            out string error)
        {
            frame = default;
            var zGround = terrain(pathX, pathY);
            if (!double.IsFinite(zGround))
            {
                error = $"Terrain height non-finite at ({pathX:F3}, {pathY:F3}) m.";
                return false;
            }

            frame = new MobilityModel.HolonomicSE2(pathX, pathY, pathYawRad, zGround + clearance).BaseFrame;
            error = "";
            return true;
        }
    }
}

/// <summary>
/// Body rides a smoothed support plane fitted through terrain under nominal feet
/// (stance-weighted; <see cref="StanceWeightMin"/>=1 → legacy equal weights).
/// </summary>
public sealed class TerrainSupportBodyPose : IBodyPoseSolver
{
    public TerrainSupportBodyPose(
        double clearanceMeters = 0,
        double stanceWeightMin = 1.0,
        double bodyBlend = 0.28,
        double maxDzPerSample = 0.004)
    {
        if (!double.IsFinite(clearanceMeters))
            throw new ArgumentException("Clearance must be finite (m).", nameof(clearanceMeters));
        if (!double.IsFinite(stanceWeightMin) || stanceWeightMin < 0)
            throw new ArgumentException("StanceWeightMin must be finite and ≥ 0.", nameof(stanceWeightMin));
        ClearanceMeters = clearanceMeters;
        StanceWeightMin = stanceWeightMin;
        BodyBlend = bodyBlend;
        MaxDzPerSample = maxDzPerSample;
    }

    /// <summary>Body-origin offset along fitted plane +Z (m). 0 = origin on support (legacy).</summary>
    public double ClearanceMeters { get; }
    /// <summary>Minimum contact weight; 1 = equal weights (legacy unweighted plane).</summary>
    public double StanceWeightMin { get; }
    public double BodyBlend { get; }
    public double MaxDzPerSample { get; }
    public string MethodId =>
        $"TerrainSupport(clearance={ClearanceMeters:F4}m,w_min={StanceWeightMin:F2})";

    public IBodyPoseSession CreateSession() =>
        new Session(ClearanceMeters, StanceWeightMin, BodyBlend, MaxDzPerSample);

    /// <summary>Least-squares / PCA-ish plane through support points (equal weight).</summary>
    public static Frame FrameFromSupportPoints(
        double px, double py, double pathYaw, IReadOnlyList<Vec3> pts, double clearanceMeters = 0)
    {
        if (pts.Count < 3 || !TryFitHeightPlane(pts, null, out var a, out var b, out var c))
        {
            var z = 0.0;
            for (var i = 0; i < pts.Count; i++)
                z += pts[i].Z;
            z = pts.Count > 0 ? z / pts.Count : 0;
            return new MobilityModel.HolonomicSE2(px, py, pathYaw, z + clearanceMeters).BaseFrame;
        }

        var zBody = a * px + b * py + c;
        if (!double.IsFinite(zBody))
            return new MobilityModel.HolonomicSE2(px, py, pathYaw, pts[0].Z + clearanceMeters).BaseFrame;

        var nx = -a;
        var ny = -b;
        var nz = 1.0;
        var nLen = Math.Sqrt(nx * nx + ny * ny + nz * nz);
        nx /= nLen;
        ny /= nLen;
        nz /= nLen;
        if (nz < 0.45)
            return new MobilityModel.HolonomicSE2(px, py, pathYaw, zBody + clearanceMeters).BaseFrame;

        // Offset body origin along support normal by clearance.
        var ox = px + nx * clearanceMeters;
        var oy = py + ny * clearanceMeters;
        var oz = zBody + nz * clearanceMeters;

        var hx = Math.Cos(pathYaw);
        var hy = Math.Sin(pathYaw);
        var hd = hx * nx + hy * ny;
        var xx = hx - nx * hd;
        var xy = hy - ny * hd;
        var xz = -nz * hd;
        var xLen = Math.Sqrt(xx * xx + xy * xy + xz * xz);
        if (xLen < 1e-9)
            return new MobilityModel.HolonomicSE2(ox, oy, pathYaw, oz).BaseFrame;
        xx /= xLen;
        xy /= xLen;
        xz /= xLen;

        var yx = ny * xz - nz * xy;
        var yy = nz * xx - nx * xz;
        var yz = nx * xy - ny * xx;

        return Transforms.ToFrame(
        [
            xx, yx, nx, ox,
            xy, yy, ny, oy,
            xz, yz, nz, oz,
            0, 0, 0, 1
        ]);
    }

    public static Frame SmoothBodyToward(
        Frame prev, Frame desired, double px, double py, double pathYaw,
        double blend, double maxDz)
    {
        var z = prev.Z + Math.Clamp(desired.Z - prev.Z, -maxDz, maxDz);
        z += blend * (desired.Z - z);

        var t = Math.Clamp(blend, 0, 1);
        SlerpQuat(
            prev.Qw, prev.Qx, prev.Qy, prev.Qz,
            desired.Qw, desired.Qx, desired.Qy, desired.Qz,
            t,
            out var qw, out var qx, out var qy, out var qz);
        var blended = new Frame(px, py, z, qw, qx, qy, qz);

        var m = Transforms.FromFrame(blended);
        var nx = m[2];
        var ny = m[6];
        var nz = m[10];
        var nLen = Math.Sqrt(nx * nx + ny * ny + nz * nz);
        if (nLen < 1e-9 || nz / nLen < 0.45)
            return new MobilityModel.HolonomicSE2(px, py, pathYaw, z).BaseFrame;
        nx /= nLen;
        ny /= nLen;
        nz /= nLen;

        var hx = Math.Cos(pathYaw);
        var hy = Math.Sin(pathYaw);
        var hd = hx * nx + hy * ny;
        var xx = hx - nx * hd;
        var xy = hy - ny * hd;
        var xz = -nz * hd;
        var xLen = Math.Sqrt(xx * xx + xy * xy + xz * xz);
        if (xLen < 1e-9)
            return new MobilityModel.HolonomicSE2(px, py, pathYaw, z).BaseFrame;
        xx /= xLen;
        xy /= xLen;
        xz /= xLen;
        var yx = ny * xz - nz * xy;
        var yy = nz * xx - nx * xz;
        var yz = nx * xy - ny * xx;
        return Transforms.ToFrame(
        [
            xx, yx, nx, px,
            xy, yy, ny, py,
            xz, yz, nz, z,
            0, 0, 0, 1
        ]);
    }

    internal static bool TryFitHeightPlane(
        IReadOnlyList<Vec3> pts, IReadOnlyList<double>? weights,
        out double a, out double b, out double c)
    {
        a = b = c = 0;
        var n = pts.Count;
        if (n < 3)
            return false;

        double sx = 0, sy = 0, sz = 0, sxx = 0, syy = 0, sxy = 0, sxz = 0, syz = 0, sw = 0;
        for (var i = 0; i < n; i++)
        {
            var w = weights is null ? 1.0 : weights[i];
            if (!(w > 0) || !double.IsFinite(w))
                continue;
            var p = pts[i];
            sx += w * p.X;
            sy += w * p.Y;
            sz += w * p.Z;
            sxx += w * p.X * p.X;
            syy += w * p.Y * p.Y;
            sxy += w * p.X * p.Y;
            sxz += w * p.X * p.Z;
            syz += w * p.Y * p.Z;
            sw += w;
        }

        if (sw < 1e-12)
            return false;

        var m00 = sxx;
        var m01 = sxy;
        var m02 = sx;
        var m10 = sxy;
        var m11 = syy;
        var m12 = sy;
        var m20 = sx;
        var m21 = sy;
        var m22 = sw;
        var d0 = sxz;
        var d1 = syz;
        var d2 = sz;

        var det =
            m00 * (m11 * m22 - m12 * m21) -
            m01 * (m10 * m22 - m12 * m20) +
            m02 * (m10 * m21 - m11 * m20);
        if (Math.Abs(det) < 1e-14)
            return false;

        a = (
            d0 * (m11 * m22 - m12 * m21) -
            m01 * (d1 * m22 - m12 * d2) +
            m02 * (d1 * m21 - m11 * d2)) / det;
        b = (
            m00 * (d1 * m22 - m12 * d2) -
            d0 * (m10 * m22 - m12 * m20) +
            m02 * (m10 * d2 - d1 * m20)) / det;
        c = (
            m00 * (m11 * d2 - d1 * m21) -
            m01 * (m10 * d2 - d1 * m20) +
            d0 * (m10 * m21 - m11 * m20)) / det;
        return double.IsFinite(a) && double.IsFinite(b) && double.IsFinite(c);
    }

    private static void SlerpQuat(
        double aw, double ax, double ay, double az,
        double bw, double bx, double by, double bz,
        double t,
        out double w, out double x, out double y, out double z)
    {
        var dot = aw * bw + ax * bx + ay * by + az * bz;
        if (dot < 0)
        {
            bw = -bw; bx = -bx; by = -by; bz = -bz;
            dot = -dot;
        }

        if (dot > 0.9995)
        {
            w = aw + t * (bw - aw);
            x = ax + t * (bx - ax);
            y = ay + t * (by - ay);
            z = az + t * (bz - az);
            var n = Math.Sqrt(w * w + x * x + y * y + z * z);
            if (n < 1e-15) { w = 1; x = y = z = 0; return; }
            w /= n; x /= n; y /= n; z /= n;
            return;
        }

        var theta0 = Math.Acos(Math.Clamp(dot, -1, 1));
        var theta = theta0 * t;
        var s0 = Math.Sin(theta0);
        var s1 = Math.Sin(theta0 - theta) / s0;
        var s2 = Math.Sin(theta) / s0;
        w = s1 * aw + s2 * bw;
        x = s1 * ax + s2 * bx;
        y = s1 * ay + s2 * by;
        z = s1 * az + s2 * bz;
    }

    private sealed class Session : IBodyPoseSession
    {
        private readonly double _clearance;
        private readonly double _wMin;
        private readonly double _blend;
        private readonly double _maxDz;
        private Frame _prev;

        public Session(double clearance, double wMin, double blend, double maxDz)
        {
            _clearance = clearance;
            _wMin = wMin;
            _blend = blend;
            _maxDz = maxDz;
        }

        public bool TryPose(
            double pathX, double pathY, double pathYawRad,
            IReadOnlyList<Vec3> nominalFootBody,
            IReadOnlyList<Vec3>? hipBody,
            Func<double, double, double> terrain,
            bool isFirstSample,
            out Frame frame,
            out string error)
        {
            frame = default;
            var flat = new MobilityModel.HolonomicSE2(pathX, pathY, pathYawRad, 0).BaseFrame;
            var n = nominalFootBody.Count;
            var pts = new Vec3[n];
            double[]? weights = _wMin < 1.0 - 1e-12 ? new double[n] : null;
            for (var leg = 0; leg < n; leg++)
            {
                var w = BodyToWorld(nominalFootBody[leg], flat);
                var z = terrain(w.X, w.Y);
                if (!double.IsFinite(z))
                {
                    error = $"Terrain height non-finite at ({w.X:F3}, {w.Y:F3}) m.";
                    return false;
                }
                pts[leg] = new Vec3(w.X, w.Y, z);
                if (weights is not null)
                    weights[leg] = Math.Max(_wMin, 1.0); // ponytail: equal weights until stance mask is threaded in
            }

            // Weighted fit when w_min≠1 (future stance mask); equal weights → legacy path.
            if (weights is not null && TryFitHeightPlane(pts, weights, out var a, out var b, out var c))
            {
                var zBody = a * pathX + b * pathY + c;
                var desired = FrameFromSupportPoints(pathX, pathY, pathYawRad, pts, _clearance);
                // Keep FrameFromSupportPoints as source of orientation; z already includes clearance.
                _ = zBody;
                frame = isFirstSample
                    ? desired
                    : SmoothBodyToward(_prev, desired, pathX, pathY, pathYawRad, _blend, _maxDz);
            }
            else
            {
                var desired = FrameFromSupportPoints(pathX, pathY, pathYawRad, pts, _clearance);
                frame = isFirstSample
                    ? desired
                    : SmoothBodyToward(_prev, desired, pathX, pathY, pathYawRad, _blend, _maxDz);
            }

            _prev = frame;
            error = "";
            return true;
        }

        private static Vec3 BodyToWorld(Vec3 body, Frame baseFrame)
        {
            var m = Transforms.FromFrame(baseFrame);
            Transforms.TransformPointInto(m, body.X, body.Y, body.Z, out var x, out var y, out var z);
            return new Vec3(x, y, z);
        }
    }
}
