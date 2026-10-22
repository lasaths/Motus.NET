using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Exact narrow-phase tests for convex primitives used by the mesh/primitive collision path:
/// OBB–OBB and triangle–OBB separating-axis tests, closest-point sphere–triangle, and
/// half-space tests. Matrices are Motus row-major 4×4 (rotation columns = local axes).
/// Touching (zero separation) counts as contact.
/// </summary>
internal static class ExactNarrowPhase
{
    private const double Eps = 1e-12;

    /// <summary>OBB–OBB overlap (Gottschalk / Ericson RTCD §4.4.1, 15 axes).</summary>
    public static bool ObbObb(
        double[] mA, double ax, double ay, double az,
        double[] mB, double bx, double by, double bz)
    {
        Span<double> a = stackalloc double[3] { ax, ay, az };
        Span<double> b = stackalloc double[3] { bx, by, bz };
        Span<double> r = stackalloc double[9];
        Span<double> absR = stackalloc double[9];
        Span<double> t = stackalloc double[3];

        for (var i = 0; i < 3; i++)
        for (var j = 0; j < 3; j++)
        {
            // dot(A axis i, B axis j); axis k of M = (M[k], M[4+k], M[8+k])
            var v = mA[i] * mB[j] + mA[4 + i] * mB[4 + j] + mA[8 + i] * mB[8 + j];
            r[i * 3 + j] = v;
            absR[i * 3 + j] = Math.Abs(v) + Eps;
        }

        var dx = mB[3] - mA[3];
        var dy = mB[7] - mA[7];
        var dz = mB[11] - mA[11];
        for (var i = 0; i < 3; i++)
            t[i] = dx * mA[i] + dy * mA[4 + i] + dz * mA[8 + i];

        double ra, rb;
        for (var i = 0; i < 3; i++)
        {
            ra = a[i];
            rb = b[0] * absR[i * 3] + b[1] * absR[i * 3 + 1] + b[2] * absR[i * 3 + 2];
            if (Math.Abs(t[i]) > ra + rb) return false;
        }
        for (var j = 0; j < 3; j++)
        {
            ra = a[0] * absR[j] + a[1] * absR[3 + j] + a[2] * absR[6 + j];
            rb = b[j];
            if (Math.Abs(t[0] * r[j] + t[1] * r[3 + j] + t[2] * r[6 + j]) > ra + rb) return false;
        }

        // A0 x Bj
        for (var j = 0; j < 3; j++)
        {
            var j1 = (j + 1) % 3; var j2 = (j + 2) % 3;
            ra = a[1] * absR[2 * 3 + j] + a[2] * absR[1 * 3 + j];
            rb = b[j1] * absR[0 * 3 + j2] + b[j2] * absR[0 * 3 + j1];
            if (Math.Abs(t[2] * r[1 * 3 + j] - t[1] * r[2 * 3 + j]) > ra + rb) return false;
        }
        // A1 x Bj
        for (var j = 0; j < 3; j++)
        {
            var j1 = (j + 1) % 3; var j2 = (j + 2) % 3;
            ra = a[0] * absR[2 * 3 + j] + a[2] * absR[0 * 3 + j];
            rb = b[j1] * absR[1 * 3 + j2] + b[j2] * absR[1 * 3 + j1];
            if (Math.Abs(t[0] * r[2 * 3 + j] - t[2] * r[0 * 3 + j]) > ra + rb) return false;
        }
        // A2 x Bj
        for (var j = 0; j < 3; j++)
        {
            var j1 = (j + 1) % 3; var j2 = (j + 2) % 3;
            ra = a[0] * absR[1 * 3 + j] + a[1] * absR[0 * 3 + j];
            rb = b[j1] * absR[2 * 3 + j2] + b[j2] * absR[2 * 3 + j1];
            if (Math.Abs(t[1] * r[0 * 3 + j] - t[0] * r[1 * 3 + j]) > ra + rb) return false;
        }
        return true;
    }

    /// <summary>
    /// Triangle (already in box-local coordinates) vs axis-aligned box centred at origin with half extents h
    /// (Akenine-Möller, 13 axes).
    /// </summary>
    public static bool TriangleAabbLocal(
        double v0x, double v0y, double v0z,
        double v1x, double v1y, double v1z,
        double v2x, double v2y, double v2z,
        double hx, double hy, double hz)
    {
        // Box face normals (AABB of triangle vs box)
        if (Math.Max(v0x, Math.Max(v1x, v2x)) < -hx || Math.Min(v0x, Math.Min(v1x, v2x)) > hx) return false;
        if (Math.Max(v0y, Math.Max(v1y, v2y)) < -hy || Math.Min(v0y, Math.Min(v1y, v2y)) > hy) return false;
        if (Math.Max(v0z, Math.Max(v1z, v2z)) < -hz || Math.Min(v0z, Math.Min(v1z, v2z)) > hz) return false;

        var e0x = v1x - v0x; var e0y = v1y - v0y; var e0z = v1z - v0z;
        var e1x = v2x - v1x; var e1y = v2y - v1y; var e1z = v2z - v1z;
        var e2x = v0x - v2x; var e2y = v0y - v2y; var e2z = v0z - v2z;

        // Triangle normal: plane vs box
        var nx = e0y * e1z - e0z * e1y;
        var ny = e0z * e1x - e0x * e1z;
        var nz = e0x * e1y - e0y * e1x;
        var d = nx * v0x + ny * v0y + nz * v0z;
        var rN = hx * Math.Abs(nx) + hy * Math.Abs(ny) + hz * Math.Abs(nz);
        if (Math.Abs(d) > rN) return false;

        // 9 edge cross axes: (unit box axis) x edge
        return EdgeAxes(e0x, e0y, e0z) && EdgeAxes(e1x, e1y, e1z) && EdgeAxes(e2x, e2y, e2z);

        bool EdgeAxes(double ex, double ey, double ez)
        {
            // X x e = (0, -ez, ey)
            if (!AxisTest(0, -ez, ey)) return false;
            // Y x e = (ez, 0, -ex)
            if (!AxisTest(ez, 0, -ex)) return false;
            // Z x e = (-ey, ex, 0)
            return AxisTest(-ey, ex, 0);
        }

        bool AxisTest(double axx, double axy, double axz)
        {
            if (Math.Abs(axx) + Math.Abs(axy) + Math.Abs(axz) < 1e-18) return true; // degenerate axis
            var p0 = axx * v0x + axy * v0y + axz * v0z;
            var p1 = axx * v1x + axy * v1y + axz * v1z;
            var p2 = axx * v2x + axy * v2y + axz * v2z;
            var mn = Math.Min(p0, Math.Min(p1, p2));
            var mx = Math.Max(p0, Math.Max(p1, p2));
            var rad = hx * Math.Abs(axx) + hy * Math.Abs(axy) + hz * Math.Abs(axz);
            return !(mn > rad || mx < -rad);
        }
    }

    /// <summary>Squared distance from point p to triangle (Ericson RTCD §5.1.5 closest point).</summary>
    public static double PointTriangleDistanceSq(
        double px, double py, double pz,
        double ax, double ay, double az,
        double bx, double by, double bz,
        double cx, double cy, double cz)
    {
        double abx = bx - ax, aby = by - ay, abz = bz - az;
        double acx = cx - ax, acy = cy - ay, acz = cz - az;
        var nx = aby * acz - abz * acy;
        var ny = abz * acx - abx * acz;
        var nz = abx * acy - aby * acx;
        if (nx * nx + ny * ny + nz * nz == 0)
            return Math.Min(PointSegmentDistanceSq(px, py, pz, ax, ay, az, bx, by, bz),
                Math.Min(PointSegmentDistanceSq(px, py, pz, bx, by, bz, cx, cy, cz),
                    PointSegmentDistanceSq(px, py, pz, cx, cy, cz, ax, ay, az)));
        double apx = px - ax, apy = py - ay, apz = pz - az;
        var d1 = abx * apx + aby * apy + abz * apz;
        var d2 = acx * apx + acy * apy + acz * apz;
        double qx, qy, qz;
        if (d1 <= 0 && d2 <= 0) { qx = ax; qy = ay; qz = az; return Sq(px - qx, py - qy, pz - qz); }

        double bpx = px - bx, bpy = py - by, bpz = pz - bz;
        var d3 = abx * bpx + aby * bpy + abz * bpz;
        var d4 = acx * bpx + acy * bpy + acz * bpz;
        if (d3 >= 0 && d4 <= d3) return Sq(px - bx, py - by, pz - bz);

        var vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0)
        {
            var v = d1 / (d1 - d3);
            return Sq(px - (ax + v * abx), py - (ay + v * aby), pz - (az + v * abz));
        }

        double cpx = px - cx, cpy = py - cy, cpz = pz - cz;
        var d5 = abx * cpx + aby * cpy + abz * cpz;
        var d6 = acx * cpx + acy * cpy + acz * cpz;
        if (d6 >= 0 && d5 <= d6) return Sq(px - cx, py - cy, pz - cz);

        var vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0)
        {
            var w = d2 / (d2 - d6);
            return Sq(px - (ax + w * acx), py - (ay + w * acy), pz - (az + w * acz));
        }

        var va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        {
            var w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return Sq(px - (bx + w * (cx - bx)), py - (by + w * (cy - by)), pz - (bz + w * (cz - bz)));
        }

        var denom = 1.0 / (va + vb + vc);
        var vv = vb * denom;
        var ww = vc * denom;
        qx = ax + abx * vv + acx * ww;
        qy = ay + aby * vv + acy * ww;
        qz = az + abz * vv + acz * ww;
        return Sq(px - qx, py - qy, pz - qz);

        static double Sq(double x, double y, double z) => x * x + y * y + z * z;
    }

    /// <summary>Squared distance from point p to segment [a,b].</summary>
    public static double PointSegmentDistanceSq(
        double px, double py, double pz, double ax, double ay, double az, double bx, double by, double bz)
    {
        double abx = bx - ax, aby = by - ay, abz = bz - az;
        double apx = px - ax, apy = py - ay, apz = pz - az;
        var ll = abx * abx + aby * aby + abz * abz;
        var t = ll > 0 ? Math.Clamp((apx * abx + apy * aby + apz * abz) / ll, 0, 1) : 0;
        double dx = apx - t * abx, dy = apy - t * aby, dz = apz - t * abz;
        return dx * dx + dy * dy + dz * dz;
    }

    /// <summary>Squared distance between segments [p1,q1] and [p2,q2] (Ericson RTCD §5.1.9).</summary>
    public static double SegmentSegmentDistanceSq(
        double p1x, double p1y, double p1z, double q1x, double q1y, double q1z,
        double p2x, double p2y, double p2z, double q2x, double q2y, double q2z)
    {
        double d1x = q1x - p1x, d1y = q1y - p1y, d1z = q1z - p1z;
        double d2x = q2x - p2x, d2y = q2y - p2y, d2z = q2z - p2z;
        double rx = p1x - p2x, ry = p1y - p2y, rz = p1z - p2z;
        var a = d1x * d1x + d1y * d1y + d1z * d1z;
        var e = d2x * d2x + d2y * d2y + d2z * d2z;
        var f = d2x * rx + d2y * ry + d2z * rz;
        double s, t;
        if (a <= Eps && e <= Eps) { s = t = 0; }
        else if (a <= Eps) { s = 0; t = Math.Clamp(f / e, 0, 1); }
        else
        {
            var c = d1x * rx + d1y * ry + d1z * rz;
            if (e <= Eps) { t = 0; s = Math.Clamp(-c / a, 0, 1); }
            else
            {
                var b = d1x * d2x + d1y * d2y + d1z * d2z;
                var denom = a * e - b * b;
                s = denom > 0 ? Math.Clamp((b * f - c * e) / denom, 0, 1) : 0;
                t = (b * s + f) / e;
                if (t < 0) { t = 0; s = Math.Clamp(-c / a, 0, 1); }
                else if (t > 1) { t = 1; s = Math.Clamp((b - c) / a, 0, 1); }
            }
        }
        double dx = p1x + d1x * s - (p2x + d2x * t), dy = p1y + d1y * s - (p2y + d2y * t), dz = p1z + d1z * s - (p2z + d2z * t);
        return dx * dx + dy * dy + dz * dz;
    }

    /// <summary>
    /// Exact squared distance from segment [p,q] to triangle (a,b,c): 0 when the segment pierces the triangle,
    /// otherwise the minimum of endpoint–triangle and segment–edge distances (closest features of a segment and a
    /// triangle are always among these).
    /// </summary>
    public static double SegmentTriangleDistanceSq(
        double px, double py, double pz, double qx, double qy, double qz,
        double ax, double ay, double az, double bx, double by, double bz, double cx, double cy, double cz)
    {
        if (SegmentPiercesTriangle(px, py, pz, qx, qy, qz, ax, ay, az, bx, by, bz, cx, cy, cz)) return 0;
        var d = PointTriangleDistanceSq(px, py, pz, ax, ay, az, bx, by, bz, cx, cy, cz);
        d = Math.Min(d, PointTriangleDistanceSq(qx, qy, qz, ax, ay, az, bx, by, bz, cx, cy, cz));
        d = Math.Min(d, SegmentSegmentDistanceSq(px, py, pz, qx, qy, qz, ax, ay, az, bx, by, bz));
        d = Math.Min(d, SegmentSegmentDistanceSq(px, py, pz, qx, qy, qz, bx, by, bz, cx, cy, cz));
        d = Math.Min(d, SegmentSegmentDistanceSq(px, py, pz, qx, qy, qz, cx, cy, cz, ax, ay, az));
        return d;
    }

    private static bool SegmentPiercesTriangle(
        double px, double py, double pz, double qx, double qy, double qz,
        double ax, double ay, double az, double bx, double by, double bz, double cx, double cy, double cz)
    {
        double abx = bx - ax, aby = by - ay, abz = bz - az, acx = cx - ax, acy = cy - ay, acz = cz - az;
        double nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
        var dp = (px - ax) * nx + (py - ay) * ny + (pz - az) * nz;
        var dq = (qx - ax) * nx + (qy - ay) * ny + (qz - az) * nz;
        if (dp * dq > 0 || dp == dq) return false; // same side or parallel (parallel case covered by edge/endpoint terms)
        var t = dp / (dp - dq);
        double x = px + t * (qx - px), y = py + t * (qy - py), z = pz + t * (qz - pz);
        // Point-in-triangle by same-sign cross products against the normal.
        double S(double ux, double uy, double uz, double vx, double vy, double vz) =>
            (uy * vz - uz * vy) * nx + (uz * vx - ux * vz) * ny + (ux * vy - uy * vx) * nz;
        var s0 = S(bx - ax, by - ay, bz - az, x - ax, y - ay, z - az);
        var s1 = S(cx - bx, cy - by, cz - bz, x - bx, y - by, z - bz);
        var s2 = S(ax - cx, ay - cy, az - cz, x - cx, y - cy, z - cz);
        return (s0 >= 0 && s1 >= 0 && s2 >= 0) || (s0 <= 0 && s1 <= 0 && s2 <= 0);
    }

    /// <summary>
    /// Exact squared distance from segment [p,q] (box-local coordinates) to the axis-aligned box with half extents h:
    /// 0 if the segment meets the box, else the minimum of endpoint–box and segment–edge distances.
    /// </summary>
    public static double SegmentAabbDistanceSqLocal(
        double px, double py, double pz, double qx, double qy, double qz, double hx, double hy, double hz)
    {
        // Slab test for intersection.
        double t0 = 0, t1 = 1;
        if (Slab(px, qx - px, hx) && Slab(py, qy - py, hy) && Slab(pz, qz - pz, hz)) return 0;
        var d = Math.Min(PointBox(px, py, pz), PointBox(qx, qy, qz));
        for (var i = 0; i < 4; i++)
        {
            double sa = (i & 1) == 0 ? -1 : 1, sb = (i & 2) == 0 ? -1 : 1;
            d = Math.Min(d, SegmentSegmentDistanceSq(px, py, pz, qx, qy, qz, -hx, sa * hy, sb * hz, hx, sa * hy, sb * hz));
            d = Math.Min(d, SegmentSegmentDistanceSq(px, py, pz, qx, qy, qz, sa * hx, -hy, sb * hz, sa * hx, hy, sb * hz));
            d = Math.Min(d, SegmentSegmentDistanceSq(px, py, pz, qx, qy, qz, sa * hx, sb * hy, -hz, sa * hx, sb * hy, hz));
        }
        return d;

        bool Slab(double o, double dir, double h)
        {
            if (Math.Abs(dir) < 1e-300) return o >= -h && o <= h;
            var ta = (-h - o) / dir; var tb = (h - o) / dir;
            if (ta > tb) (ta, tb) = (tb, ta);
            t0 = Math.Max(t0, ta); t1 = Math.Min(t1, tb);
            return t0 <= t1;
        }
        double PointBox(double x, double y, double z)
        {
            var dx = Math.Max(Math.Abs(x) - hx, 0); var dy = Math.Max(Math.Abs(y) - hy, 0); var dz = Math.Max(Math.Abs(z) - hz, 0);
            return dx * dx + dy * dy + dz * dz;
        }
    }

    /// <summary>Frame overload of <see cref="SegmentSegmentDistanceSq(double,double,double,double,double,double,double,double,double,double,double,double)"/>.</summary>
    public static double SegmentSegmentDistanceSq(Frame p1, Frame q1, Frame p2, Frame q2) =>
        SegmentSegmentDistanceSq(p1.X, p1.Y, p1.Z, q1.X, q1.Y, q1.Z, p2.X, p2.Y, p2.Z, q2.X, q2.Y, q2.Z);

    /// <summary>Frame overload of the exact segment–triangle distance.</summary>
    public static double SegmentTriangleDistanceSq(Frame p, Frame q, Frame a, Frame b, Frame c) =>
        SegmentTriangleDistanceSq(p.X, p.Y, p.Z, q.X, q.Y, q.Z, a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z);

    /// <summary>Exact squared distance from segment [p,q] (box-local) to the AABB with half extents (hx,hy,hz).</summary>
    public static double SegmentAabbDistanceSq(Frame p, Frame q, double hx, double hy, double hz) =>
        SegmentAabbDistanceSqLocal(p.X, p.Y, p.Z, q.X, q.Y, q.Z, hx, hy, hz);

    /// <summary>Sphere centres along a capsule axis spaced at most a quarter radius apart (legacy inner approximation; kept only OR-ed with the exact capsule tests so verdicts are never less conservative).</summary>
    public static int DenseCapsuleSamples(double radius, double halfLength)
    {
        if (radius <= 0) return 8;
        var n = (int)Math.Ceiling(2 * halfLength / (0.25 * radius));
        return Math.Clamp(n, 5, 400);
    }
}
