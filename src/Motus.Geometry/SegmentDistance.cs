using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Exact squared distances between points and line segments (Ericson RTCD §5.1.2, §5.1.9).
/// The exact narrow phase uses them for capsules, triangles and boxes.
/// </summary>
public static class SegmentDistance
{
    private const double Eps = 1e-12;

    /// <summary>Squared distance from point p to segment [a,b].</summary>
    public static double PointSegmentSq(
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
    public static double SegmentSegmentSq(
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

    /// <summary>Frame overload of <see cref="PointSegmentSq(double,double,double,double,double,double,double,double,double)"/>.</summary>
    public static double PointSegmentSq(Frame p, Frame a, Frame b) =>
        PointSegmentSq(p.X, p.Y, p.Z, a.X, a.Y, a.Z, b.X, b.Y, b.Z);

    /// <summary>Frame overload of <see cref="SegmentSegmentSq(double,double,double,double,double,double,double,double,double,double,double,double)"/>.</summary>
    public static double SegmentSegmentSq(Frame p1, Frame q1, Frame p2, Frame q2) =>
        SegmentSegmentSq(p1.X, p1.Y, p1.Z, q1.X, q1.Y, q1.Z, p2.X, p2.Y, p2.Z, q2.X, q2.Y, q2.Z);
}
