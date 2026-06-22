using Motus.Core;

namespace Motus.Geometry;

internal static class Transforms
{
    public static double[] Identity() =>
        [1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1];

    public static double[] FromDh(double theta, double d, double a, double alpha)
    {
        var ct = Math.Cos(theta);
        var st = Math.Sin(theta);
        var ca = Math.Cos(alpha);
        var sa = Math.Sin(alpha);
        return
        [
            ct, -st * ca, st * sa, a * ct,
            st, ct * ca, -ct * sa, a * st,
            0, sa, ca, d,
            0, 0, 0, 1
        ];
    }

    public static double[] Multiply(double[] a, double[] b)
    {
        var r = new double[16];
        for (var col = 0; col < 4; col++)
        {
            for (var row = 0; row < 4; row++)
            {
                var sum = 0.0;
                for (var k = 0; k < 4; k++)
                    sum += a[row + k * 4] * b[k + col * 4];
                r[row + col * 4] = sum;
            }
        }
        return r;
    }

    public static double[] FromFrame(Frame frame)
    {
        var q = NormalizeQuat(frame.Qw, frame.Qx, frame.Qy, frame.Qz);
        var (w, x, y, z) = (q.w, q.x, q.y, q.z);
        var xx = x * x; var yy = y * y; var zz = z * z;
        var xy = x * y; var xz = x * z; var yz = y * z;
        var wx = w * x; var wy = w * y; var wz = w * z;
        return
        [
            1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), frame.X,
            2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), frame.Y,
            2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), frame.Z,
            0, 0, 0, 1
        ];
    }

    public static Frame ToFrame(double[] m)
    {
        var trace = m[0] + m[5] + m[10];
        double w, x, y, z;
        if (trace > 0)
        {
            var s = Math.Sqrt(trace + 1) * 2;
            w = 0.25 * s;
            x = (m[9] - m[6]) / s;
            y = (m[2] - m[8]) / s;
            z = (m[4] - m[1]) / s;
        }
        else if (m[0] > m[5] && m[0] > m[10])
        {
            var s = Math.Sqrt(1 + m[0] - m[5] - m[10]) * 2;
            w = (m[9] - m[6]) / s;
            x = 0.25 * s;
            y = (m[1] + m[4]) / s;
            z = (m[2] + m[8]) / s;
        }
        else if (m[5] > m[10])
        {
            var s = Math.Sqrt(1 + m[5] - m[0] - m[10]) * 2;
            w = (m[2] - m[8]) / s;
            x = (m[1] + m[4]) / s;
            y = 0.25 * s;
            z = (m[6] + m[9]) / s;
        }
        else
        {
            var s = Math.Sqrt(1 + m[10] - m[0] - m[5]) * 2;
            w = (m[4] - m[1]) / s;
            x = (m[2] + m[8]) / s;
            y = (m[6] + m[9]) / s;
            z = 0.25 * s;
        }
        var q = NormalizeQuat(w, x, y, z);
        return new Frame(m[3], m[7], m[11], q.w, q.x, q.y, q.z);
    }

    public static (double w, double x, double y, double z) NormalizeQuat(double w, double x, double y, double z)
    {
        var n = Math.Sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-12) return (1, 0, 0, 0);
        return (w / n, x / n, y / n, z / n);
    }

    public static double[] Inverse(double[] m)
    {
        var r = new double[9];
        r[0] = m[0]; r[1] = m[4]; r[2] = m[8];
        r[3] = m[1]; r[4] = m[5]; r[5] = m[9];
        r[6] = m[2]; r[7] = m[6]; r[8] = m[10];
        var tx = m[3]; var ty = m[7]; var tz = m[11];
        return
        [
            r[0], r[1], r[2], -(r[0] * tx + r[1] * ty + r[2] * tz),
            r[3], r[4], r[5], -(r[3] * tx + r[4] * ty + r[5] * tz),
            r[6], r[7], r[8], -(r[6] * tx + r[7] * ty + r[8] * tz),
            0, 0, 0, 1
        ];
    }

    public static double Distance(double[] a, double[] b)
    {
        var dx = a[3] - b[3];
        var dy = a[7] - b[7];
        var dz = a[11] - b[11];
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }
}
