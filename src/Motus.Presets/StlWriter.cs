using System.Globalization;
using System.Text;

namespace Motus.Presets;

/// <summary>Write triangle soups (vertices + triangle indices, as produced by <see cref="StlReader"/>) to STL.</summary>
public static class StlWriter
{
    /// <summary>Write a compact binary STL. <paramref name="indices"/> length must be a multiple of 3.</summary>
    public static void WriteBinary(string path, IReadOnlyList<double[]> vertices, IReadOnlyList<int> indices)
    {
        if (indices.Count % 3 != 0)
            throw new ArgumentException("indices.Count must be a multiple of 3.", nameof(indices));

        var triCount = (uint)(indices.Count / 3);
        using var stream = File.Create(path);
        using var writer = new BinaryWriter(stream);
        writer.Write(new byte[80]); // header, left blank
        writer.Write(triCount);

        for (var t = 0; t < triCount; t++)
        {
            var i0 = indices[t * 3];
            var i1 = indices[t * 3 + 1];
            var i2 = indices[t * 3 + 2];
            var v0 = vertices[i0];
            var v1 = vertices[i1];
            var v2 = vertices[i2];
            var (nx, ny, nz) = FaceNormal(v0, v1, v2);

            writer.Write((float)nx); writer.Write((float)ny); writer.Write((float)nz);
            WriteVertex(writer, v0);
            WriteVertex(writer, v1);
            WriteVertex(writer, v2);
            writer.Write((ushort)0); // attribute byte count
        }
    }

    /// <summary>Write a human-readable ASCII STL. <paramref name="indices"/> length must be a multiple of 3.</summary>
    public static void WriteAscii(string path, IReadOnlyList<double[]> vertices, IReadOnlyList<int> indices, string solidName = "solid")
    {
        if (indices.Count % 3 != 0)
            throw new ArgumentException("indices.Count must be a multiple of 3.", nameof(indices));

        var sb = new StringBuilder();
        sb.Append("solid ").Append(solidName).Append('\n');
        var triCount = indices.Count / 3;
        for (var t = 0; t < triCount; t++)
        {
            var v0 = vertices[indices[t * 3]];
            var v1 = vertices[indices[t * 3 + 1]];
            var v2 = vertices[indices[t * 3 + 2]];
            var (nx, ny, nz) = FaceNormal(v0, v1, v2);

            sb.Append("facet normal ").Append(Fmt(nx)).Append(' ').Append(Fmt(ny)).Append(' ').Append(Fmt(nz)).Append('\n');
            sb.Append("  outer loop\n");
            AppendVertex(sb, v0);
            AppendVertex(sb, v1);
            AppendVertex(sb, v2);
            sb.Append("  endloop\n");
            sb.Append("endfacet\n");
        }
        sb.Append("endsolid ").Append(solidName).Append('\n');
        File.WriteAllText(path, sb.ToString());
    }

    private static void WriteVertex(BinaryWriter writer, double[] v)
    {
        writer.Write((float)v[0]);
        writer.Write((float)v[1]);
        writer.Write((float)v[2]);
    }

    private static void AppendVertex(StringBuilder sb, double[] v) =>
        sb.Append("    vertex ").Append(Fmt(v[0])).Append(' ').Append(Fmt(v[1])).Append(' ').Append(Fmt(v[2])).Append('\n');

    private static (double x, double y, double z) FaceNormal(double[] v0, double[] v1, double[] v2)
    {
        var ax = v1[0] - v0[0]; var ay = v1[1] - v0[1]; var az = v1[2] - v0[2];
        var bx = v2[0] - v0[0]; var by = v2[1] - v0[1]; var bz = v2[2] - v0[2];
        var nx = ay * bz - az * by;
        var ny = az * bx - ax * bz;
        var nz = ax * by - ay * bx;
        var len = Math.Sqrt(nx * nx + ny * ny + nz * nz);
        return len > 1e-12 ? (nx / len, ny / len, nz / len) : (0, 0, 0);
    }

    private static string Fmt(double v) => v.ToString("G9", CultureInfo.InvariantCulture);
}
