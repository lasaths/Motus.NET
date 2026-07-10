using System.Globalization;

namespace Motus.Presets;

/// <summary>Read ASCII or binary STL into vertex/index lists for collision meshes.</summary>
public static class StlReader
{
    public static (List<double[]> vertices, List<int> indices) Read(string path, double uniformScale = 1.0) =>
        ReadBytes(File.ReadAllBytes(path), File.ReadAllLines(path), uniformScale);

    internal static (List<double[]> vertices, List<int> indices) ReadBytes(byte[] bytes, string[] asciiLines, double uniformScale = 1.0)
    {
        if (bytes.Length < 84) return (new List<double[]>(), new List<int>());
        var triCount = BitConverter.ToUInt32(bytes, 80);
        var expected = 84L + triCount * 50L;
        return expected == bytes.LongLength
            ? ReadBinary(bytes, triCount, uniformScale)
            : ReadAscii(asciiLines, uniformScale);
    }

    private static (List<double[]> vertices, List<int> indices) ReadBinary(byte[] data, uint triCount, double scale)
    {
        var vertices = new List<double[]>((int)triCount * 3);
        var indices = new List<int>((int)triCount * 3);
        var map = new Dictionary<string, int>();
        var offset = 84;
        for (var t = 0; t < triCount; t++)
        {
            offset += 12;
            for (var v = 0; v < 3; v++)
            {
                var x = BitConverter.ToSingle(data, offset) * scale; offset += 4;
                var y = BitConverter.ToSingle(data, offset) * scale; offset += 4;
                var z = BitConverter.ToSingle(data, offset) * scale; offset += 4;
                var key = $"{x:F6},{y:F6},{z:F6}";
                if (!map.TryGetValue(key, out var idx))
                {
                    idx = vertices.Count;
                    vertices.Add(new[] { (double)x, (double)y, (double)z });
                    map[key] = idx;
                }
                indices.Add(idx);
            }
            offset += 2;
        }
        return (vertices, indices);
    }

    private static (List<double[]> vertices, List<int> indices) ReadAscii(string[] lines, double scale)
    {
        var vertices = new List<double[]>();
        var indices = new List<int>();
        var map = new Dictionary<string, int>();
        double[]? v0 = null; double[]? v1 = null;

        foreach (var raw in lines)
        {
            var line = raw.Trim();
            if (line.StartsWith("vertex", StringComparison.OrdinalIgnoreCase))
            {
                var p = line.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
                if (p.Length < 4) continue;
                var vtx = new[] { Parse(p[1]) * scale, Parse(p[2]) * scale, Parse(p[3]) * scale };
                var key = $"{vtx[0]:F6},{vtx[1]:F6},{vtx[2]:F6}";
                if (!map.TryGetValue(key, out var idx))
                {
                    idx = vertices.Count;
                    vertices.Add(vtx);
                    map[key] = idx;
                }
                if (v0 is null) v0 = vtx;
                else if (v1 is null) v1 = vtx;
                else
                {
                    indices.Add(map[$"{v0[0]:F6},{v0[1]:F6},{v0[2]:F6}"]);
                    indices.Add(map[$"{v1[0]:F6},{v1[1]:F6},{v1[2]:F6}"]);
                    indices.Add(idx);
                    v0 = v1 = null;
                }
            }
            else if (line.StartsWith("endloop", StringComparison.OrdinalIgnoreCase))
                v0 = v1 = null;
        }
        return (vertices, indices);
    }

    private static double Parse(string s) =>
        double.Parse(s, CultureInfo.InvariantCulture);
}
