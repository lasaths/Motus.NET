using Motus.Core;

namespace Motus.Geometry;

/// <summary>Bounding Volume Hierarchy node for mesh collision detection.</summary>
public class BvhNode
{
    public double[] Min { get; private set; }  // AABB min [x,y,z]
    public double[] Max { get; private set; }  // AABB max [x,y,z]
    public BvhNode? Left { get; private set; }
    public BvhNode? Right { get; private set; }
    public List<int> TriangleIndices { get; private set; }  // Only for leaf nodes

    // PONYTAIL: Leaf constructor
    public BvhNode(double[] min, double[] max, List<int> triangleIndices)
    {
        Min = min;
        Max = max;
        TriangleIndices = triangleIndices;
        Left = Right = null;
    }

    // PONYTAIL: Internal node constructor
    public BvhNode(double[] min, double[] max, BvhNode left, BvhNode right)
    {
        Min = min;
        Max = max;
        Left = left;
        Right = right;
        TriangleIndices = new List<int>();
    }

    // PONYTAIL: AABB query with sphere
    public bool OverlapsSphere(Frame sphereCenter, double sphereRadius)
    {
        return SphereAabbOverlap(sphereCenter, sphereRadius, this);
    }

    // PONYTAIL: Collect all triangles that could overlap sphere
    public List<int> GetPotentialTriangles(Frame sphereCenter, double sphereRadius)
    {
        var results = new List<int>();
        GetPotentialTriangles(sphereCenter, sphereRadius, results);
        return results;
    }

    /// <summary>Append candidate triangle indices into a caller-owned list (cleared first).</summary>
    public void GetPotentialTriangles(Frame sphereCenter, double sphereRadius, List<int> results)
    {
        results.Clear();
        GetPotentialTrianglesRecursive(sphereCenter, sphereRadius, results);
    }

    private void GetPotentialTrianglesRecursive(Frame sphereCenter, double sphereRadius, List<int> results)
    {
        if (!OverlapsSphere(sphereCenter, sphereRadius))
            return;  // Prune this subtree

        if (Left is null && Right is null)
        {
            // PONYTAIL: Leaf node - add all triangles
            results.AddRange(TriangleIndices);
            return;
        }

        // PONYTAIL: Internal node - recurse
        Left?.GetPotentialTrianglesRecursive(sphereCenter, sphereRadius, results);
        Right?.GetPotentialTrianglesRecursive(sphereCenter, sphereRadius, results);
    }

    private static bool SphereAabbOverlap(Frame sphereCenter, double sphereRadius, BvhNode node)
    {
        // PONYTAIL: Find closest point on AABB to sphere center
        var cx = Math.Clamp(sphereCenter.X, node.Min[0], node.Max[0]);
        var cy = Math.Clamp(sphereCenter.Y, node.Min[1], node.Max[1]);
        var cz = Math.Clamp(sphereCenter.Z, node.Min[2], node.Max[2]);

        var dx = sphereCenter.X - cx;
        var dy = sphereCenter.Y - cy;
        var dz = sphereCenter.Z - cz;
        var distSq = dx * dx + dy * dy + dz * dz;

        return distSq < sphereRadius * sphereRadius;
    }
}

/// <summary>Bounding Volume Hierarchy builder for mesh triangle sets.</summary>
public class BvhBuilder
{
    public static BvhNode Build(CollisionObject meshObj)
    {
        if (meshObj.MeshVertices is null || meshObj.MeshIndices is null)
            throw new ArgumentException($"Mesh {meshObj.Name} has no vertex/index data");

        var triCount = meshObj.MeshIndices.Count / 3;
        var triangleNumbers = Enumerable.Range(0, triCount).ToList();
        return Build(meshObj.MeshVertices, meshObj.MeshIndices, triangleNumbers);
    }

    public static BvhNode Build(List<double[]> vertices, List<int> meshIndices, List<int> triangleNumbers, int maxTrianglesPerNode = 10)
    {
        if (triangleNumbers.Count == 0)
            throw new ArgumentException("No triangles provided");

        return BuildNode(vertices, meshIndices, triangleNumbers, 0, maxTrianglesPerNode);
    }

    private static BvhNode BuildNode(
        List<double[]> vertices, List<int> meshIndices, List<int> triangleNumbers, int depth, int maxTrianglesPerNode)
    {
        var (min, max) = ComputeTriangleAabb(vertices, meshIndices, triangleNumbers);

        if (triangleNumbers.Count <= maxTrianglesPerNode || depth >= 20)
            return new BvhNode(min, max, new List<int>(triangleNumbers));

        var (left, right) = SplitTriangles(vertices, meshIndices, triangleNumbers, min, max);
        var leftNode = BuildNode(vertices, meshIndices, left, depth + 1, maxTrianglesPerNode);
        var rightNode = BuildNode(vertices, meshIndices, right, depth + 1, maxTrianglesPerNode);

        var combinedMin = new[]
        {
            Math.Min(leftNode.Min[0], rightNode.Min[0]),
            Math.Min(leftNode.Min[1], rightNode.Min[1]),
            Math.Min(leftNode.Min[2], rightNode.Min[2])
        };
        var combinedMax = new[]
        {
            Math.Max(leftNode.Max[0], rightNode.Max[0]),
            Math.Max(leftNode.Max[1], rightNode.Max[1]),
            Math.Max(leftNode.Max[2], rightNode.Max[2])
        };

        return new BvhNode(combinedMin, combinedMax, leftNode, rightNode);
    }

    private static (List<int> left, List<int> right) SplitTriangles(
        List<double[]> vertices, List<int> meshIndices, List<int> triangleNumbers, double[] min, double[] max)
    {
        var extents = new[] { max[0] - min[0], max[1] - min[1], max[2] - min[2] };
        var splitAxis = extents[0] >= extents[1] && extents[0] >= extents[2] ? 0 :
                        extents[1] >= extents[2] ? 1 : 2;
        var splitPos = (min[splitAxis] + max[splitAxis]) / 2;

        var left = new List<int>();
        var right = new List<int>();

        foreach (var tri in triangleNumbers)
        {
            var centroid = TriangleCentroid(vertices, meshIndices, tri)[splitAxis];
            if (centroid < splitPos)
                left.Add(tri);
            else
                right.Add(tri);
        }

        if (left.Count == 0) { left.Add(right[^1]); right.RemoveAt(right.Count - 1); }
        if (right.Count == 0) { right.Add(left[^1]); left.RemoveAt(left.Count - 1); }

        return (left, right);
    }

    private static double[] TriangleCentroid(List<double[]> vertices, List<int> meshIndices, int triangleNumber)
    {
        var i0 = meshIndices[triangleNumber * 3];
        var i1 = meshIndices[triangleNumber * 3 + 1];
        var i2 = meshIndices[triangleNumber * 3 + 2];
        var v0 = vertices[i0];
        var v1 = vertices[i1];
        var v2 = vertices[i2];
        return new[]
        {
            (v0[0] + v1[0] + v2[0]) / 3.0,
            (v0[1] + v1[1] + v2[1]) / 3.0,
            (v0[2] + v1[2] + v2[2]) / 3.0
        };
    }

    private static (double[] min, double[] max) ComputeTriangleAabb(
        List<double[]> vertices, List<int> meshIndices, List<int> triangleNumbers)
    {
        if (triangleNumbers.Count == 0)
            return (new[] { 0.0, 0.0, 0.0 }, new[] { 0.0, 0.0, 0.0 });

        var min = new[] { double.MaxValue, double.MaxValue, double.MaxValue };
        var max = new[] { double.MinValue, double.MinValue, double.MinValue };

        foreach (var tri in triangleNumbers)
        {
            var i0 = meshIndices[tri * 3];
            var i1 = meshIndices[tri * 3 + 1];
            var i2 = meshIndices[tri * 3 + 2];
            foreach (var idx in new[] { i0, i1, i2 })
            {
                var vertex = vertices[idx];
                for (var d = 0; d < 3; d++)
                {
                    if (vertex[d] < min[d]) min[d] = vertex[d];
                    if (vertex[d] > max[d]) max[d] = vertex[d];
                }
            }
        }

        return (min, max);
    }
}
