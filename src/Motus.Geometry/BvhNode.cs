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
        GetPotentialTrianglesRecursive(sphereCenter, sphereRadius, results);
        return results;
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
    // PONYTAIL: Build BVH from CollisionObject
    public static BvhNode Build(CollisionObject meshObj)
    {
        if (meshObj.MeshVertices == null || meshObj.MeshIndices == null)
            throw new ArgumentException($"Mesh {meshObj.Name} has no vertex/index data");
        
        // PONYTAIL: Convert to triangle index groups (3 indices per triangle)
        var triangleIndices = new List<int>();
        for (var i = 0; i < meshObj.MeshIndices.Count; i += 3)
        {
            if (i + 2 < meshObj.MeshIndices.Count)
                triangleIndices.Add(i / 3);
        }
        
        return Build(meshObj.MeshVertices, meshObj.MeshIndices, 10);
    }
    
    // PONYTAIL: Build BVH from triangle indices and vertices using SAH (Surface Area Heuristic)
    public static BvhNode Build(List<double[]> vertices, List<int> triangleIndices, int maxTrianglesPerNode = 10)
    {
        if (triangleIndices.Count == 0)
            throw new ArgumentException("No triangles provided");

        // PONYTAIL: Build tree recursively
        return BuildNode(vertices, triangleIndices, 0);
    }

    private static BvhNode BuildNode(List<double[]> vertices, List<int> triangleIndices, int depth)
    {
        // PONYTAIL: Compute AABB for this set of triangles
        var (min, max) = ComputeTriangleAabb(vertices, triangleIndices);

        // PONYTAIL: Base case - leaf node
        if (triangleIndices.Count <= 10 || depth >= 20)  // PONYTAIL: Max depth prevent stack overflow
        {
            return new BvhNode(min, max, new List<int>(triangleIndices));
        }

        // PONYTAIL: Split triangles - use SAH heuristic
        var (leftIndices, rightIndices) = SplitTriangles(vertices, triangleIndices, min, max);

        // PONYTAIL: Build subtrees
        var left = BuildNode(vertices, leftIndices, depth + 1);
        var right = BuildNode(vertices, rightIndices, depth + 1);

        // PONYTAIL: Combine AABBs
        var combinedMin = new[]
        {
            Math.Min(left.Min[0], right.Min[0]),
            Math.Min(left.Min[1], right.Min[1]),
            Math.Min(left.Min[2], right.Min[2])
        };
        var combinedMax = new[]
        {
            Math.Max(left.Max[0], right.Max[0]),
            Math.Max(left.Max[1], right.Max[1]),
            Math.Max(left.Max[2], right.Max[2])
        };

        return new BvhNode(combinedMin, combinedMax, left, right);
    }

    private static (List<int> left, List<int> right) SplitTriangles(
        List<double[]> vertices, List<int> triangleIndices, double[] min, double[] max)
    {
        // PONYTAIL: Simple median split on longest axis
        var extents = new[] { max[0] - min[0], max[1] - min[1], max[2] - min[2] };
        var splitAxis = extents[0] >= extents[1] && extents[0] >= extents[2] ? 0 :
                       extents[1] >= extents[2] ? 1 : 2;

        var splitPos = (min[splitAxis] + max[splitAxis]) / 2;

        var left = new List<int>();
        var right = new List<int>();

        foreach (var triangleIdx in triangleIndices)
        {
            var v0 = vertices[triangleIndices[triangleIdx * 3]];
            var centroid = (v0[splitAxis] + min[splitAxis]) / 2;  // PONYTAIL: Approximate centroid

            if (centroid < splitPos)
                left.Add(triangleIdx);
            else
                right.Add(triangleIdx);
        }

        // PONYTAIL: Ensure both sides have triangles
        if (left.Count == 0) left.Add(right[^1]); right.RemoveAt(right.Count - 1);
        if (right.Count == 0) right.Add(left[^1]); left.RemoveAt(left.Count - 1);

        return (left, right);
    }

    private static (double[] min, double[] max) ComputeTriangleAabb(List<double[]> vertices, List<int> triangleIndices)
    {
        if (triangleIndices.Count == 0)
            return (new[] { 0.0, 0.0, 0.0 }, new[] { 0.0, 0.0, 0.0 });

        var min = new[] { double.MaxValue, double.MaxValue, double.MaxValue };
        var max = new[] { double.MinValue, double.MinValue, double.MinValue };

        foreach (var triIdx in triangleIndices)
        {
            // PONYTAIL: triIdx points to a vertex index (triangleIndices is [v0, v1, v2] format)
            var vertex = vertices[triIdx];
            for (var d = 0; d < 3; d++)
            {
                if (vertex[d] < min[d]) min[d] = vertex[d];
                if (vertex[d] > max[d]) max[d] = vertex[d];
            }
        }

        return (min, max);
    }
}
