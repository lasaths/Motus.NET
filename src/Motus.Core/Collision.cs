namespace Motus.Core;

public enum CollisionShape
{
    Sphere,
    Box,
    Capsule,
    Mesh,
    /// <summary>Infinite half-space. Free side is Motus local +X (Rhino plane +Z via FrameConversion).</summary>
    Plane
}

public sealed class CollisionObject
{
    public string Name { get; }
    public Frame Pose { get; }
    public CollisionShape Shape { get; }
    /// <summary>Sphere radius or box half-extent X (meters). Unused for Plane.</summary>
    public double ExtentX { get; }
    public double ExtentY { get; }
    public double ExtentZ { get; }

    // PONYTAIL: Mesh data (vertices [x,y,z], triangle indices)
    public List<double[]>? MeshVertices { get; }
    public List<int>? MeshIndices { get; }

    // PONYTAIL: AABB cache (min/max per axis in object space)
    public double[]? MeshAabbMin { get; }  // [minX, minY, minZ]
    public double[]? MeshAabbMax { get; }  // [maxX, maxY, maxZ]

    /// <summary>Stable content key computed once at construction (verts/indices or primitive extents).</summary>
    public int ContentHash { get; }

    // PONYTAIL: Sphere/Box/Capsule/Plane constructor
    private CollisionObject(string name, Frame pose, CollisionShape shape, double extentX, double extentY, double extentZ)
    {
        Name = name;
        Pose = pose;
        Shape = shape;
        ExtentX = extentX;
        ExtentY = shape == CollisionShape.Box ? extentY : shape == CollisionShape.Capsule ? extentY : extentX;
        ExtentZ = shape == CollisionShape.Box ? extentZ : extentX;
        ContentHash = ComputePrimitiveContentHash(name, shape, ExtentX, ExtentY, ExtentZ);
    }

    // PONYTAIL: Mesh constructor
    private CollisionObject(string name, Frame pose, List<double[]> vertices, List<int> indices)
    {
        Name = name;
        Pose = pose;
        Shape = CollisionShape.Mesh;
        MeshVertices = vertices;
        MeshIndices = indices;
        ExtentX = 0; // PONYTAIL: Not used for mesh
        ExtentY = 0;
        ExtentZ = 0;

        // PONYTAIL: Compute AABB once at construction
        (MeshAabbMin, MeshAabbMax) = ComputeAabb(vertices);
        ContentHash = ComputeMeshContentHash(name, vertices, indices);
    }

    public static CollisionObject Sphere(string name, Frame pose, double radiusMeters) =>
        new(name, pose, CollisionShape.Sphere, radiusMeters, 0, 0);

    public static CollisionObject Box(string name, Frame pose, double halfX, double halfY, double halfZ) =>
        new(name, pose, CollisionShape.Box, halfX, halfY, halfZ);

    /// <summary>Capsule aligned with local +Z; ExtentX = radius, ExtentY = half-length along Z.</summary>
    public static CollisionObject Capsule(string name, Frame pose, double radiusMeters, double halfLengthMeters) =>
        new(name, pose, CollisionShape.Capsule, radiusMeters, halfLengthMeters, 0);

    /// <summary>
    /// Infinite half-space. Occupied side is Motus local −X; free side is +X.
    /// With Grasshopper <c>FrameConversion.FromPlane</c>, Rhino plane +Z is the free side (floor = WorldXY).
    /// </summary>
    public static CollisionObject Plane(string name, Frame pose) =>
        new(name, pose, CollisionShape.Plane, 0, 0, 0);

    // PONYTAIL: Mesh factory
    public static CollisionObject Mesh(string name, Frame pose, List<double[]> vertices, List<int> indices) =>
        new(name, pose, vertices, indices);

    private static (double[] min, double[] max) ComputeAabb(List<double[]> vertices)
    {
        if (vertices.Count == 0)
            return (new double[] { 0, 0, 0 }, new double[] { 0, 0, 0 });

        var min = new double[] { vertices[0][0], vertices[0][1], vertices[0][2] };
        var max = new double[] { vertices[0][0], vertices[0][1], vertices[0][2] };

        for (var i = 1; i < vertices.Count; i++)
        {
            for (var d = 0; d < 3; d++)
            {
                if (vertices[i][d] < min[d]) min[d] = vertices[i][d];
                if (vertices[i][d] > max[d]) max[d] = vertices[i][d];
            }
        }

        return (min, max);
    }

    private static int ComputePrimitiveContentHash(
        string name, CollisionShape shape, double extentX, double extentY, double extentZ)
    {
        var hash = new HashCode();
        hash.Add(name, StringComparer.Ordinal);
        hash.Add((int)shape);
        hash.Add(extentX);
        hash.Add(extentY);
        hash.Add(extentZ);
        return hash.ToHashCode();
    }

    private static int ComputeMeshContentHash(string name, List<double[]> vertices, List<int> indices)
    {
        var hash = new HashCode();
        hash.Add(name, StringComparer.Ordinal);
        hash.Add(0.0); // ExtentX/Y/Z for meshes (matches historical GeometryFingerprint)
        hash.Add(0.0);
        hash.Add(0.0);
        hash.Add(vertices.Count);
        foreach (var v in vertices)
        {
            hash.Add(v[0]);
            hash.Add(v[1]);
            hash.Add(v[2]);
        }
        hash.Add(indices.Count);
        foreach (var i in indices)
            hash.Add(i);
        return hash.ToHashCode();
    }
}
