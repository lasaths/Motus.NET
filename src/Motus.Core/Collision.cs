namespace Motus.Core;

public enum CollisionShape
{
    Sphere,
    Box,
    Capsule,
    Mesh
}

public sealed class CollisionObject
{
    public string Name { get; }
    public Frame Pose { get; }
    public CollisionShape Shape { get; }
    /// <summary>Sphere radius or box half-extent X (meters).</summary>
    public double ExtentX { get; }
    public double ExtentY { get; }
    public double ExtentZ { get; }
    
    // PONYTAIL: Mesh data (vertices [x,y,z], triangle indices)
    public List<double[]>? MeshVertices { get; }
    public List<int>? MeshIndices { get; }
    
    // PONYTAIL: AABB cache (min/max per axis in object space)
    public double[]? MeshAabbMin { get; }  // [minX, minY, minZ]
    public double[]? MeshAabbMax { get; }  // [maxX, maxY, maxZ]

    // PONYTAIL: Sphere/Box constructor (existing)
    private CollisionObject(string name, Frame pose, CollisionShape shape, double extentX, double extentY, double extentZ)
    {
        Name = name;
        Pose = pose;
        Shape = shape;
        ExtentX = extentX;
        ExtentY = shape == CollisionShape.Box ? extentY : shape == CollisionShape.Capsule ? extentY : extentX;
        ExtentZ = shape == CollisionShape.Box ? extentZ : extentX;
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
    }

    public static CollisionObject Sphere(string name, Frame pose, double radiusMeters) =>
        new(name, pose, CollisionShape.Sphere, radiusMeters, 0, 0);

    public static CollisionObject Box(string name, Frame pose, double halfX, double halfY, double halfZ) =>
        new(name, pose, CollisionShape.Box, halfX, halfY, halfZ);

    /// <summary>Capsule aligned with local +Z; ExtentX = radius, ExtentY = half-length along Z.</summary>
    public static CollisionObject Capsule(string name, Frame pose, double radiusMeters, double halfLengthMeters) =>
        new(name, pose, CollisionShape.Capsule, radiusMeters, halfLengthMeters, 0);
    
    // PONYTAIL: Mesh factory
    public static CollisionObject Mesh(string name, Frame pose, List<double[]> vertices, List<int> indices) =>
        new(name, pose, vertices, indices);
    
    private static (double[] min, double[] max) ComputeAabb(List<double[]> vertices)
    {
        if (vertices.Count == 0) 
            return (new double[] {0,0,0}, new double[] {0,0,0});
            
        var min = new double[] {vertices[0][0], vertices[0][1], vertices[0][2]};
        var max = new double[] {vertices[0][0], vertices[0][1], vertices[0][2]};
        
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
}
