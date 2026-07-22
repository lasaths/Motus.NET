using Motus.Core;

namespace Motus.Geometry;

public enum UrdfGeometryKind
{
    Box,
    Cylinder,
    Sphere,
    Mesh
}

/// <summary>
/// In-memory URDF visual/collision geometry primitive. Units are meters; <see cref="Origin"/> is
/// relative to the owning link's frame (no scale/skew — rotation + translation only).
/// </summary>
public sealed class UrdfGeometry
{
    public UrdfGeometryKind Kind { get; }

    /// <summary>Box full extents along X/Y/Z (meters). Zero for non-Box kinds.</summary>
    public double SizeX { get; }
    public double SizeY { get; }
    public double SizeZ { get; }

    /// <summary>Cylinder/Sphere radius (meters). Zero for Box/Mesh.</summary>
    public double Radius { get; }

    /// <summary>Cylinder length along local Z (meters). Zero for non-Cylinder kinds.</summary>
    public double Length { get; }

    /// <summary>Original mesh file reference as declared (e.g. a URDF <c>filename</c> attribute); optional.</summary>
    public string? FilePath { get; }

    /// <summary>Mesh vertices as [x,y,z] tuples in the geometry's local frame. Null for non-Mesh kinds.</summary>
    public IReadOnlyList<double[]>? Vertices { get; }

    /// <summary>Mesh triangle indices (flat, 3 per triangle). Null for non-Mesh kinds.</summary>
    public IReadOnlyList<int>? Indices { get; }

    /// <summary>Optional per-axis mesh scale [sx, sy, sz]; null means unscaled.</summary>
    public double[]? Scale { get; }

    /// <summary>Visual/collision origin relative to the owning link's frame.</summary>
    public Frame Origin { get; }

    private UrdfGeometry(
        UrdfGeometryKind kind,
        double sizeX, double sizeY, double sizeZ,
        double radius, double length,
        string? filePath,
        IReadOnlyList<double[]>? vertices,
        IReadOnlyList<int>? indices,
        double[]? scale,
        Frame origin)
    {
        Kind = kind;
        SizeX = sizeX; SizeY = sizeY; SizeZ = sizeZ;
        Radius = radius;
        Length = length;
        FilePath = filePath;
        Vertices = vertices;
        Indices = indices;
        Scale = scale;
        Origin = origin;
    }

    public static UrdfGeometry Box(double x, double y, double z, Frame? origin = null) =>
        new(UrdfGeometryKind.Box, x, y, z, 0, 0, null, null, null, null, origin ?? Frame.Identity);

    public static UrdfGeometry Cylinder(double radius, double length, Frame? origin = null) =>
        new(UrdfGeometryKind.Cylinder, 0, 0, 0, radius, length, null, null, null, null, origin ?? Frame.Identity);

    public static UrdfGeometry Sphere(double radius, Frame? origin = null) =>
        new(UrdfGeometryKind.Sphere, 0, 0, 0, radius, 0, null, null, null, null, origin ?? Frame.Identity);

    public static UrdfGeometry Mesh(
        IReadOnlyList<double[]> vertices,
        IReadOnlyList<int> indices,
        string? filePath = null,
        Frame? origin = null,
        double[]? scale = null)
    {
        if (vertices is null) throw new ArgumentNullException(nameof(vertices));
        if (indices is null) throw new ArgumentNullException(nameof(indices));
        return new(UrdfGeometryKind.Mesh, 0, 0, 0, 0, 0, filePath, vertices, indices, scale, origin ?? Frame.Identity);
    }
}
