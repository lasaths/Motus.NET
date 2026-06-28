using Motus.Core;

namespace Motus.Geometry;

/// <summary>Separating Axis Theorem (SAT) per-triangle collision detection.</summary>
public static class TriangleCollision
{
    /// <summary>Test sphere-triangle overlap using simplified SAT (face + vertex axes).</summary>
    public static bool SphereTriangleOverlap(
        Frame sphereCenter, double sphereRadius,
        Frame v0, Frame v1, Frame v2, Frame triFrame)
    {
        // PONYTAIL: Transform sphere center to triangle local space
        var localSphereCenter = Transforms.ToFrame(
            Transforms.Multiply(Transforms.Inverse(Transforms.FromFrame(triFrame)),
                              Transforms.FromFrame(sphereCenter)));

        // PONYTAIL: Test axis 1: Triangle face normal
        var normal = ComputeTriangleNormal(triFrame);
        if (!ProjectAndTestOverlap(localSphereCenter, sphereRadius, v0, v1, v2, normal))
            return false;

        // PONYTAIL: Test axes 2-4: Sphere center to triangle vertices
        var axis0 = new double[] { v0.X - localSphereCenter.X, v0.Y - localSphereCenter.Y, v0.Z - localSphereCenter.Z };
        Normalize(axis0);
        if (!ProjectAndTestOverlap(localSphereCenter, sphereRadius, v0, v1, v2, axis0))
            return false;

        var axis1 = new double[] { v1.X - localSphereCenter.X, v1.Y - localSphereCenter.Y, v1.Z - localSphereCenter.Z };
        Normalize(axis1);
        if (!ProjectAndTestOverlap(localSphereCenter, sphereRadius, v0, v1, v2, axis1))
            return false;

        var axis2 = new double[] { v2.X - localSphereCenter.X, v2.Y - localSphereCenter.Y, v2.Z - localSphereCenter.Z };
        Normalize(axis2);
        if (!ProjectAndTestOverlap(localSphereCenter, sphereRadius, v0, v1, v2, axis2))
            return false;

        return true;  // All axes overlap - collision
    }

    /// <summary>Test triangle-triangle overlap using full SAT.</summary>
    public static bool TriangleTriangleOverlap(
        Frame a0, Frame a1, Frame a2, Frame frameA,
        Frame b0, Frame b1, Frame b2, Frame frameB)
    {
        // PONYTAIL: Transform both triangles to world space for simplicity
        var worldA0 = TransformToWorld(a0, frameA);
        var worldA1 = TransformToWorld(a1, frameA);
        var worldA2 = TransformToWorld(a2, frameA);

        var worldB0 = TransformToWorld(b0, frameB);
        var worldB1 = TransformToWorld(b1, frameB);
        var worldB2 = TransformToWorld(b2, frameB);

        // PONYTAIL: Test 11 axes: 3 normals (A) + 3 normals (B) + 9 edge-edge cross products
        var axes = new List<double[]>();

        // PONYTAIL: Axes from triangle A
        var an0 = Cross(worldA1, worldA0, worldA2); Normalize(an0); axes.Add(an0);
        var an1 = Cross(worldA2, worldA1, worldA0); Normalize(an1); axes.Add(an1);
        var an2 = Cross(worldA0, worldA2, worldA1); Normalize(an2); axes.Add(an2);

        // PONYTAIL: Axes from triangle B
        var bn0 = Cross(worldB1, worldB0, worldB2); Normalize(bn0); axes.Add(bn0);
        var bn1 = Cross(worldB2, worldB1, worldB0); Normalize(bn1); axes.Add(bn1);
        var bn2 = Cross(worldB0, worldB2, worldB1); Normalize(bn2); axes.Add(bn2);

        // PONYTAIL: Edge-edge axes (9 cross products)
        var edgesA = new[] { Sub(worldA1, worldA0), Sub(worldA2, worldA1), Sub(worldA0, worldA2) };
        var edgesB = new[] { Sub(worldB1, worldB0), Sub(worldB2, worldB1), Sub(worldB0, worldB2) };

        foreach (var ea in edgesA)
        {
            foreach (var eb in edgesB)
            {
                var axis = Cross(ea, eb);
                var len = Math.Sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
                if (len > 1e-10)
                {
                    Normalize(axis);
                    axes.Add(axis);
                }
            }
        }

        // PONYTAIL: Test all axes - if any separates, no collision
        foreach (var axis in axes)
        {
            if (!ProjectAndTestTriangleTriangleOverlap(axis, worldA0, worldA1, worldA2, worldB0, worldB1, worldB2))
                return false;
        }

        return true;  // All axes overlap - collision
    }

    private static bool ProjectAndTestOverlap(
        Frame sphereCenter, double sphereRadius,
        Frame v0, Frame v1, Frame v2, double[] axis)
    {
        // PONYTAIL: Project triangle vertices to axis
        var projV0 = Dot(v0, axis);
        var projV1 = Dot(v1, axis);
        var projV2 = Dot(v2, axis);

        // PONYTAIL: Triangle min and max on axis
        var triMin = Math.Min(projV0, Math.Min(projV1, projV2));
        var triMax = Math.Max(projV0, Math.Max(projV1, projV2));

        // PONYTAIL: Project sphere to axis (point projection ± radius)
        var projSphere = Dot(sphereCenter, axis);
        var sphereMin = projSphere - sphereRadius;
        var sphereMax = projSphere + sphereRadius;

        // PONYTAIL: Check for interval overlap
        return !(sphereMax < triMin || triMax < sphereMin);
    }

    private static bool ProjectAndTestTriangleTriangleOverlap(
        double[] axis, Frame a0, Frame a1, Frame a2, Frame b0, Frame b1, Frame b2)
    {
        // PONYTAIL: Project triangle A
        var projA0 = Dot(a0, axis);
        var projA1 = Dot(a1, axis);
        var projA2 = Dot(a2, axis);
        var aMin = Math.Min(projA0, Math.Min(projA1, projA2));
        var aMax = Math.Max(projA0, Math.Max(projA1, projA2));

        // PONYTAIL: Project triangle B
        var projB0 = Dot(b0, axis);
        var projB1 = Dot(b1, axis);
        var projB2 = Dot(b2, axis);
        var bMin = Math.Min(projB0, Math.Min(projB1, projB2));
        var bMax = Math.Max(projB0, Math.Max(projB1, projB2));

        // PONYTAIL: Check for interval overlap
        return !(aMax < bMin || bMax < aMin);
    }

    private static double[] ComputeTriangleNormal(Frame triFrame)
    {
        // PONYTAIL: Normal from frame Z axis (simplified - assumes pose Z is normal)
        var m = Transforms.FromFrame(triFrame);
        return new[] { m[2], m[6], m[10] };  // Z column
    }

    private static Frame TransformToWorld(Frame local, Frame frame)
    {
        var worldM = Transforms.Multiply(Transforms.FromFrame(frame), Transforms.FromFrame(local));
        return Transforms.ToFrame(worldM);
    }

    private static double Dot(Frame f, double[] axis)
    {
        return f.X * axis[0] + f.Y * axis[1] + f.Z * axis[2];
    }

    private static void Normalize(double[] v)
    {
        var len = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (len > 1e-10)
        {
            v[0] /= len;
            v[1] /= len;
            v[2] /= len;
        }
    }

    private static double[] Cross(Frame a, Frame b, Frame c)
    {
        // PONYTAIL: Vector AB × AC = triangle normal
        var ab = Sub(b, a);
        var ac = Sub(c, a);
        return new[] { ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0] };
    }

    private static double[] Cross(double[] a, double[] b)
    {
        return new[] { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
    }

    private static double[] Sub(Frame a, Frame b)
    {
        return new[] { a.X - b.X, a.Y - b.Y, a.Z - b.Z };
    }
}
