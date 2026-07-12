using Motus.Core;

namespace Motus.Geometry;

internal static class CollisionGeometry
{
    public static CollisionObject Transform(CollisionObject local, double[] linkWorldMatrix)
    {
        var worldM = Transforms.Multiply(linkWorldMatrix, Transforms.FromFrame(local.Pose));
        var worldFrame = Transforms.ToFrame(worldM);
        return local.Shape switch
        {
            CollisionShape.Sphere => CollisionObject.Sphere(local.Name, worldFrame, local.ExtentX),
            CollisionShape.Box => CollisionObject.Box(local.Name, worldFrame, local.ExtentX, local.ExtentY, local.ExtentZ),
            CollisionShape.Capsule => CollisionObject.Capsule(local.Name, worldFrame, local.ExtentX, local.ExtentY),
            CollisionShape.Mesh when local.MeshVertices is not null && local.MeshIndices is not null =>
                CollisionObject.Mesh(local.Name, Frame.Identity, TransformVertices(local.MeshVertices, worldM), local.MeshIndices),
            _ => local
        };
    }

    private static List<double[]> TransformVertices(List<double[]> vertices, double[] worldM)
    {
        var result = new List<double[]>(vertices.Count);
        foreach (var v in vertices)
        {
            var p = Transforms.TransformPoint(worldM, v[0], v[1], v[2]);
            result.Add(new[] { p[0], p[1], p[2] });
        }
        return result;
    }

    public static bool Intersects(CollisionObject robot, CollisionObject obstacle, Dictionary<string, BvhNode> bvhCache)
    {
        return robot.Shape switch
        {
            CollisionShape.Sphere => SphereIntersectsObject(robot.Pose, robot.ExtentX, obstacle, bvhCache),
            CollisionShape.Capsule => CapsuleIntersectsObject(robot, obstacle, bvhCache),
            CollisionShape.Box => BoxIntersectsObject(robot, obstacle, bvhCache),
            CollisionShape.Mesh => MeshIntersectsObject(robot, obstacle, bvhCache),
            _ => false
        };
    }

    public static IEnumerable<(Frame center, double radius)> SampleCapsule(CollisionObject capsule, int samples = 5)
    {
        var r = capsule.ExtentX;
        var halfLen = capsule.ExtentY;
        var m = Transforms.FromFrame(capsule.Pose);
        for (var s = 0; s <= samples; s++)
        {
            var t = (double)s / samples;
            var z = -halfLen + t * (2 * halfLen);
            var p = Transforms.TransformPoint(m, 0.0, 0.0, z);
            yield return (new Frame(p[0], p[1], p[2]), r);
        }
    }

    private static bool CapsuleIntersectsObject(CollisionObject capsule, CollisionObject obstacle, Dictionary<string, BvhNode> bvhCache)
    {
        foreach (var (center, radius) in SampleCapsule(capsule))
            if (SphereIntersectsObject(center, radius, obstacle, bvhCache))
                return true;
        return false;
    }

    private static bool SphereIntersectsObject(Frame center, double radius, CollisionObject obj, Dictionary<string, BvhNode> bvhCache) =>
        obj.Shape switch
        {
            CollisionShape.Sphere => SphereSphereOverlap(center, radius, obj.Pose, obj.ExtentX),
            CollisionShape.Box => SphereBoxOverlap(center, radius, obj),
            CollisionShape.Capsule => CapsuleIntersectsObject(obj, CollisionObject.Sphere("_", center, radius), bvhCache),
            CollisionShape.Mesh => SphereMeshOverlap(center, radius, obj, bvhCache),
            _ => false
        };

    private static bool BoxIntersectsObject(CollisionObject box, CollisionObject obstacle, Dictionary<string, BvhNode> bvhCache)
    {
        var hx = box.ExtentX; var hy = box.ExtentY; var hz = box.ExtentZ;
        var offsets = new[]
        {
            (-hx, -hy, -hz), (hx, -hy, -hz), (-hx, hy, -hz), (hx, hy, -hz),
            (-hx, -hy, hz), (hx, -hy, hz), (-hx, hy, hz), (hx, hy, hz)
        };
        foreach (var (ox, oy, oz) in offsets)
        {
            var local = Transforms.TransformPoint(Transforms.FromFrame(box.Pose), ox, oy, oz);
            var pt = new Frame(local[0], local[1], local[2]);
            if (SphereIntersectsObject(pt, 1e-3, obstacle, bvhCache)) return true;
            if (SphereIntersectsObject(pt, Math.Max(hx, Math.Max(hy, hz)) * 0.5, obstacle, bvhCache)) return true;
        }
        return SphereIntersectsObject(box.Pose, Math.Max(hx, Math.Max(hy, hz)), obstacle, bvhCache);
    }

    private static bool MeshIntersectsObject(CollisionObject mesh, CollisionObject obstacle, Dictionary<string, BvhNode> bvhCache)
    {
        if (mesh.MeshVertices is null || mesh.MeshIndices is null) return false;

        if (obstacle.Shape == CollisionShape.Mesh &&
            obstacle.MeshVertices is not null &&
            obstacle.MeshIndices is not null &&
            bvhCache.TryGetValue(obstacle.Name, out var obstacleBvh))
        {
            return RobotMeshIntersectsMesh(mesh, obstacle, obstacleBvh);
        }

        // ponytail: vertex+10mm fallback for sphere/box/capsule obstacles only
        foreach (var v in mesh.MeshVertices)
        {
            var world = Transforms.TransformPoint(Transforms.FromFrame(mesh.Pose), v[0], v[1], v[2]);
            if (SphereIntersectsObject(new Frame(world[0], world[1], world[2]), 0.01, obstacle, bvhCache))
                return true;
        }
        return false;
    }

    private static bool RobotMeshIntersectsMesh(CollisionObject robot, CollisionObject obstacle, BvhNode obstacleBvh)
    {
        var robotIndices = robot.MeshIndices!;
        var robotVerts = robot.MeshVertices!;
        var obsVerts = obstacle.MeshVertices!;
        var obsIndices = obstacle.MeshIndices!;

        for (var tri = 0; tri < robotIndices.Count / 3; tri++)
        {
            var bi = tri * 3;
            var a0 = WorldVertex(robotVerts, robotIndices[bi], robot.Pose);
            var a1 = WorldVertex(robotVerts, robotIndices[bi + 1], robot.Pose);
            var a2 = WorldVertex(robotVerts, robotIndices[bi + 2], robot.Pose);
            var cx = (a0.X + a1.X + a2.X) / 3;
            var cy = (a0.Y + a1.Y + a2.Y) / 3;
            var cz = (a0.Z + a1.Z + a2.Z) / 3;
            var center = new Frame(cx, cy, cz);
            var radius = Math.Max(Distance(center, a0), Math.Max(Distance(center, a1), Distance(center, a2)));

            foreach (var oTri in obstacleBvh.GetPotentialTriangles(center, radius))
            {
                var ob = oTri * 3;
                if (ob + 2 >= obsIndices.Count) continue;
                var b0 = Vertex(obsVerts, obsIndices[ob]);
                var b1 = Vertex(obsVerts, obsIndices[ob + 1]);
                var b2 = Vertex(obsVerts, obsIndices[ob + 2]);
                if (TriangleCollision.TriangleTriangleOverlap(a0, a1, a2, Frame.Identity, b0, b1, b2, obstacle.Pose))
                    return true;
            }
        }
        return false;
    }

    private static Frame WorldVertex(List<double[]> verts, int idx, Frame pose)
    {
        var local = Vertex(verts, idx);
        return Transforms.ToFrame(Transforms.Multiply(Transforms.FromFrame(pose), Transforms.FromFrame(local)));
    }

    private static double Distance(Frame a, Frame b)
    {
        var dx = a.X - b.X;
        var dy = a.Y - b.Y;
        var dz = a.Z - b.Z;
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    private static bool SphereMeshOverlap(Frame linkCenter, double linkRadius, CollisionObject mesh, Dictionary<string, BvhNode> bvhCache)
    {
        if (!bvhCache.TryGetValue(mesh.Name, out var bvh))
            return SphereAabbOverlap(linkCenter, linkRadius, mesh);

        var localSphere = Transforms.ToFrame(
            Transforms.Multiply(Transforms.Inverse(Transforms.FromFrame(mesh.Pose)), Transforms.FromFrame(linkCenter)));
        if (!bvh.OverlapsSphere(localSphere, linkRadius)) return false;

        if (mesh.MeshIndices is null || mesh.MeshVertices is null) return false;
        foreach (var triIdx in bvh.GetPotentialTriangles(localSphere, linkRadius))
        {
            var baseIdx = triIdx * 3;
            if (baseIdx + 2 >= mesh.MeshIndices.Count) continue;
            var v0 = Vertex(mesh.MeshVertices, mesh.MeshIndices[baseIdx]);
            var v1 = Vertex(mesh.MeshVertices, mesh.MeshIndices[baseIdx + 1]);
            var v2 = Vertex(mesh.MeshVertices, mesh.MeshIndices[baseIdx + 2]);
            if (TriangleCollision.SphereTriangleOverlap(linkCenter, linkRadius, v0, v1, v2, mesh.Pose))
                return true;
        }
        return false;
    }

    private static Frame Vertex(List<double[]> vertices, int idx) =>
        new(vertices[idx][0], vertices[idx][1], vertices[idx][2]);

    private static bool SphereAabbOverlap(Frame sphereCenter, double radius, CollisionObject mesh)
    {
        if (mesh.MeshAabbMin is null || mesh.MeshAabbMax is null) return false;
        var local = WorldToLocal(sphereCenter, mesh.Pose);
        var min = mesh.MeshAabbMin; var max = mesh.MeshAabbMax;
        var cx = Math.Clamp(local.X, min[0], max[0]);
        var cy = Math.Clamp(local.Y, min[1], max[1]);
        var cz = Math.Clamp(local.Z, min[2], max[2]);
        var dx = local.X - cx; var dy = local.Y - cy; var dz = local.Z - cz;
        return dx * dx + dy * dy + dz * dz < radius * radius;
    }

    private static bool SphereSphereOverlap(Frame a, double ra, Frame b, double rb)
    {
        var dx = a.X - b.X; var dy = a.Y - b.Y; var dz = a.Z - b.Z;
        var dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        return dist < ra + rb;
    }

    private static bool SphereBoxOverlap(Frame center, double radius, CollisionObject box)
    {
        var local = WorldToLocal(center, box.Pose);
        var hx = box.ExtentX; var hy = box.ExtentY; var hz = box.ExtentZ;
        var cx = Math.Clamp(local.X, -hx, hx);
        var cy = Math.Clamp(local.Y, -hy, hy);
        var cz = Math.Clamp(local.Z, -hz, hz);
        var dx = local.X - cx; var dy = local.Y - cy; var dz = local.Z - cz;
        return dx * dx + dy * dy + dz * dz < radius * radius;
    }

    private static Frame WorldToLocal(Frame point, Frame objectPose)
    {
        var inv = Transforms.Inverse(Transforms.FromFrame(objectPose));
        return Transforms.ToFrame(Transforms.Multiply(inv, Transforms.FromFrame(point)));
    }
}
