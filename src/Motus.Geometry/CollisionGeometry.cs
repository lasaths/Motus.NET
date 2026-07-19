using Motus.Core;

namespace Motus.Geometry;

/// <summary>Reusable buffers for hot-path collision queries (one per checker).</summary>
internal sealed class CollisionQueryScratch
{
    public readonly List<int> TriangleHits = new(64);
    public readonly double[] WorldAabbMin = new double[3];
    public readonly double[] WorldAabbMax = new double[3];
    public readonly double[] WorldAabbMinB = new double[3];
    public readonly double[] WorldAabbMaxB = new double[3];
}

internal static class CollisionGeometry
{
    public static CollisionObject Transform(CollisionObject local, double[] linkWorldMatrix)
    {
        var worldM = ComposeWorldMatrix(linkWorldMatrix, local.Pose);
        var worldFrame = Transforms.ToFrame(worldM);
        return local.Shape switch
        {
            CollisionShape.Sphere => CollisionObject.Sphere(local.Name, worldFrame, local.ExtentX),
            CollisionShape.Box => CollisionObject.Box(local.Name, worldFrame, local.ExtentX, local.ExtentY, local.ExtentZ),
            CollisionShape.Capsule => CollisionObject.Capsule(local.Name, worldFrame, local.ExtentX, local.ExtentY),
            // Mesh: pose only — verts stay local. Prefer IntersectsAtPose on the hot path.
            CollisionShape.Mesh when local.MeshVertices is not null && local.MeshIndices is not null =>
                CollisionObject.Mesh(local.Name, worldFrame, local.MeshVertices, local.MeshIndices),
            _ => local
        };
    }

    public static double[] ComposeWorldMatrix(double[] linkWorldMatrix, Frame localPose) =>
        Transforms.Multiply(linkWorldMatrix, Transforms.FromFrame(localPose));

    public static bool Intersects(CollisionObject robot, CollisionObject obstacle, Dictionary<int, BvhNode> bvhCache) =>
        Intersects(robot, obstacle, bvhCache, null);

    public static bool Intersects(
        CollisionObject robot,
        CollisionObject obstacle,
        Dictionary<int, BvhNode> bvhCache,
        CollisionQueryScratch? scratch)
    {
        return robot.Shape switch
        {
            CollisionShape.Sphere => SphereIntersectsObject(robot.Pose, robot.ExtentX, obstacle, bvhCache, scratch),
            CollisionShape.Capsule => CapsuleIntersectsObject(robot, obstacle, bvhCache, scratch),
            CollisionShape.Box => BoxIntersectsObject(robot, obstacle, bvhCache, scratch),
            CollisionShape.Mesh => MeshIntersectsObject(robot, obstacle, bvhCache, scratch),
            CollisionShape.Plane => false, // planes are scene obstacles only
            _ => false
        };
    }

    /// <summary>Test local geometry posed by <paramref name="worldM"/> against an obstacle without copying mesh verts.</summary>
    public static bool IntersectsAtPose(
        CollisionObject local,
        double[] worldM,
        CollisionObject obstacle,
        Dictionary<int, BvhNode> bvhCache,
        CollisionQueryScratch scratch)
    {
        return local.Shape switch
        {
            CollisionShape.Sphere => SphereIntersectsObject(Transforms.ToFrame(worldM), local.ExtentX, obstacle, bvhCache, scratch),
            CollisionShape.Capsule => CapsuleIntersectsObject(
                CollisionObject.Capsule(local.Name, Transforms.ToFrame(worldM), local.ExtentX, local.ExtentY),
                obstacle, bvhCache, scratch),
            CollisionShape.Box => BoxIntersectsObject(
                CollisionObject.Box(local.Name, Transforms.ToFrame(worldM), local.ExtentX, local.ExtentY, local.ExtentZ),
                obstacle, bvhCache, scratch),
            CollisionShape.Mesh => LocalMeshIntersectsObject(local, worldM, obstacle, bvhCache, scratch),
            _ => false
        };
    }

    /// <summary>Mesh–mesh with both geometries in local space + world matrices.</summary>
    public static bool IntersectsMeshesAtPoses(
        CollisionObject meshA,
        double[] worldMA,
        CollisionObject meshB,
        double[] worldMB,
        BvhNode? localBvhB,
        CollisionQueryScratch scratch)
    {
        if (meshA.MeshVertices is null || meshA.MeshIndices is null ||
            meshB.MeshVertices is null || meshB.MeshIndices is null)
            return false;

        TransformLocalAabbToWorld(meshA, worldMA, scratch.WorldAabbMin, scratch.WorldAabbMax);
        TransformLocalAabbToWorld(meshB, worldMB, scratch.WorldAabbMinB, scratch.WorldAabbMaxB);
        if (!AabbAabbOverlap(scratch.WorldAabbMin, scratch.WorldAabbMax, scratch.WorldAabbMinB, scratch.WorldAabbMaxB))
            return false;

        var bvhB = localBvhB ?? CollisionMeshCache.GetOrBuild(meshB);
        var invB = Transforms.Inverse(worldMB);
        var indicesA = meshA.MeshIndices;
        var vertsA = meshA.MeshVertices;
        var indicesB = meshB.MeshIndices;
        var vertsB = meshB.MeshVertices;

        for (var tri = 0; tri < indicesA.Count / 3; tri++)
        {
            var bi = tri * 3;
            TransformVertex(worldMA, vertsA, indicesA[bi], out var a0x, out var a0y, out var a0z);
            TransformVertex(worldMA, vertsA, indicesA[bi + 1], out var a1x, out var a1y, out var a1z);
            TransformVertex(worldMA, vertsA, indicesA[bi + 2], out var a2x, out var a2y, out var a2z);

            var cx = (a0x + a1x + a2x) / 3;
            var cy = (a0y + a1y + a2y) / 3;
            var cz = (a0z + a1z + a2z) / 3;
            var radius = Math.Max(
                Distance(cx, cy, cz, a0x, a0y, a0z),
                Math.Max(Distance(cx, cy, cz, a1x, a1y, a1z), Distance(cx, cy, cz, a2x, a2y, a2z)));

            Transforms.TransformPointInto(invB, cx, cy, cz, out var lx, out var ly, out var lz);
            var localSphere = new Frame(lx, ly, lz);
            bvhB.GetPotentialTriangles(localSphere, radius, scratch.TriangleHits);

            var a0 = new Frame(a0x, a0y, a0z);
            var a1 = new Frame(a1x, a1y, a1z);
            var a2 = new Frame(a2x, a2y, a2z);

            foreach (var oTri in scratch.TriangleHits)
            {
                var ob = oTri * 3;
                if (ob + 2 >= indicesB.Count) continue;
                TransformVertex(worldMB, vertsB, indicesB[ob], out var b0x, out var b0y, out var b0z);
                TransformVertex(worldMB, vertsB, indicesB[ob + 1], out var b1x, out var b1y, out var b1z);
                TransformVertex(worldMB, vertsB, indicesB[ob + 2], out var b2x, out var b2y, out var b2z);
                if (TriangleCollision.TriangleTriangleOverlap(
                        a0, a1, a2, Frame.Identity,
                        new Frame(b0x, b0y, b0z), new Frame(b1x, b1y, b1z), new Frame(b2x, b2y, b2z),
                        Frame.Identity))
                    return true;
            }
        }

        return false;
    }

    public static double MeshEnvelopeRadius(CollisionObject mesh)
    {
        if (mesh.MeshAabbMin is null || mesh.MeshAabbMax is null) return 0.01;
        var dx = mesh.MeshAabbMax[0] - mesh.MeshAabbMin[0];
        var dy = mesh.MeshAabbMax[1] - mesh.MeshAabbMin[1];
        var dz = mesh.MeshAabbMax[2] - mesh.MeshAabbMin[2];
        return 0.5 * Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    public static void MeshEnvelopeCenterLocal(CollisionObject mesh, out double x, out double y, out double z)
    {
        if (mesh.MeshAabbMin is null || mesh.MeshAabbMax is null)
        {
            x = y = z = 0;
            return;
        }
        x = 0.5 * (mesh.MeshAabbMin[0] + mesh.MeshAabbMax[0]);
        y = 0.5 * (mesh.MeshAabbMin[1] + mesh.MeshAabbMax[1]);
        z = 0.5 * (mesh.MeshAabbMin[2] + mesh.MeshAabbMax[2]);
    }

    public static bool EnvelopeMayHit(
        CollisionObject local,
        double[] worldM,
        double envelopeRadius,
        CollisionObject obstacle,
        Dictionary<int, BvhNode> bvhCache,
        CollisionQueryScratch scratch)
    {
        if (local.Shape == CollisionShape.Mesh)
        {
            MeshEnvelopeCenterLocal(local, out var lx, out var ly, out var lz);
            Transforms.TransformPointInto(worldM, lx, ly, lz, out var wx, out var wy, out var wz);
            return SphereIntersectsObject(new Frame(wx, wy, wz), envelopeRadius, obstacle, bvhCache, scratch);
        }

        // worldM already includes local pose — sphere at frame origin with conservative radius
        var frame = Transforms.ToFrame(worldM);
        return SphereIntersectsObject(frame, envelopeRadius, obstacle, bvhCache, scratch);
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

    private static bool LocalMeshIntersectsObject(
        CollisionObject mesh,
        double[] worldM,
        CollisionObject obstacle,
        Dictionary<int, BvhNode> bvhCache,
        CollisionQueryScratch scratch)
    {
        if (mesh.MeshVertices is null || mesh.MeshIndices is null) return false;

        TransformLocalAabbToWorld(mesh, worldM, scratch.WorldAabbMin, scratch.WorldAabbMax);
        if (!WorldAabbMayHitObstacle(scratch.WorldAabbMin, scratch.WorldAabbMax, obstacle))
            return false;

        if (obstacle.Shape == CollisionShape.Mesh &&
            obstacle.MeshVertices is not null &&
            obstacle.MeshIndices is not null &&
            TryGetMeshBvh(bvhCache, obstacle, out var obstacleBvh))
        {
            return LocalMeshIntersectsMesh(mesh, worldM, obstacle, obstacleBvh, scratch);
        }

        // Primitive obstacles: sample transformed verts with small spheres
        foreach (var v in mesh.MeshVertices)
        {
            Transforms.TransformPointInto(worldM, v[0], v[1], v[2], out var wx, out var wy, out var wz);
            if (SphereIntersectsObject(new Frame(wx, wy, wz), 0.01, obstacle, bvhCache, scratch))
                return true;
        }
        return false;
    }

    private static bool LocalMeshIntersectsMesh(
        CollisionObject robot,
        double[] robotWorldM,
        CollisionObject obstacle,
        BvhNode obstacleBvh,
        CollisionQueryScratch scratch)
    {
        var robotIndices = robot.MeshIndices!;
        var robotVerts = robot.MeshVertices!;
        var obsVerts = obstacle.MeshVertices!;
        var obsIndices = obstacle.MeshIndices!;
        var invObs = Transforms.Inverse(Transforms.FromFrame(obstacle.Pose));

        for (var tri = 0; tri < robotIndices.Count / 3; tri++)
        {
            var bi = tri * 3;
            TransformVertex(robotWorldM, robotVerts, robotIndices[bi], out var a0x, out var a0y, out var a0z);
            TransformVertex(robotWorldM, robotVerts, robotIndices[bi + 1], out var a1x, out var a1y, out var a1z);
            TransformVertex(robotWorldM, robotVerts, robotIndices[bi + 2], out var a2x, out var a2y, out var a2z);

            var cx = (a0x + a1x + a2x) / 3;
            var cy = (a0y + a1y + a2y) / 3;
            var cz = (a0z + a1z + a2z) / 3;
            var radius = Math.Max(
                Distance(cx, cy, cz, a0x, a0y, a0z),
                Math.Max(Distance(cx, cy, cz, a1x, a1y, a1z), Distance(cx, cy, cz, a2x, a2y, a2z)));

            Transforms.TransformPointInto(invObs, cx, cy, cz, out var lx, out var ly, out var lz);
            obstacleBvh.GetPotentialTriangles(new Frame(lx, ly, lz), radius, scratch.TriangleHits);

            var a0 = new Frame(a0x, a0y, a0z);
            var a1 = new Frame(a1x, a1y, a1z);
            var a2 = new Frame(a2x, a2y, a2z);

            foreach (var oTri in scratch.TriangleHits)
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

    private static bool MeshIntersectsObject(
        CollisionObject mesh,
        CollisionObject obstacle,
        Dictionary<int, BvhNode> bvhCache,
        CollisionQueryScratch? scratch)
    {
        if (mesh.MeshVertices is null || mesh.MeshIndices is null) return false;
        scratch ??= new CollisionQueryScratch();

        // Verts already in mesh.Pose frame (identity or world). Use FromFrame(pose) as worldM.
        var worldM = Transforms.FromFrame(mesh.Pose);
        // If pose is Identity and verts are world-space (legacy TransformVertices path), Pose=Identity works.
        // If Mesh() was created with worldFrame pose + local verts (new Transform), worldM applies pose.
        return LocalMeshIntersectsObject(mesh, worldM, obstacle, bvhCache, scratch);
    }

    private static bool CapsuleIntersectsObject(
        CollisionObject capsule, CollisionObject obstacle, Dictionary<int, BvhNode> bvhCache, CollisionQueryScratch? scratch)
    {
        foreach (var (center, radius) in SampleCapsule(capsule))
            if (SphereIntersectsObject(center, radius, obstacle, bvhCache, scratch))
                return true;
        return false;
    }

    private static bool SphereIntersectsObject(
        Frame center, double radius, CollisionObject obj, Dictionary<int, BvhNode> bvhCache, CollisionQueryScratch? scratch) =>
        obj.Shape switch
        {
            CollisionShape.Sphere => SphereSphereOverlap(center, radius, obj.Pose, obj.ExtentX),
            CollisionShape.Box => SphereBoxOverlap(center, radius, obj),
            CollisionShape.Capsule => CapsuleIntersectsObject(obj, CollisionObject.Sphere("_", center, radius), bvhCache, scratch),
            CollisionShape.Mesh => SphereMeshOverlap(center, radius, obj, bvhCache, scratch),
            CollisionShape.Plane => SpherePlaneOverlap(center, radius, obj),
            _ => false
        };

    /// <summary>Half-space: Motus local +X is free. Collide when signed distance &lt; radius.</summary>
    private static bool SpherePlaneOverlap(Frame center, double radius, CollisionObject plane)
    {
        var m = Transforms.FromFrame(plane.Pose);
        var nx = m[0]; var ny = m[4]; var nz = m[8];
        var signed = (center.X - plane.Pose.X) * nx + (center.Y - plane.Pose.Y) * ny + (center.Z - plane.Pose.Z) * nz;
        return signed < radius;
    }

    private static bool BoxPlaneOverlap(CollisionObject box, CollisionObject plane)
    {
        var hx = box.ExtentX; var hy = box.ExtentY; var hz = box.ExtentZ;
        var boxM = Transforms.FromFrame(box.Pose);
        Span<(double x, double y, double z)> corners =
        [
            (-hx, -hy, -hz), (hx, -hy, -hz), (-hx, hy, -hz), (hx, hy, -hz),
            (-hx, -hy, hz), (hx, -hy, hz), (-hx, hy, hz), (hx, hy, hz)
        ];
        foreach (var (ox, oy, oz) in corners)
        {
            var w = Transforms.TransformPoint(boxM, ox, oy, oz);
            if (SpherePlaneOverlap(new Frame(w[0], w[1], w[2]), 0, plane))
                return true;
        }
        return false;
    }

    private static bool BoxIntersectsObject(
        CollisionObject box, CollisionObject obstacle, Dictionary<int, BvhNode> bvhCache, CollisionQueryScratch? scratch)
    {
        // O(1) for plane: box vs half-space via 8 corners (still tiny)
        if (obstacle.Shape == CollisionShape.Plane)
            return BoxPlaneOverlap(box, obstacle);

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
            if (SphereIntersectsObject(pt, 1e-3, obstacle, bvhCache, scratch)) return true;
            if (SphereIntersectsObject(pt, Math.Max(hx, Math.Max(hy, hz)) * 0.5, obstacle, bvhCache, scratch)) return true;
        }
        return SphereIntersectsObject(box.Pose, Math.Max(hx, Math.Max(hy, hz)), obstacle, bvhCache, scratch);
    }

    private static bool SphereMeshOverlap(
        Frame linkCenter, double linkRadius, CollisionObject mesh, Dictionary<int, BvhNode> bvhCache, CollisionQueryScratch? scratch)
    {
        if (!TryGetMeshBvh(bvhCache, mesh, out var bvh))
            return SphereAabbOverlap(linkCenter, linkRadius, mesh);

        var localSphere = Transforms.ToFrame(
            Transforms.Multiply(Transforms.Inverse(Transforms.FromFrame(mesh.Pose)), Transforms.FromFrame(linkCenter)));
        if (!bvh.OverlapsSphere(localSphere, linkRadius)) return false;

        if (mesh.MeshIndices is null || mesh.MeshVertices is null) return false;
        scratch ??= new CollisionQueryScratch();
        bvh.GetPotentialTriangles(localSphere, linkRadius, scratch.TriangleHits);
        foreach (var triIdx in scratch.TriangleHits)
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

    public static void TransformLocalAabbToWorld(CollisionObject mesh, double[] worldM, double[] minOut, double[] maxOut)
    {
        if (mesh.MeshAabbMin is null || mesh.MeshAabbMax is null)
        {
            minOut[0] = minOut[1] = minOut[2] = 0;
            maxOut[0] = maxOut[1] = maxOut[2] = 0;
            return;
        }

        var min = mesh.MeshAabbMin;
        var max = mesh.MeshAabbMax;
        minOut[0] = minOut[1] = minOut[2] = double.PositiveInfinity;
        maxOut[0] = maxOut[1] = maxOut[2] = double.NegativeInfinity;

        for (var i = 0; i < 8; i++)
        {
            var x = (i & 1) == 0 ? min[0] : max[0];
            var y = (i & 2) == 0 ? min[1] : max[1];
            var z = (i & 4) == 0 ? min[2] : max[2];
            Transforms.TransformPointInto(worldM, x, y, z, out var wx, out var wy, out var wz);
            if (wx < minOut[0]) minOut[0] = wx;
            if (wy < minOut[1]) minOut[1] = wy;
            if (wz < minOut[2]) minOut[2] = wz;
            if (wx > maxOut[0]) maxOut[0] = wx;
            if (wy > maxOut[1]) maxOut[1] = wy;
            if (wz > maxOut[2]) maxOut[2] = wz;
        }
    }

    public static bool AabbAabbOverlap(double[] minA, double[] maxA, double[] minB, double[] maxB) =>
        minA[0] <= maxB[0] && maxA[0] >= minB[0] &&
        minA[1] <= maxB[1] && maxA[1] >= minB[1] &&
        minA[2] <= maxB[2] && maxA[2] >= minB[2];

    private static bool WorldAabbMayHitObstacle(double[] min, double[] max, CollisionObject obstacle)
    {
        if (obstacle.Shape == CollisionShape.Mesh &&
            obstacle.MeshAabbMin is not null &&
            obstacle.MeshAabbMax is not null)
        {
            var obsM = Transforms.FromFrame(obstacle.Pose);
            var omin = new double[3];
            var omax = new double[3];
            TransformLocalAabbToWorld(obstacle, obsM, omin, omax);
            return AabbAabbOverlap(min, max, omin, omax);
        }

        if (obstacle.Shape == CollisionShape.Sphere)
        {
            var cx = Math.Clamp(obstacle.Pose.X, min[0], max[0]);
            var cy = Math.Clamp(obstacle.Pose.Y, min[1], max[1]);
            var cz = Math.Clamp(obstacle.Pose.Z, min[2], max[2]);
            var dx = obstacle.Pose.X - cx;
            var dy = obstacle.Pose.Y - cy;
            var dz = obstacle.Pose.Z - cz;
            var r = obstacle.ExtentX;
            return dx * dx + dy * dy + dz * dz < r * r;
        }

        // Box/capsule: conservative accept (narrowphase decides)
        return true;
    }

    private static void TransformVertex(double[] worldM, List<double[]> verts, int idx, out double x, out double y, out double z)
    {
        var v = verts[idx];
        Transforms.TransformPointInto(worldM, v[0], v[1], v[2], out x, out y, out z);
    }

    private static double Distance(double ax, double ay, double az, double bx, double by, double bz)
    {
        var dx = ax - bx;
        var dy = ay - by;
        var dz = az - bz;
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
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
        Transforms.TransformPointInto(inv, point.X, point.Y, point.Z, out var x, out var y, out var z);
        return new Frame(x, y, z);
    }

    private static bool TryGetMeshBvh(Dictionary<int, BvhNode> cache, CollisionObject mesh, out BvhNode bvh)
    {
        bvh = null!;
        if (mesh.Shape != CollisionShape.Mesh ||
            mesh.MeshVertices is null ||
            mesh.MeshIndices is null)
            return false;
        var key = CollisionMeshCache.GeometryFingerprint(mesh);
        if (cache.TryGetValue(key, out bvh!))
            return true;
        // Fall back to shared process cache
        bvh = CollisionMeshCache.GetOrBuild(mesh);
        cache[key] = bvh;
        return true;
    }
}
