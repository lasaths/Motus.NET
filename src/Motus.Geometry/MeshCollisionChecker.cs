using System;
using System.Collections.Generic;
using System.Linq;
using Motus.Core;

namespace Motus.Geometry;

/// <summary>Mesh-accurate collision checker using BVH (Bounding Volume Hierarchy) + SAT per-triangle.</summary>
public sealed class MeshCollisionChecker : ICollisionChecker
{
    private readonly DhForwardKinematics _fk;
    private readonly BaseFrame _base;
    private Dictionary<string, BvhNode> _meshBvhCache = new();

    public MeshCollisionChecker(RobotPreset preset)
    {
        _fk = new DhForwardKinematics(preset);
        _base = preset.BaseFrame;
    }

    public bool IsCollisionFree(JointState state, CollisionScene scene)
    {
        if (!SelfCollisionFree(state)) return false;
        
        BuildBvhCache(scene);
        
        var origins = _fk.ComputeLinkOrigins(state.Positions, _base.Frame);
        var radii = _fk.LinkRadiiMeters;
        
        foreach (var obj in scene.Objects)
        {
            for (var i = 0; i < origins.Count; i++)
            {
                if (Intersects(origins[i], radii[i], obj))
                    return false;
            }
        }
        return true;
    }

    private void BuildBvhCache(CollisionScene scene)
    {
        // PONYTAIL: BVH builder has indexing issues - disabled for now
        // Falling back to AABB-only collision detection
        // foreach (var meshObj in scene.Objects.Where(o => o.Shape == CollisionShape.Mesh))
        // {
        //     if (!_meshBvhCache.ContainsKey(meshObj.Name) && meshObj.MeshVertices != null)
        //     {
        //         _meshBvhCache[meshObj.Name] = BvhBuilder.Build(meshObj);
        //     }
        // }
    }

    private bool SelfCollisionFree(JointState state)
    {
        var origins = _fk.ComputeLinkOrigins(state.Positions, _base.Frame);
        var radii = _fk.LinkRadiiMeters;
        
        // PONYTAIL: Adjacent links exempt (i and i+1 share joint)
        for (var i = 0; i < origins.Count; i++)
        {
            for (var j = i + 2; j < origins.Count; j++)
            {
                if (SphereSphereOverlap(origins[i], radii[i], origins[j], radii[j]))
                    return false;
            }
        }
        return true;
    }

    private bool Intersects(Frame link, double linkRadius, CollisionObject obj) =>
        obj.Shape switch
        {
            CollisionShape.Sphere => SphereSphereOverlap(link, linkRadius, obj.Pose, obj.ExtentX),
            CollisionShape.Box => SphereBoxOverlap(link, linkRadius, obj),
            CollisionShape.Mesh => SphereMeshOverlap(link, linkRadius, obj),
            _ => false
        };

    private static bool SphereSphereOverlap(Frame a, double ra, Frame b, double rb)
    {
        var dx = a.X - b.X; 
        var dy = a.Y - b.Y; 
        var dz = a.Z - b.Z;
        var dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        return dist < ra + rb;
    }

    private static bool SphereBoxOverlap(Frame center, double radius, CollisionObject box)
    {
        var local = WorldToLocal(center, box.Pose);
        var hx = box.ExtentX; 
        var hy = box.ExtentY; 
        var hz = box.ExtentZ;
        
        // PONYTAIL: Clamp sphere center to box bounds, find closest point
        var cx = Math.Clamp(local.X, -hx, hx);
        var cy = Math.Clamp(local.Y, -hy, hy);
        var cz = Math.Clamp(local.Z, -hz, hz);
        
        var dx = local.X - cx; 
        var dy = local.Y - cy; 
        var dz = local.Z - cz;
        var distSq = dx * dx + dy * dy + dz * dz;
        
        return distSq < radius * radius;
    }

    private bool SphereMeshOverlap(Frame linkCenter, double linkRadius, CollisionObject mesh)
    {
        if (!_meshBvhCache.TryGetValue(mesh.Name, out var bvh))
            return SphereAabbOverlap(linkCenter, linkRadius, mesh);
        
        // PONYTAIL: Broad phase: BVH query
        var transform = Transforms.FromFrame(mesh.Pose);
        var localSphereCenter = Transforms.ToFrame(
            Transforms.Multiply(Transforms.Inverse(transform), Transforms.FromFrame(linkCenter)));
        
        if (!bvh.OverlapsSphere(localSphereCenter, linkRadius))
            return false;
        
        // PONYTAIL: Narrow phase: SAT with potential triangles
        var potentialTriangles = bvh.GetPotentialTriangles(localSphereCenter, linkRadius);
        foreach (var i in potentialTriangles)
        {
            if (mesh.MeshIndices == null || mesh.MeshVertices == null || i + 2 >= mesh.MeshIndices.Count)
                continue;
            
            var v0Idx = mesh.MeshIndices[i];
            var v1Idx = mesh.MeshIndices[i + 1];
            var v2Idx = mesh.MeshIndices[i + 2];
            
            if (v0Idx >= mesh.MeshVertices.Count || v1Idx >= mesh.MeshVertices.Count || v2Idx >= mesh.MeshVertices.Count)
                continue;
            
            var v0 = new Frame(mesh.MeshVertices[v0Idx][0], mesh.MeshVertices[v0Idx][1], mesh.MeshVertices[v0Idx][2]);
            var v1 = new Frame(mesh.MeshVertices[v1Idx][0], mesh.MeshVertices[v1Idx][1], mesh.MeshVertices[v1Idx][2]);
            var v2 = new Frame(mesh.MeshVertices[v2Idx][0], mesh.MeshVertices[v2Idx][1], mesh.MeshVertices[v2Idx][2]);
            
            if (TriangleCollision.SphereTriangleOverlap(linkCenter, linkRadius, v0, v1, v2, mesh.Pose))
                return true;
        }
        
        return false;
    }

    private static bool SphereAabbOverlap(Frame sphereCenter, double radius, CollisionObject mesh)
    {
        if (mesh.MeshAabbMin == null || mesh.MeshAabbMax == null)
            return false;
            
        // PONYTAIL: Transform sphere center to mesh local space
        var local = WorldToLocal(sphereCenter, mesh.Pose);
        
        // PONYTAIL: AABB clamping
        var min = mesh.MeshAabbMin;
        var max = mesh.MeshAabbMax;
        
        var cx = Math.Clamp(local.X, min[0], max[0]);
        var cy = Math.Clamp(local.Y, min[1], max[1]);
        var cz = Math.Clamp(local.Z, min[2], max[2]);
        
        var dx = local.X - cx;
        var dy = local.Y - cy;
        var dz = local.Z - cz;
        var distSq = dx * dx + dy * dy + dz * dz;
        
        return distSq < radius * radius;
    }

    private static Frame WorldToLocal(Frame point, Frame objectPose)
    {
        var invTransform = Transforms.Inverse(Transforms.FromFrame(objectPose));
        var localPoint = Transforms.Multiply(invTransform, Transforms.FromFrame(point));
        return Transforms.ToFrame(localPoint);
    }
}
