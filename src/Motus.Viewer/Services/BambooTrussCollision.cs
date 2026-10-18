using Motus.Core;
using Motus.Geometry;

namespace Motus.Viewer.Services;

/// <summary>
/// The one collision check of /bamboo-truss, shared by the planner (segment checks) and <see cref="BambooTrussScan"/>
/// (playback / tests): same environment, same allowed touches, same canonical sample points on a segment.
/// Agreement rule: the planner's verify pass checks every canonical point of a segment (plus finer ones), and the scan
/// checks exactly the canonical points, so a segment the planner accepts is a segment the scan accepts.
/// </summary>
public static class BambooTrussCollision
{
    /// <summary>
    /// Canonical joint-space fractions (0, 1] of a segment with joint travel <paramref name="dqDeg"/> and jaw travel
    /// <paramref name="dgMm"/>: n = ⌈oversample · max(Δq / scan step, Δg / jaw step)⌉, fractions s / n.
    /// </summary>
    public static int CanonicalCount(double dqDeg, double dgMm, MotionConstraints c) =>
        Math.Max(1, (int)Math.Ceiling(c.ScanOversample * Math.Max(dqDeg / c.ScanStepDeg, dgMm / c.JawStepMm)));

    /// <summary>
    /// Playback time fraction u at which the eased interpolation (smoothstep u²(3 − 2u), see
    /// <see cref="BambooTrussMotion.SampleState"/>) reaches joint-space fraction <paramref name="f"/>.
    /// </summary>
    public static double TimeForFraction(double f)
    {
        if (f <= 0) return 0;
        if (f >= 1) return 1;
        return 0.5 - Math.Sin(Math.Asin(1 - 2 * f) / 3);
    }

    /// <summary>
    /// Environment for a hold state: ground + fixtures, standing members (+ their node connectors), and the active cane
    /// (resting on the feed, or released in place); between members the next cane lies on the stock.
    /// </summary>
    public static List<CollisionObject> Environment(BambooIcdCell cell, IEnumerable<(string Name, double[] World)> placed,
        string? activeId, double[]? rest, double[]? released, bool heldTracked, int hold)
    {
        var placedList = placed.ToList();
        var env = new List<CollisionObject>(cell.GroundObjects);
        env.AddRange(BambooTrussLayout.FixtureObjects);
        foreach (var (name, w) in placedList) env.Add(cell.StrutAt(w, name));
        var hubIds = placedList.Select(p => p.Name["placed-".Length..]).ToList();
        if (activeId is not null && hold == 2 && released is not null) hubIds.Add(activeId);
        env.AddRange(BambooTrussLayout.HubObjects(hubIds));
        if (activeId is null)
        {
            env.Add(cell.StrutAt(rest ?? BambooIcdCell.PlacementWorld(BambooTrussLayout.Store), BambooIcdCell.Strut));
            return env;
        }
        if (hold == 0 && rest is not null) env.Add(cell.StrutAt(rest, BambooIcdCell.Strut));
        else if (hold == 2 && released is not null) env.Add(cell.StrutAt(released, "placed-" + activeId));
        else if (hold == 1 && !heldTracked && rest is not null) env.Add(cell.StrutAt(rest, BambooIcdCell.Strut));
        return env;
    }

    /// <summary>Allowed touches: the jaw plates of each gripper on what it clamps (and only that).</summary>
    public static List<(string A, string B)> Allowed(string? lOn, string? rOn)
    {
        var list = new List<(string, string)>();
        if (!string.IsNullOrEmpty(lOn))
            foreach (var o in lOn.Split('|'))
                foreach (var p in BambooIcdCell.LGripPlates) list.Add((p, o));
        if (!string.IsNullOrEmpty(rOn))
            foreach (var o in rOn.Split('|'))
                foreach (var p in BambooIcdCell.RGripPlates) list.Add((p, o));
        return list;
    }

    /// <summary>
    /// Motus contacts of one pose: robot, both grippers, mount parts and held cane vs the environment. With a clearance
    /// margin (<see cref="MotionConstraints.ClearanceMarginM"/>) every obstacle is grown by it (<see cref="Inflate"/>),
    /// so anything closer than the margin is a contact. Exempt (intended contacts only): the objects a gripper clamps
    /// (<paramref name="lOn"/> / <paramref name="rOn"/>: exact contact, jaw plates allowed), and the held cane resting on
    /// its stock pegs (held cane ↔ store peg is checked exactly; the robot ↔ peg keeps the margin).
    /// </summary>
    public static IReadOnlyList<CollisionContact> Contacts(BambooIcdCell cell, string? baseGripper, double[] baseWorld,
        IReadOnlyList<double> qDeg, double gR, double gL, Frame? held, IReadOnlyList<CollisionObject> env,
        string? lOn, string? rOn, int max, double marginM = 0)
    {
        cell.SetMountViewer(BambooTrussPlanner.LWorld(baseGripper, baseWorld, qDeg));
        var allowed = Allowed(lOn, rOn);
        if (marginM <= 0) return cell.PoseContacts(qDeg, gR, gL, held, env, allowed, max);
        var clamped = $"{lOn}|{rOn}".Split('|', StringSplitOptions.RemoveEmptyEntries);
        var grown = env.Select(o => clamped.Contains(o.Name) ? o : Inflate(o, marginM)).ToList();
        if (held is null) return cell.PoseContacts(qDeg, gR, gL, held, grown, allowed, max);
        var all = cell.PoseContacts(qDeg, gR, gL, held, grown, allowed, max + 8);
        var kept = all.Where(c => !IsPegRest(c)).ToList();
        if (kept.Count < all.Count && kept.Count < max)
        {
            // The resting contact is exempt from the margin only: the held cane must still not touch a peg exactly.
            var pegs = env.Where(o => o.Name.StartsWith(PegPrefix, StringComparison.Ordinal)).ToList();
            kept.AddRange(cell.PoseContacts(qDeg, gR, gL, held, pegs, allowed, max).Where(IsPegRest));
        }
        return kept.Take(max).ToList();
    }

    const string PegPrefix = "store-peg-";

    static bool IsPegRest(CollisionContact c) =>
        (c.BodyA == BambooIcdCell.Strut && c.BodyB.StartsWith(PegPrefix, StringComparison.Ordinal)) ||
        (c.BodyB == BambooIcdCell.Strut && c.BodyA.StartsWith(PegPrefix, StringComparison.Ordinal));

    static readonly System.Runtime.CompilerServices.ConditionalWeakTable<CollisionObject, Tuple<double, CollisionObject>> _grown = new();

    /// <summary>
    /// Obstacle grown by <paramref name="d"/>: primitives exactly (box corners conservatively); meshes by moving each
    /// vertex along its normal far enough that every adjacent face plane moves out by at least d. Cached per object.
    /// </summary>
    public static CollisionObject Inflate(CollisionObject o, double d)
    {
        if (_grown.TryGetValue(o, out var c) && c.Item1 == d) return c.Item2;
        var g = InflateCore(o, d);
        _grown.AddOrUpdate(o, Tuple.Create(d, g));
        return g;
    }

    static CollisionObject InflateCore(CollisionObject o, double d)
    {
        switch (o.Shape)
        {
            case CollisionShape.Sphere: return CollisionObject.Sphere(o.Name, o.Pose, o.ExtentX + d);
            case CollisionShape.Box: return CollisionObject.Box(o.Name, o.Pose, o.ExtentX + d, o.ExtentY + d, o.ExtentZ + d);
            case CollisionShape.Capsule: return CollisionObject.Capsule(o.Name, o.Pose, o.ExtentX + d, o.ExtentY);
            case CollisionShape.Plane:
            {
                var m = Transforms.FromFrame(o.Pose); // free side = local +X
                return CollisionObject.Plane(o.Name, new Frame(o.Pose.X + m[0] * d, o.Pose.Y + m[4] * d, o.Pose.Z + m[8] * d, o.Pose.Qw, o.Pose.Qx, o.Pose.Qy, o.Pose.Qz));
            }
            default:
            {
                var v = o.MeshVertices!; var idx = o.MeshIndices!;
                var faces = new List<int>[v.Count];
                var fn = new List<double[]>();
                for (var i = 0; i + 2 < idx.Count; i += 3)
                {
                    var a = v[idx[i]]; var b = v[idx[i + 1]]; var cc = v[idx[i + 2]];
                    double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2], wx = cc[0] - a[0], wy = cc[1] - a[1], wz = cc[2] - a[2];
                    double nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
                    var l = Math.Sqrt(nx * nx + ny * ny + nz * nz);
                    if (l < 1e-15) continue;
                    fn.Add(new[] { nx / l, ny / l, nz / l });
                    foreach (var j in new[] { idx[i], idx[i + 1], idx[i + 2] }) (faces[j] ??= new()).Add(fn.Count - 1);
                }
                var nv = new List<double[]>(v.Count);
                for (var i = 0; i < v.Count; i++)
                {
                    if (faces[i] is null) { nv.Add(v[i].ToArray()); continue; }
                    double sx = 0, sy = 0, sz = 0;
                    foreach (var f in faces[i]) { sx += fn[f][0]; sy += fn[f][1]; sz += fn[f][2]; }
                    var l = Math.Sqrt(sx * sx + sy * sy + sz * sz);
                    if (l < 1e-12) { nv.Add(v[i].ToArray()); continue; }
                    sx /= l; sy /= l; sz /= l;
                    var minDot = faces[i].Min(f => sx * fn[f][0] + sy * fn[f][1] + sz * fn[f][2]);
                    var s = d / Math.Max(minDot, 1.0 / 3);
                    nv.Add(new[] { v[i][0] + sx * s, v[i][1] + sy * s, v[i][2] + sz * s });
                }
                return CollisionObject.Mesh(o.Name, o.Pose, nv, idx);
            }
        }
    }
}
