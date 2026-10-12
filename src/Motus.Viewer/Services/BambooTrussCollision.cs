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

    /// <summary>Motus contacts of one pose: robot, both grippers, mount parts and held cane vs the environment.</summary>
    public static IReadOnlyList<CollisionContact> Contacts(BambooIcdCell cell, string? baseGripper, double[] baseWorld,
        IReadOnlyList<double> qDeg, double gR, double gL, Frame? held, IReadOnlyList<CollisionObject> env,
        string? lOn, string? rOn, int max)
    {
        cell.SetMountViewer(BambooTrussPlanner.LWorld(baseGripper, baseWorld, qDeg));
        return cell.PoseContacts(qDeg, gR, gL, held, env, Allowed(lOn, rOn), max);
    }
}
