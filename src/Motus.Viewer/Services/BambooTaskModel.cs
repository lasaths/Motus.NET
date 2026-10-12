using Motus.Core;

namespace Motus.Viewer.Services;

/// <summary>
/// Binds the viewer's editable task blocks (<see cref="BambooIcdMotion.TaskEdit"/>) to the core task types
/// (<see cref="CoreTasks"/>: pick, place, walk, park). Each plane becomes a named core <see cref="Frame"/> (position, and
/// orientation R = Ry(yaw) · Rz(pitch), cane axis = local +X); the exact plane values ride along as metadata so the
/// round trip is lossless.
/// </summary>
public static class BambooTaskModel
{
    const string PlaneKey = "plane.";

    public static TaskInstance ToCore(BambooIcdMotion.TaskEdit t)
    {
        var frames = t.Planes.ToDictionary(p => p.Name, ToFrame);
        TaskInstance core = t.Identity switch
        {
            TaskIdentities.Walk => CoreTasks.Walk(t.Object, t.Gait == BambooTrussMotion.LandGait ? WalkMode.Land : WalkMode.Node,
                t.Gait == BambooTrussMotion.LandGait ? null : t.Gait, frames),
            TaskIdentities.Park => CoreTasks.Park(t.Object, ViaNode(t.Via), ViaDir(t.Via), t.JawMm, frames),
            TaskIdentities.Place => CoreTasks.Place(t.Object, t.JawMm, frames),
            TaskIdentities.Pick => CoreTasks.Pick(t.Object, t.JawMm, frames),
            _ => new TaskInstance(t.Identity, frames, new Dictionary<string, object> { [TaskMetadataKeys.Object] = t.Object, [TaskMetadataKeys.JawMm] = t.JawMm })
        };
        foreach (var p in t.Planes)
            core = core.WithMetadataValue(PlaneKey + p.Name, new[] { p.X, p.Y0, p.Z, p.PitchDeg, p.YawDeg, p.YMm });
        if (t.Identity == TaskIdentities.Walk && t.Gait is not null) core = core.WithMetadataValue("bamboo.gait", t.Gait);
        return core;
    }

    public static BambooIcdMotion.TaskEdit FromCore(TaskInstance c)
    {
        var e = new BambooIcdMotion.TaskEdit
        {
            Identity = c.Identity,
            Object = CoreTasks.ObjectOf(c),
            JawMm = c.Metadata.TryGetValue(TaskMetadataKeys.JawMm, out var j) && j is IConvertible jc ? jc.ToDouble(System.Globalization.CultureInfo.InvariantCulture) : BambooTrussLayout.JawClosedMm,
        };
        if (CoreTasks.AsWalk(c) is { } w)
            e.Gait = c.Metadata.TryGetValue("bamboo.gait", out var g) ? g.ToString() : w.Mode == WalkMode.Land ? BambooTrussMotion.LandGait : w.Gait;
        if (CoreTasks.AsPark(c) is { } pk) e.Via = $"{pk.Connector}/{pk.Direction}";
        foreach (var (name, f) in c.Frames)
        {
            if (c.Metadata.TryGetValue(PlaneKey + name, out var raw) && raw is double[] v && v.Length == 6)
                e.Planes.Add(new BambooIcdMotion.PlaneEdit { Name = name, X = v[0], Y0 = v[1], Z = v[2], PitchDeg = v[3], YawDeg = v[4], YMm = v[5] });
            else
            {
                var (pitch, yaw) = PitchYaw(f);
                e.Planes.Add(new BambooIcdMotion.PlaneEdit { Name = name, X = f.X, Y0 = f.Y, Z = f.Z, PitchDeg = pitch, YawDeg = yaw });
            }
        }
        return e;
    }

    public static List<TaskInstance> ToCore(IEnumerable<BambooIcdMotion.TaskEdit> tasks) => tasks.Select(ToCore).ToList();

    /// <summary>The bamboo id of a park task's parked pose (relay id, <see cref="BambooTrussLayout.RelayId"/>).</summary>
    public static string RelayIdOf(ParkTask p) => BambooTrussLayout.RelayId(p.Object, p.Connector, p.Direction);

    static string ViaNode(string? via) => via is null ? "" : via.Split('/')[0];
    static string ViaDir(string? via) => via is null || !via.Contains('/') ? "" : via[(via.IndexOf('/') + 1)..];

    /// <summary>Plane → frame: position (X, Y0 + YMm/1000, Z); R = Ry(yaw) · Rz(pitch).</summary>
    public static Frame ToFrame(BambooIcdMotion.PlaneEdit p)
    {
        double hy = p.YawDeg * Math.PI / 360, hp = p.PitchDeg * Math.PI / 360;
        // q = qy(yaw) ⊗ qz(pitch)
        double yw = Math.Cos(hy), yy = Math.Sin(hy), pw = Math.Cos(hp), pz = Math.Sin(hp);
        return new Frame(p.X, p.Y0 + p.YMm / 1000, p.Z, yw * pw, yy * pz, yy * pw, yw * pz);
    }

    static (double Pitch, double Yaw) PitchYaw(Frame f)
    {
        // cane axis = R · (1, 0, 0)
        double w = f.Qw, x = f.Qx, y = f.Qy, z = f.Qz;
        var ax = 1 - 2 * (y * y + z * z); var ay = 2 * (x * y + w * z); var az = 2 * (x * z - w * y);
        var pitch = Math.Asin(Math.Clamp(ay, -1, 1)) * 180 / Math.PI;
        var yaw = Math.Atan2(-az, ax) * 180 / Math.PI;
        return (pitch, yaw);
    }
}
