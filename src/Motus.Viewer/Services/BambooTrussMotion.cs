using Motus.Core;
using K = Motus.Viewer.Services.BambooGripKinematics;

namespace Motus.Viewer.Services;

/// <summary>
/// Motus string-identity tasks and motion keys for /bamboo-truss. Each member is a pick (single store feed) →
/// place (cane exactly on its truss edge) pair, built by <see cref="BambooTrussPlanner"/> from a grounded base:
/// the L-grip clamped on anchor stake S1. After the chord is double-fixed the arm walks with base swaps.
/// </summary>
public static class BambooTrussMotion
{
    public sealed class CycleBound
    {
        public required string MemberId { get; init; }
        public double T0 { get; init; }
        public double T1 { get; init; }
        /// <summary>Key index range [First, Last] of this cycle in <see cref="TrussBuild.Keys"/>.</summary>
        public int FirstKey { get; init; }
        public int LastKey { get; init; }
    }

    public sealed class TrussBuild
    {
        public string Error { get; set; } = "";
        public List<BambooIcdMotion.MotionKey> Keys { get; set; } = new();
        public List<CycleBound> Cycles { get; set; } = new();
        public List<BambooIcdMotion.TaskEdit> Tasks { get; set; } = new();
        /// <summary>Every clamp / grasp / placement with its FK residual vs the target frame.</summary>
        public List<BambooTrussPlanner.GripRecord> Grips { get; set; } = new();
        /// <summary>Planner notes (grip choices, limits) for the page and the report.</summary>
        public List<string> Notes { get; set; } = new();
        /// <summary>Members actually built (in order).</summary>
        public List<string> Built { get; set; } = new();
        public int MotusChecks { get; set; }
    }

    /// <summary>Max IK candidates tried per Motus-checked stage (WASM budget).</summary>
    public const int MotusCandMax = 6;
    /// <summary>Beam width kept for linking (WASM budget).</summary>
    public const int LinkBeamWidth = 5;

    static readonly object CellGate = new();
    static BambooIcdCell? MotusCell;

    /// <summary>
    /// Bind the Motus collision cell used while planning. Required under Blazor WASM (no disk URDF);
    /// the /bamboo-truss page passes the same cell the viewer scan uses so planner Motus == Play Motus.
    /// </summary>
    public static void UseCell(BambooIcdCell cell)
    {
        ArgumentNullException.ThrowIfNull(cell);
        BambooTrussLayout.Configure(cell);
        lock (CellGate) MotusCell = cell;
        cell.SetMountViewer(BambooTrussLayout.StartBase);
    }

    static BambooIcdCell? TryCell()
    {
        if (MotusCell is not null) return MotusCell;
        lock (CellGate)
        {
            if (MotusCell is not null) return MotusCell;
            try
            {
                var urdf = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..",
                    "tests", "fixtures", "bamboo_icd", "bamboo_icd.urdf"));
                if (!File.Exists(urdf))
                    urdf = Path.GetFullPath(Path.Combine("tests", "fixtures", "bamboo_icd", "bamboo_icd.urdf"));
                if (!File.Exists(urdf)) return null;
                var cell = BambooIcdCell.LoadFile(urdf, BambooTrussLayout.StrutLength);
                BambooTrussLayout.Configure(cell);
                MotusCell = cell;
                return MotusCell;
            }
            catch
            {
                return null; // WASM / missing fixture: caller must UseCell.
            }
        }
    }

    const double StoreApproachDy = 0.04;
    const double PlaceHoldDy = 0.04;

    public static List<BambooIcdMotion.TaskEdit> DefaultTasks(IReadOnlyList<BambooTrussLayout.MemberDef>? members = null)
    {
        var tasks = new List<BambooIcdMotion.TaskEdit>();
        foreach (var m in members ?? BambooTrussLayout.Members)
        {
            tasks.Add(new BambooIcdMotion.TaskEdit
            {
                Identity = "pick", Object = m.Id, JawMm = BambooTrussLayout.JawClosedMm,
                Planes = [Plane("approach", m.Store, StoreApproachDy), Plane("grasp", m.Store, 0)]
            });
            tasks.Add(new BambooIcdMotion.TaskEdit
            {
                Identity = "place", Object = m.Id, JawMm = BambooTrussLayout.JawOpenMm,
                Planes = [Plane("hold", m.Place, PlaceHoldDy), Plane("release", m.Place, 0)]
            });
        }
        return tasks;
    }

    static BambooIcdMotion.PlaneEdit Plane(string name, BambooIcdCell.StrutPlacement p, double dy) => new()
    {
        Name = name, X = p.X, Y0 = p.Y + dy, Z = p.Z, PitchDeg = p.PitchDeg, YawDeg = p.YawDeg, YMm = 0
    };

    public static List<(BambooIcdMotion.TaskEdit Pick, BambooIcdMotion.TaskEdit Place, string MemberId)> Pairs(
        IReadOnlyList<BambooIcdMotion.TaskEdit> tasks)
    {
        var pairs = new List<(BambooIcdMotion.TaskEdit, BambooIcdMotion.TaskEdit, string)>();
        BambooIcdMotion.TaskEdit? pendingPick = null;
        string? pendingId = null;
        foreach (var t in tasks)
        {
            if (t.Identity == "pick") { pendingPick = t; pendingId = t.Object; }
            else if (t.Identity == "place" && pendingPick is not null && string.Equals(pendingId, t.Object, StringComparison.Ordinal))
            {
                pairs.Add((pendingPick, t, t.Object));
                pendingPick = null; pendingId = null;
            }
        }
        return pairs;
    }

    public static TrussBuild Build(IReadOnlyList<BambooIcdMotion.TaskEdit>? tasks = null) =>
        BuildAsync(tasks).GetAwaiter().GetResult();

    /// <summary>Plan phase by phase, awaiting <paramref name="onMember"/> between phases so Blazor WASM can paint.</summary>
    public static async Task<TrussBuild> BuildAsync(
        IReadOnlyList<BambooIcdMotion.TaskEdit>? tasks = null,
        Func<string, Task>? onMember = null)
    {
        tasks ??= DefaultTasks();
        var pairs = Pairs(tasks);
        if (pairs.Count == 0)
            return new TrussBuild { Error = "No pick/place pairs (object-matched).", Tasks = tasks.ToList() };
        var cell = TryCell();
        if (cell is null)
            return new TrussBuild { Error = "Motus cell required (call UseCell with the viewer BambooIcdCell).", Tasks = tasks.ToList() };

        async Task Yield(string what)
        {
            if (onMember is not null) await onMember(what);
            else await Task.Yield();
        }

        var planner = new BambooTrussPlanner(cell);
        var built = new List<string>();
        var notes = new List<string>();
        await Yield("start");
        if (!planner.Start())
            return new TrussBuild { Error = planner.Error, Tasks = tasks.ToList() };
        foreach (var (_, _, memberId) in pairs)
        {
            var member = BambooTrussLayout.Members.FirstOrDefault(m => m.Id == memberId);
            if (member is null) { notes.Add($"{memberId}: not a truss member — skipped"); break; }
            if (built.Contains(memberId)) { notes.Add($"{memberId}: already built — skipped"); continue; }
            await Yield(memberId);
            if (!planner.BuildMember(member))
            {
                notes.Add($"{memberId}: not built — {planner.Error}");
                break;
            }
            built.Add(memberId);
        }
        if (built.Contains("baseL") && built.Contains("baseR"))
        {
            await Yield("walk");
            if (!planner.Walk()) notes.Add($"walk: not planned — {planner.Error}");
        }
        else notes.Add("walk: skipped (chord not double-fixed)");
        planner.StampTimes();
        cell.SetMountViewer(BambooTrussLayout.StartBase);

        var keys = planner.Keys;
        var cycles = new List<CycleBound>();
        foreach (var (name, first, last) in planner.CycleKeys)
        {
            if (last < first) continue;
            cycles.Add(new CycleBound
            {
                MemberId = name, FirstKey = first, LastKey = last,
                T0 = first > 0 ? keys[first - 1].T : 0, T1 = keys[last].T
            });
        }
        notes.InsertRange(0, planner.Notes);
        return new TrussBuild
        {
            Error = "", Keys = keys, Cycles = cycles, Tasks = tasks.ToList(), Grips = planner.Grips,
            Notes = notes, Built = built, MotusChecks = planner.Checks
        };
    }

    /// <summary>Full interpolated state at t (joints, both jaw openings, hold) plus the index of the key ending the segment.</summary>
    public readonly record struct State(double[] Q, double GR, double GL, int Hold, string Label, int SegEnd);

    public static State SampleState(IReadOnlyList<BambooIcdMotion.MotionKey> keys, double t)
    {
        if (keys.Count == 0) return new State(BambooIcdMotion.HomeDeg.ToArray(), 40, 24, 0, "Home", 0);
        if (t <= keys[0].T) return new State(keys[0].Q.ToArray(), keys[0].GR, keys[0].GL, keys[0].Hold, keys[0].Label, 0);
        if (t >= 1 || t >= keys[^1].T)
        {
            var l = keys[^1];
            return new State(l.Q.ToArray(), l.GR, l.GL, l.Hold, l.Label, keys.Count - 1);
        }
        var i = 0;
        while (i < keys.Count - 2 && keys[i + 1].T < t) i++;
        var a = keys[i]; var b = keys[i + 1];
        var denom = b.T - a.T;
        var u = denom <= 1e-12 ? 1 : (t - a.T) / denom;
        var e = u * u * (3 - 2 * u);
        var q = new double[5];
        for (var k = 0; k < 5; k++) q[k] = a.Q[k] + (b.Q[k] - a.Q[k]) * e;
        return new State(q, a.GR + (b.GR - a.GR) * e, a.GL + (b.GL - a.GL) * e,
            u >= 1 ? b.Hold : a.Hold, e < 0.5 ? a.Label : b.Label, i + 1);
    }

    public static string? MemberIdAt(IReadOnlyList<CycleBound> cycles, double t)
    {
        foreach (var c in cycles)
            if (t >= c.T0 - 1e-12 && t <= c.T1 + 1e-12) return c.MemberId;
        return cycles.Count > 0 ? cycles[^1].MemberId : null;
    }

    public static int MemberIndexAt(IReadOnlyList<CycleBound> cycles, double t)
    {
        for (var i = 0; i < cycles.Count; i++)
            if (t >= cycles[i].T0 - 1e-12 && t <= cycles[i].T1 + 1e-12) return i;
        return Math.Max(0, cycles.Count - 1);
    }
}
