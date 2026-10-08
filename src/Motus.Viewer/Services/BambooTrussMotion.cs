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

    public static TrussBuild Build(IReadOnlyList<BambooIcdMotion.TaskEdit>? tasks = null, bool useCache = false) =>
        BuildAsync(tasks, null, useCache).GetAwaiter().GetResult();

    // ---------------- build-time plan cache (WASM) ----------------

    /// <summary>Bump when planner behaviour changes (invalidates <see cref="DefaultPlanResource"/>).</summary>
    public const string PlannerVersion = "grounded-ik-1";
    public const string DefaultPlanResource = "Motus.Viewer.Services.BambooTrussDefaultPlan.json";

    /// <summary>Everything the planner's output depends on (layout, tolerances, version), invariant-formatted.</summary>
    public static string LayoutFingerprint
    {
        get
        {
            var ci = System.Globalization.CultureInfo.InvariantCulture;
            string F(double v) => v.ToString("R", ci);
            string P(BambooIcdCell.StrutPlacement p) => $"{F(p.X)},{F(p.Y)},{F(p.Z)},{F(p.PitchDeg)},{F(p.YawDeg)}";
            string St(BambooTrussLayout.Stake t) => $"{t.Name},{F(t.X)},{F(t.Z)},{F(t.ClampY)},{F(t.Top)},{F(t.HeadingDeg)}";
            var parts = new List<string>
            {
                PlannerVersion, F(BambooTrussLayout.StrutLength), St(BambooTrussLayout.S1), St(BambooTrussLayout.S2),
                P(BambooTrussLayout.Store), F(BambooTrussLayout.RackTop), F(K.GripPosTolM), F(K.GripAxisTolDeg),
                F(BambooTrussLayout.JawOpenMm), F(BambooTrussLayout.LJawOpenMm), F(BambooTrussLayout.JawClosedMm)
            };
            parts.AddRange(BambooTrussLayout.Members.Select(m => $"{m.Id}:{P(m.Place)}"));
            parts.AddRange(BambooTrussLayout.Pads.Select(p => $"{p.Name}:{F(p.X)},{F(p.Z)},{F(p.HalfX)},{F(p.HalfZ)}"));
            parts.AddRange(BambooTrussLayout.Posts.Select(p => $"{p.Name}:{F(p.X)},{F(p.Z)},{F(p.Top)},{F(p.Radius)}"));
            return string.Join(";", parts);
        }
    }

    public sealed class PlanCache
    {
        public string Fingerprint { get; set; } = "";
        public List<BambooIcdMotion.MotionKey> Keys { get; set; } = new();
        public List<CycleBound> Cycles { get; set; } = new();
        public List<BambooTrussPlanner.GripRecord> Grips { get; set; } = new();
        public List<string> Notes { get; set; } = new();
        public List<string> Built { get; set; } = new();
        public int MotusChecks { get; set; }
    }

    public static string SerializePlan(TrussBuild b) => System.Text.Json.JsonSerializer.Serialize(new PlanCache
    {
        Fingerprint = LayoutFingerprint, Keys = b.Keys, Cycles = b.Cycles, Grips = b.Grips,
        Notes = b.Notes.Where(n => !n.StartsWith("planned in", StringComparison.Ordinal)).ToList(),
        Built = b.Built, MotusChecks = b.MotusChecks
    });

    /// <summary>The default plan shipped with the app, if it matches the current layout/planner (else null → plan live).</summary>
    public static TrussBuild? LoadCachedDefaultPlan()
    {
        try
        {
            using var stream = typeof(BambooTrussMotion).Assembly.GetManifestResourceStream(DefaultPlanResource);
            if (stream is null) return null;
            var c = System.Text.Json.JsonSerializer.Deserialize<PlanCache>(stream);
            if (c is null || c.Fingerprint != LayoutFingerprint || c.Keys.Count == 0) return null;
            var notes = c.Notes.ToList();
            notes.Add("plan: shipped build-time plan for the default tasks (byte-identical to live planning — enforced by a test); Play still Motus-scans it live");
            return new TrussBuild { Keys = c.Keys, Cycles = c.Cycles, Grips = c.Grips, Notes = notes, Built = c.Built, MotusChecks = c.MotusChecks };
        }
        catch
        {
            return null;
        }
    }

    static bool IsDefaultMemberOrder(IReadOnlyList<BambooIcdMotion.TaskEdit> tasks) =>
        Pairs(tasks).Select(p => p.MemberId).SequenceEqual(BambooTrussLayout.Members.Select(m => m.Id));

    /// <summary>
    /// Plan phase by phase, awaiting <paramref name="onMember"/> between phases so Blazor WASM can paint. With
    /// <paramref name="useCache"/> the default member order uses the shipped plan (live planning takes minutes in
    /// interpreted WASM and cannot yield inside a phase).
    /// </summary>
    public static async Task<TrussBuild> BuildAsync(
        IReadOnlyList<BambooIcdMotion.TaskEdit>? tasks = null,
        Func<string, Task>? onMember = null,
        bool useCache = false)
    {
        tasks ??= DefaultTasks();
        var pairs = Pairs(tasks);
        if (pairs.Count == 0)
            return new TrussBuild { Error = "No pick/place pairs (object-matched).", Tasks = tasks.ToList() };
        if (useCache && IsDefaultMemberOrder(tasks) && LoadCachedDefaultPlan() is { } cached)
        {
            cached.Tasks = tasks.ToList();
            return cached;
        }
        var cell = TryCell();
        if (cell is null)
            return new TrussBuild { Error = "Motus cell required (call UseCell with the viewer BambooIcdCell).", Tasks = tasks.ToList() };

        async Task Yield(string what)
        {
            if (onMember is not null) await onMember(what);
            else await Task.Yield();
        }

        var clock = System.Diagnostics.Stopwatch.StartNew();
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
        notes.Add($"planned in {clock.Elapsed.TotalSeconds:F1} s ({planner.Checks} Motus pose checks)");
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
