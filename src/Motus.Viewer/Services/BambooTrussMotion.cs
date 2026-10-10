using Motus.Core;
using K = Motus.Viewer.Services.BambooGripKinematics;

namespace Motus.Viewer.Services;

/// <summary>
/// Motus string-identity tasks and motion keys for /bamboo-truss. Each member is a pick (single store feed) →
/// place (cane exactly on its truss edge) pair, built by <see cref="BambooTrussPlanner"/> from a grounded base:
/// the L-grip clamped on the anchor column at A. Once triangle 1 is closed (rigid, hinged at A) the arm walks on it with base swaps.
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

    // ---------------- walk task block ----------------

    /// <summary>
    /// A <c>walk</c> task block for /bamboo-truss: string identity "walk", <c>Object</c> = the target clamp object
    /// ("placed-baseR", "stake-S3"), one plane "target" at the clamp point on that cane (its parameter along the cane is
    /// read back from the plane position), and <c>Gait</c> = one of <see cref="BambooWalk.Gaits"/> (null = default).
    /// </summary>
    public static BambooIcdMotion.TaskEdit WalkTask(string target, double f = 0, string? gait = null)
    {
        BambooIcdCell.StrutPlacement at;
        if (BambooTrussLayout.IsSupport(target))
        {
            var st = BambooTrussLayout.StakeNamed(target);
            at = new BambooIcdCell.StrutPlacement(st.X, st.ClampY, st.Z, -90, 0);
        }
        else
        {
            var m = BambooTrussLayout.Members.First(x => "placed-" + x.Id == target).Place;
            var pos = BambooWalk.Pos(target, f);
            at = new BambooIcdCell.StrutPlacement(pos.X, pos.Y, pos.Z, m.PitchDeg, m.YawDeg);
        }
        return new BambooIcdMotion.TaskEdit
        {
            Identity = "walk", Object = target, JawMm = BambooTrussLayout.JawClosedMm, Gait = gait,
            Planes = [Plane("target", at, 0)]
        };
    }

    /// <summary>
    /// A targeted-landing <c>walk</c> block (gait "land"): <c>Object</c> = the member (or relay) id to land for; the
    /// "target" plane sits on that member's place pose. The landing itself (cane, point, roll) is chosen by the planner.
    /// </summary>
    public static BambooIcdMotion.TaskEdit LandTask(string memberId)
    {
        var m = BambooTrussLayout.Def(memberId) ?? throw new ArgumentException($"unknown member {memberId}");
        return new BambooIcdMotion.TaskEdit
        {
            Identity = "walk", Object = memberId, JawMm = BambooTrussLayout.JawClosedMm, Gait = LandGait,
            Planes = [Plane("target", m.Place, 0)]
        };
    }

    /// <summary>Target parameter of a walk block: its "target" plane projected on the cane, as a fraction of half − 3 cm.</summary>
    public static double WalkTargetF(BambooIcdMotion.TaskEdit t)
    {
        if (BambooTrussLayout.IsSupport(t.Object) || t.Gait == LandGait) return 0;
        var pl = t.Planes.FirstOrDefault(x => x.Name == "target");
        if (pl is null) return 0;
        var m = BambooTrussLayout.Members.First(x => "placed-" + x.Id == t.Object).Place; var u = m.Axis;
        return Math.Round(((pl.X - m.X) * u.X + (pl.Y0 + pl.YMm / 1000 - m.Y) * u.Y + (pl.Z - m.Z) * u.Z) / BambooWalk.Half * 4) / 4;
    }

    /// <summary>The walking benchmark at the shipped site: walk to S3, build the chord, then S3 → baseL B end → across B
    /// onto baseR → baseR D end → back to S3, all with <paramref name="gait"/>.</summary>
    public static List<BambooIcdMotion.TaskEdit> WalkBenchmarkTasks(string? gait = null)
    {
        var members = DefaultTasks(BambooTrussLayout.Members.Where(m => m.Id is "baseL" or "baseR").ToList());
        var tasks = new List<BambooIcdMotion.TaskEdit> { WalkTask(BambooTrussLayout.Current.WalkLTarget) };
        tasks.AddRange(members);
        tasks.Add(WalkTask("placed-baseL", -1, gait));
        tasks.Add(WalkTask("placed-baseR", -1, gait));
        tasks.Add(WalkTask("placed-baseR", 1, gait));
        tasks.Add(WalkTask(BambooTrussLayout.Current.WalkLTarget, 0, gait));
        return tasks;
    }

    /// <summary>
    /// The shipped /bamboo-truss task sequence (Warren truss over the pit): triangle 1 from the A column, a node-transfer
    /// walk onto it, the far members tried from there (reported when unreachable), then a walk back onto the A column.
    /// </summary>
    public static List<BambooIcdMotion.TaskEdit> ShippedTasks()
    {
        var site = BambooTrussLayout.Current;
        var byId = BambooTrussLayout.Members.ToDictionary(m => m.Id);
        List<BambooIcdMotion.TaskEdit> Member(string id) => DefaultTasks(new[] { byId[id] });
        // Triangle 1 from the A column (A–T1, T1–M, A–M); then each remaining member after a targeted-landing walk
        // (one node-transfer step to a cane point + roll from which the stock and the place are both Motus-reachable):
        // M–T2 and T1–T2 (closes triangle 2: T2 rigid), M–D; T2–D through one relay — its cane is parked in M's
        // connector (pointing out of the plane, +z), the robot walks onto the top chord and re-grasps it there.
        var tasks = new List<BambooIcdMotion.TaskEdit>();
        foreach (var id in new[] { "AT1", "T1M", "AM" }) tasks.AddRange(Member(id));
        foreach (var id in new[] { "MT2", "T1T2", "MD", ShippedRelay, "T2D" })
        {
            tasks.Add(LandTask(id));
            tasks.AddRange(DefaultTasks(new[] { BambooTrussLayout.Def(id)! }));
        }
        return tasks;
    }

    static string Signature(IEnumerable<BambooIcdMotion.TaskEdit> tasks) =>
        string.Join(";", tasks.Select(t => t.Identity == "walk" ? $"walk:{t.Object}:{FingerprintNumber(WalkTargetF(t))}:{t.Gait}" : $"{t.Identity}:{t.Object}"));

    /// <summary>True when <paramref name="tasks"/> is the shipped sequence (the build-time plan applies).</summary>
    public static bool IsShippedSequence(IReadOnlyList<BambooIcdMotion.TaskEdit> tasks) => Signature(tasks) == Signature(ShippedTasks());

    /// <summary>Per-walk-block outcome (for the benchmark table).</summary>
    public sealed record WalkLeg(string Target, string Gait, bool Ok, int Steps, int Tried, double StrideM, double Seconds,
        List<BambooWalk.Step> Path, BambooWalk.Result Search);

    /// <summary>
    /// Run a task sequence in order: <c>walk</c> blocks walk (stake → stake with the stake walk; otherwise
    /// <see cref="BambooWalk.Explore"/> with the block's gait), pick/place pairs build their member (only with one end
    /// fixed). Failures are noted and the sequence continues.
    /// </summary>
    public static TrussBuild RunSequence(IReadOnlyList<BambooIcdMotion.TaskEdit> tasks, Action<string>? log = null,
        List<WalkLeg>? legs = null, int maxStepsPerWalk = 150)
    {
        var cell = TryCell();
        if (cell is null) return new TrussBuild { Error = "Motus cell required (call UseCell with the viewer BambooIcdCell).", Tasks = tasks.ToList() };
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var site = BambooTrussLayout.Current;
        var p = new BambooTrussPlanner(cell);
        var notes = new List<string>();
        var built = new List<string>();
        if (!p.Start()) return new TrussBuild { Error = p.Error, Tasks = tasks.ToList() };
        BambooIcdMotion.TaskEdit? pick = null;
        foreach (var t in tasks)
        {
            if (t.Identity == "pick") { pick = t; continue; }
            if (t.Identity == "place" && pick is not null && pick.Object == t.Object)
            {
                pick = null;
                var m = BambooTrussLayout.Def(t.Object);
                if (m is null) { notes.Add($"{t.Object}: not a truss member — skipped"); continue; }
                var fx = BambooTrussLayout.ExistingNodes(built);
                if (!fx.Contains(m.NodeA) && !fx.Contains(m.NodeB)) { notes.Add($"{m.Id}: not built — neither end node exists yet"); continue; }
                if (p.BuildMember(m)) { if (!BambooTrussLayout.IsRelay(m.Id)) built.Add(m.Id); log?.Invoke($"{m.Id}: built"); }
                else { notes.Add($"{m.Id}: not built — {p.Error}"); log?.Invoke($"{m.Id}: {p.Error}"); }
                continue;
            }
            if (t.Identity != "walk") continue;
            if (t.Gait == LandGait)
            {
                notes.Add(Land(p, t.Object, log));
                continue;
            }
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var gait = BambooWalk.GaitNamed(t.Gait);
            var target = t.Object; var f = WalkTargetF(t);
            if (BambooTrussLayout.IsSupport(target) && BambooTrussLayout.IsSupport(p.LStake))
            {
                if (p.LStake == target) continue;
                var ok = p.Walk(site.WalkRAnchor, BambooTrussLayout.StakeNamed(target));
                notes.Add(ok ? $"walk → {target}: stake walk" : $"walk → {target}: not planned — {p.Error}");
                continue;
            }
            bool Goal(BambooWalk.Site s) => s.On == target && (s.IsStake || Math.Abs(s.F - f) < 1e-9);
            var label = BambooTrussLayout.IsSupport(target) ? target : $"{target["placed-".Length..]}@{f:+0.00;-0.00}";
            var res = BambooWalk.Explore(p, gait, site.WalkRAnchor, Goal, maxStepsPerWalk, log, cycleName: $"walk ({gait.Id}) → {label}");
            var path = res.Reached is { } r ? res.PathTo(r) : new List<BambooWalk.Step>();
            var leg = new WalkLeg($"{target}@{f:+0.00;-0.00}", gait.Id, res.Reached is not null, path.Count, res.Tried,
                path.Count == 0 ? 0 : path.Average(x => x.StrideM), sw.Elapsed.TotalSeconds, path, res);
            legs?.Add(leg);
            notes.Add(res.Reached is { } rr
                ? $"walk ({gait.Id}) → {rr}: {path.Count} steps — " + string.Join(" → ", path.Select(x => $"{x.Kind} (R on {x.RAnchor}) to {x.To}"))
                : $"walk ({gait.Id}) → {target}@{f:+0.00;-0.00}: not reachable with this gait ({res.Tried} steps tried, {res.Reachable.Count} sites reached)");
            log?.Invoke(notes[^1]);
        }
        p.StampTimes();
        cell.SetMountViewer(BambooTrussLayout.StartBase);
        var keys = p.Keys;
        var cycles = new List<CycleBound>();
        foreach (var (name, first, last) in p.CycleKeys)
        {
            if (last < first) continue;
            cycles.Add(new CycleBound { MemberId = name, FirstKey = first, LastKey = last, T0 = first > 0 ? keys[first - 1].T : 0, T1 = keys[last].T });
        }
        notes.InsertRange(0, p.Notes);
        notes.Add($"planned in {clock.Elapsed.TotalSeconds:F1} s ({p.Checks} Motus pose checks)");
        return new TrussBuild { Keys = keys, Cycles = cycles, Grips = p.Grips, Notes = notes, Built = built, MotusChecks = p.Checks, Tasks = tasks.ToList() };
    }

    /// <summary>The one relay of the shipped plan: T2–D's cane parked in M's connector, pointing +z.</summary>
    public static readonly string ShippedRelay = BambooTrussLayout.RelayId("T2D", "M", "z+");

    /// <summary>Walk mode "land": one node-transfer step to a landing (point + roll) chosen for the next member.</summary>
    public const string LandGait = "land";

    /// <summary>
    /// Targeted landing walk for member (or relay) <paramref name="id"/>: if it is not buildable from where L stands,
    /// rank IK landing candidates (<see cref="BambooTrussPlanner.LandingCandidates"/>) and take the first whose walk
    /// step AND trial build (stock pick → carry → place → R park, Motus-clear) succeed; only the walk is kept.
    /// </summary>
    static string Land(BambooTrussPlanner p, string id, Action<string>? log)
    {
        var m = BambooTrussLayout.Def(id);
        if (m is null) return $"walk (land) → {id}: unknown member";
        var cp = p.SaveDeep();
        if (p.BuildMember(m)) { p.RollbackDeep(cp); return $"walk (land) → {id}: not needed (buildable from {p.LStake})"; }
        p.RollbackDeep(cp);
        var cands = p.LandingCandidates(m);
        var tried = 0;
        foreach (var c in cands.GroupBy(c => (c.RCane, c.ROff, c.LCane, c.LOff)).Select(g => g.First()))
        {
            if (tried++ >= BambooTrussPlanner.LandTries) break;
            p.RollbackDeep(cp);
            if (!p.WalkStep(c.RCane, c.ROff, c.RFrame, c.LCane, c.LOff, c.LFrame, $"walk (land) → {id}")) continue;
            var cpw = p.SaveDeep();
            if (!p.BuildMember(m)) continue;
            p.RollbackDeep(cpw);
            var note = $"walk (land) → {id}: R on {c.RCane} at {c.ROff:+0.000;-0.000} m, L lands on {c.LCane} at {c.LOff:+0.000;-0.000} m ({cands.Count} IK candidates, {tried} tried)";
            log?.Invoke(note);
            return note;
        }
        p.RollbackDeep(cp);
        return $"walk (land) → {id}: no landing found ({cands.Count} IK candidates, {Math.Min(tried, BambooTrussPlanner.LandTries)} tried)";
    }

    public static TrussBuild Build(IReadOnlyList<BambooIcdMotion.TaskEdit>? tasks = null, bool useCache = false) =>
        BuildAsync(tasks, null, useCache).GetAwaiter().GetResult();

    // ---------------- build-time plan cache (WASM) ----------------

    /// <summary>Bump when planner behaviour changes (invalidates <see cref="DefaultPlanResource"/>).</summary>
    /// <summary>L side-grasp IK solutions (rolls about the clamped cane) tried per brachiation clamp point.</summary>
    public const int BrachiateLSolutions = 4;

    public const string PlannerVersion = "warren-pit-2-relay";
    public const string DefaultPlanResource = "Motus.Viewer.Services.BambooTrussDefaultPlan.json";

    /// <summary>Everything the planner's output depends on (layout, tolerances, version), invariant-formatted.</summary>
    public static string LayoutFingerprint
    {
        get
        {
            static string F(double v) => FingerprintNumber(v);
            string P(BambooIcdCell.StrutPlacement p) => $"{F(p.X)},{F(p.Y)},{F(p.Z)},{F(p.PitchDeg)},{F(p.YawDeg)}";
            string St(BambooTrussLayout.Stake t) => $"{t.Name},{F(t.X)},{F(t.Z)},{F(t.ClampY)},{F(t.Top)},{F(t.HeadingDeg)}";
            var parts = new List<string>
            {
                PlannerVersion, F(BambooTrussLayout.StrutLength), string.Join("/", BambooTrussLayout.Stakes.Select(St)), BambooTrussLayout.Current.StartStake,
                BambooTrussLayout.Current.WalkRAnchor, BambooTrussLayout.Current.WalkLTarget,
                string.Join("/", BambooTrussLayout.UnreachableMembers.Keys),
                P(BambooTrussLayout.Store), F(BambooTrussLayout.RackTop), F(K.GripPosTolM), F(K.GripAxisTolDeg),
                F(BambooTrussLayout.JawOpenMm), F(BambooTrussLayout.LJawOpenMm), F(BambooTrussLayout.JawClosedMm)
            };
            parts.AddRange(BambooTrussLayout.Members.Select(m => $"{m.Id}:{P(m.Place)}"));
            parts.AddRange(BambooTrussLayout.Pads.Select(p => $"{p.Name}:{F(p.X)},{F(p.Z)},{F(p.HalfX)},{F(p.HalfZ)}"));
            parts.AddRange(BambooTrussLayout.Posts.Select(p => $"{p.Name}:{F(p.X)},{F(p.Z)},{F(p.Top)},{F(p.Radius)}"));
            parts.AddRange(BambooTrussLayout.Racks.Select(r => $"{r.Name}:{F(r.X)},{F(r.Z)},{F(r.HalfX)},{F(r.HalfZ)},{F(r.Top)}"));
            parts.AddRange(BambooTrussLayout.PitBoxes.Select(b => $"{b.Name}:{F(b.X)},{F(b.Y)},{F(b.Z)},{F(b.HalfX)},{F(b.HalfY)},{F(b.HalfZ)}"));
            parts.Add($"hubs:{F(BambooTrussLayout.HubHalf)},gap:{F(BambooTrussLayout.NodeGap)}");
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

    /// <summary>
    /// A number as it enters <see cref="LayoutFingerprint"/>: rounded to 1e-6 (µm / µdeg), invariant culture, no
    /// negative zero. Derived values (member pitch/yaw from Atan2/Asin, stake tops from sums) differ in the last
    /// ulp between desktop .NET and Blazor WASM (−29.999999999999996 vs −29.999999999999993), so a round-trip
    /// ("R") fingerprint never matched in the browser and the page always planned live.
    /// </summary>
    public static string FingerprintNumber(double v)
    {
        var r = Math.Round(v, 6, MidpointRounding.AwayFromZero);
        if (r == 0) r = 0; // −0 → 0
        return r.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }

    /// <summary>Why the last <see cref="LoadCachedDefaultPlan"/> returned null (or "hit").</summary>
    public static string CacheStatus { get; private set; } = "not tried";

    /// <summary>The default plan shipped with the app, if it matches the current layout/planner (else null → plan live).</summary>
    public static TrussBuild? LoadCachedDefaultPlan()
    {
        try
        {
            using var stream = typeof(BambooTrussMotion).Assembly.GetManifestResourceStream(DefaultPlanResource);
            if (stream is null) return Miss("resource missing");
            var c = System.Text.Json.JsonSerializer.Deserialize<PlanCache>(stream);
            if (c is null) return Miss("deserialised null");
            if (c.Keys.Count == 0) return Miss("no keys");
            var fp = LayoutFingerprint;
            if (c.Fingerprint != fp)
            {
                var i = 0;
                while (i < Math.Min(fp.Length, c.Fingerprint.Length) && fp[i] == c.Fingerprint[i]) i++;
                return Miss($"fingerprint differs at {i}: shipped '…{Snip(c.Fingerprint, i)}' vs live '…{Snip(fp, i)}'");
            }
            var notes = c.Notes.ToList();
            notes.Add("plan: shipped build-time plan for the default tasks (byte-identical to live planning — enforced by a test); Play still Motus-scans it live");
            CacheStatus = "hit";
            return new TrussBuild { Keys = c.Keys, Cycles = c.Cycles, Grips = c.Grips, Notes = notes, Built = c.Built, MotusChecks = c.MotusChecks };
        }
        catch (Exception ex)
        {
            return Miss($"{ex.GetType().Name}: {ex.Message}");
        }

        static string Snip(string s, int i) => s.Substring(Math.Max(0, i - 20), Math.Min(60, s.Length - Math.Max(0, i - 20)));
        static TrussBuild? Miss(string why)
        {
            CacheStatus = "miss: " + why;
            Console.WriteLine("[bamboo-truss] shipped plan not used — " + why);
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
        tasks ??= ShippedTasks();
        if (tasks.Any(t => t.Identity == "walk"))
        {
            if (useCache && IsShippedSequence(tasks) && LoadCachedDefaultPlan() is { } shipped)
            {
                shipped.Tasks = tasks.ToList();
                return shipped;
            }
            if (useCache) Console.WriteLine("[bamboo-truss] shipped plan not used — tasks are not the shipped sequence");
            if (onMember is not null) await onMember("walk");
            return RunSequence(tasks);
        }
        var pairs = Pairs(tasks);
        if (pairs.Count == 0)
            return new TrussBuild { Error = "No pick/place pairs (object-matched).", Tasks = tasks.ToList() };
        if (useCache && !IsDefaultMemberOrder(tasks)) Console.WriteLine("[bamboo-truss] shipped plan not used — tasks are not the default member order");
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
        // Build every requested member that is reachable, in order. A member is only placed when at least one of its
        // ends is already fixed (A/D posts or a triangulated node), so no cane is ever left hanging free. When members
        // remain and the site has a second stake, the arm brachiates (R on a cane fixed at both ends, or a stake) to
        // it and tries the rest from there.
        var pending = new List<string>();
        var ids = pairs.Select(p => p.MemberId).Distinct().ToList();
        bool TryBuild(string memberId, bool lastChance)
        {
            var member = BambooTrussLayout.Members.FirstOrDefault(m => m.Id == memberId);
            if (member is null) { notes.Add($"{memberId}: not a truss member — skipped"); return true; }
            if (BambooTrussLayout.UnreachableMembers.TryGetValue(memberId, out var why))
            {
                notes.Add($"{memberId}: not built — {why}");
                return true;
            }
            var fixedNodes = BambooTrussLayout.FixedNodes(built);
            if (!fixedNodes.Contains(NodeName(member.EdgeA)) && !fixedNodes.Contains(NodeName(member.EdgeB)))
            {
                if (lastChance) notes.Add($"{memberId}: not built — neither end is fixed yet (it would hang free)");
                return false;
            }
            if (!planner.BuildMember(member))
            {
                if (lastChance) notes.Add($"{memberId}: not built — {planner.Error}");
                return false;
            }
            built.Add(memberId);
            return true;
        }
        foreach (var id in ids)
        {
            await Yield(id);
            if (!TryBuild(id, lastChance: false)) pending.Add(id);
        }
        var site = BambooTrussLayout.Current;
        var walked = false;
        await Yield("walk");
        if (planner.Walk(site.WalkRAnchor, BambooTrussLayout.WalkTarget)) walked = true;
        else notes.Add($"walk: not planned — {planner.Error}");
        foreach (var id in pending)
        {
            await Yield(id);
            TryBuild(id, lastChance: true);
        }
        if (!walked && pending.Count > 0) notes.Add("remaining members were tried from the start stake only (walk failed)");
        // Brachiation onto the built structure: once canes are fixed at both ends, L (open linear jaw) may clamp one
        // from the side and become the base, bringing the remaining members within reach.
        var remaining = pending.Where(id => !built.Contains(id) && !BambooTrussLayout.UnreachableMembers.ContainsKey(id)).ToList();
        if (remaining.Count > 0 && BambooTrussLayout.FixedBothEnds(built).Count > 0)
        {
            await Yield("brachiate");
            // Goal-driven: every (cane, R anchor, clamp point) is tried from the same checkpoint and judged by how many
            // of the remaining members can then be placed; the best option is replayed (the planner is deterministic).
            // PTP: every failing option would otherwise spend 3 × 1500 RRT iterations per transit; the search and its
            // replay use a cheaper RRT (same settings, so the replay reproduces the chosen option).
            planner.RrtAttempts = 1; planner.RrtIters = 400;
            var cp = planner.Save();
            var builtCp = built.ToList();
            var doubleFixed = BambooTrussLayout.FixedBothEnds(built).ToList();
            var options = (from target in doubleFixed
                           from anchor in new[] { site.WalkRAnchor }.Concat(doubleFixed.Where(x => x != target).Select(x => "placed-" + x))
                           from f in BambooTrussPlanner.BrachiateOffsets
                           from lSol in Enumerable.Range(0, BrachiateLSolutions)
                           select (target, anchor, f, lSol)).ToList();
            (string target, string anchor, double f, int lSol)? best = null;
            var bestCount = 0;
            var lastErr = "";
            var placedOptions = 0;
            foreach (var o in options)
            {
                planner.Rollback(cp);
                built.Clear(); built.AddRange(builtCp);
                if (!planner.Brachiate(o.anchor, o.target, o.f, o.lSol)) { lastErr = planner.Error; continue; }
                placedOptions++;
                var n = remaining.Count(id => TryBuild(id, lastChance: false));
                if (best is null || n > bestCount) { best = o; bestCount = n; }
                if (n == remaining.Count) break;
            }
            planner.Rollback(cp);
            built.Clear(); built.AddRange(builtCp);
            if (best is not { } pick)
                notes.Add("brachiate onto the chord: not planned — no Motus-clear side approach of the L-grip (open on +y) onto a " +
                          $"double-fixed cane from any R anchor / clamp point; last: {lastErr}");
            else
            {
                planner.Brachiate(pick.anchor, pick.target, pick.f, pick.lSol);
                foreach (var id in remaining)
                {
                    await Yield(id);
                    // Notes for these members were already written on the walk pass; replace them on success.
                    if (TryBuild(id, lastChance: false)) notes.RemoveAll(n => n.StartsWith(id + ": not built", StringComparison.Ordinal));
                    else
                    {
                        notes.RemoveAll(n => n.StartsWith(id + ": not built", StringComparison.Ordinal));
                        notes.Add($"{id}: not built — not reachable Motus-clear after brachiating onto {pick.target} ({placedOptions} clamp options tried; best placed {bestCount}/{remaining.Count}): {planner.Error}");
                    }
                }
            }
        }
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

    /// <summary>
    /// Full interpolated state at t (joints, both jaw openings, hold) plus the index of the key ending the segment.
    /// Key k's hold and label describe the segment that ends at k (the motion *to* k), so a block's label covers
    /// exactly its own motion — e.g. "L:Slide up" ends where the slide ends, the next transit is "L:Transit".
    /// </summary>
    public static string NodeName((double X, double Y, double Z) p) =>
        BambooTrussLayout.FootNodes.OrderBy(n => Math.Abs(n.X - p.X) + Math.Abs(n.Y - p.Y)).First().Name;

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
            b.Hold, b.Label, i + 1);
    }

    /// <summary>
    /// Cycle owning the segment that is playing at t: the cycle of the key that ends it (<see cref="State.SegEnd"/>).
    /// Key k's state (hold, base, clamps) applies to the segment ending at k, so world and hold always come from the
    /// same cycle — including the hand-over segment from one member's last key into the next member's first key.
    /// </summary>
    public static int CycleIndexAt(IReadOnlyList<BambooIcdMotion.MotionKey> keys, IReadOnlyList<CycleBound> cycles, double t)
    {
        if (cycles.Count == 0) return 0;
        var k = SampleState(keys, t).SegEnd;
        for (var i = 0; i < cycles.Count; i++)
            if (k >= cycles[i].FirstKey && k <= cycles[i].LastKey) return i;
        return k < cycles[0].FirstKey ? 0 : cycles.Count - 1;
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
