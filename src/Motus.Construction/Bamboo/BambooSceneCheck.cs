namespace Motus.Construction;

/// <summary>
/// Fail-fast scene check, before any planning: for every member there must be a clamp site whose free gripper reaches
/// both the stock and the member's place line by IK, or a relay chain (stock → parked pose in a node connector → … →
/// place) whose every hop has such a site. Optimistic about the build order (every anchor and every other member counts
/// as a possible site), so a failure is a hard "cannot be built from this layout" with numbers; a pass is necessary,
/// not sufficient (collisions along the path and the order are the planner's job).
/// </summary>
public static class BambooSceneCheck
{
    public sealed record MemberResult(string MemberId, int Sites, int SitesStock, int SitesPlace, int SitesBoth,
        IReadOnlyList<string> Chain, IReadOnlyList<string> RelayOptions, string Verdict)
    {
        public bool Reachable => SitesBoth > 0 || Chain.Count > 0;
        public bool NeedsRelay => SitesBoth == 0 && Chain.Count > 0;
    }

    public sealed record Report(IReadOnlyList<MemberResult> Members, int Sites, int Lines, double Seconds)
    {
        public bool Ok => Members.All(m => m.Reachable);
        public string Summary =>
            $"scene check: {Members.Count(m => m.Reachable)}/{Members.Count} members reachable ({Members.Count(m => m.NeedsRelay)} need a relay; " +
            $"{Sites} clamp sites × {Lines} target lines)";
        public IEnumerable<string> Details => Members.Select(m => $"{m.MemberId}: {m.Verdict}");
    }

    public sealed record Options
    {
        /// <summary>Clamp offsets along a cane (fraction of half length − 3 cm).</summary>
        public IReadOnlyList<double> CaneOffsets { get; init; } = new[] { -1.0, -0.5, 0.0, 0.5, 1.0 };
        /// <summary>Roll angles of a cane clamp (degrees).</summary>
        public IReadOnlyList<double> Rolls { get; init; } = Enumerable.Range(0, 8).Select(i => i * 45.0).ToArray();
        /// <summary>Heading step of anchor-column clamps (degrees).</summary>
        public double AnchorHeadingStepDeg { get; init; } = 30;
        /// <summary>Connector park directions tried for relays (see <see cref="BambooTrussLayout.RelayId"/>).</summary>
        public IReadOnlyList<string> RelayDirs { get; init; } = new[] { "z+", "z-", "90", "270" };
        /// <summary>Longest relay chain searched (number of parks).</summary>
        public int MaxRelays { get; init; } = 2;
        /// <summary>Collision-check the IK poses against the design scene.</summary>
        public bool Collision { get; init; }
    }

    public static Report Run(BambooIcdCell? cell = null, IEnumerable<BambooTrussLayout.MemberDef>? members = null, Options? options = null)
    {
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var o = options ?? new Options();
        var list = (members ?? BambooTrussLayout.Members).ToList();
        var map = new BambooReachMap(cell) { Collision = o.Collision && cell is not null };

        // Sites: every anchor column, and every member (optimistic: any of them may be fixed by then).
        var siteOwner = new List<string>();
        var sites = new List<BambooReachMap.ClampSite>();
        foreach (var a in BambooTrussLayout.Stakes)
            foreach (var s in BambooReachMap.AnchorSites(a.Name, o.AnchorHeadingStepDeg)) { sites.Add(s); siteOwner.Add(a.Name); }
        foreach (var m in BambooTrussLayout.Members)
            foreach (var s in BambooReachMap.CaneSites("placed-" + m.Id, m.Place, o.CaneOffsets.Select(f => f * (m.Length / 2 - 0.03)), o.Rolls)) { sites.Add(s); siteOwner.Add(m.Id); }

        // Lines: stock, each member's place, and the parked poses relays may use.
        var lines = new List<BambooReachMap.TargetLine> { BambooReachMap.Stock() };
        lines.AddRange(list.Select(BambooReachMap.PlaceOf));
        var relayLines = new List<(string Node, string Dir, int Line)>();
        foreach (var n in BambooTrussLayout.FootNodes)
            foreach (var d in o.RelayDirs)
            {
                var def = BambooTrussLayout.Def(BambooTrussLayout.RelayId("X", n.Name, d))!;
                relayLines.Add((n.Name, d, lines.Count));
                lines.Add(new BambooReachMap.TargetLine($"park {n.Name}/{d}", def.Place, BambooTrussLayout.StockGraspOffsets));
            }
        // Stock + place columns first; the relay columns only if some member has no direct site.
        var direct = map.Matrix(sites, lines.Take(1 + list.Count).ToList());
        var anyMissing = Enumerable.Range(0, list.Count).Any(mi =>
            !Enumerable.Range(0, sites.Count).Any(si => siteOwner[si] != list[mi].Id && direct[si, 0].Ik && direct[si, 1 + mi].Ik));
        var relay = anyMissing ? map.Matrix(sites, lines.Skip(1 + list.Count).ToList()) : null;
        if (!anyMissing) lines.RemoveRange(1 + list.Count, lines.Count - 1 - list.Count);
        bool R(int site, int line)
        {
            var r = line <= list.Count ? direct[site, line] : relay![site, line - 1 - list.Count];
            return r.Ik && r.Clear;
        }

        var results = new List<MemberResult>();
        for (var mi = 0; mi < list.Count; mi++)
        {
            var m = list[mi];
            var place = 1 + mi;
            // A member is not a site for its own placement.
            var usable = Enumerable.Range(0, sites.Count).Where(s => siteOwner[s] != m.Id).ToList();
            var nStock = usable.Count(s => R(s, 0));
            var nPlace = usable.Count(s => R(s, place));
            var nBoth = usable.Count(s => R(s, 0) && R(s, place));
            var chain = new List<string>();
            var options1 = new List<string>();
            if (nBoth == 0 && relay is not null)
            {
                // BFS over target lines: hop a → b when one usable site reaches both. Parks must not sit on the member.
                bool Hop(int a, int b) => usable.Any(s => R(s, a) && R(s, b));
                var nodes = new List<int> { 0 };
                nodes.AddRange(relayLines.Select(r => r.Line));
                var prev = new Dictionary<int, int> { [0] = -1 };
                var frontier = new List<int> { 0 };
                for (var depth = 0; depth <= o.MaxRelays && !prev.ContainsKey(place); depth++)
                {
                    var next = new List<int>();
                    foreach (var a in frontier)
                    {
                        if (Hop(a, place) && !prev.ContainsKey(place)) { prev[place] = a; break; }
                        if (depth == o.MaxRelays) continue;
                        foreach (var b in nodes.Where(b => !prev.ContainsKey(b)))
                            if (Hop(a, b)) { prev[b] = a; next.Add(b); }
                    }
                    frontier = next;
                }
                if (prev.ContainsKey(place))
                    for (var x = prev[place]; x > 0; x = prev[x]) chain.Insert(0, lines[x].Name["park ".Length..]);
                // Every single-park relay that works (stock → park and park → place each have a site).
                foreach (var r in relayLines)
                    if (Hop(0, r.Line) && Hop(r.Line, place)) options1.Add($"{r.Node}/{r.Dir}");
            }
            var verdict = nBoth > 0 ? $"direct — {nBoth} sites reach stock + place (stock {nStock}, place {nPlace} of {usable.Count})"
                : chain.Count > 0 ? $"relay via {string.Join(" → ", chain)} — no single site reaches both (stock {nStock}, place {nPlace} of {usable.Count}); single-park options: {(options1.Count > 0 ? string.Join(", ", options1) : "none")}"
                : $"UNREACHABLE — stock from {nStock}, place from {nPlace} of {usable.Count} sites, none both; no relay chain ≤ {o.MaxRelays} parks";
            results.Add(new MemberResult(m.Id, usable.Count, nStock, nPlace, nBoth, chain, options1, verdict));
        }
        return new Report(results, sites.Count, lines.Count, clock.Elapsed.TotalSeconds);
    }
}
