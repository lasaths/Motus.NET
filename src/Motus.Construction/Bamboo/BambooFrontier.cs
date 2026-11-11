using Motus.Construction.Search;
using static Motus.Construction.BambooBuildSearch;

namespace Motus.Construction;

/// <summary>
/// The build search's frontier candidates: for a build state (placed members + where the robot stands), every way to
/// build each frontier member (direct, landing, anchor walk, relay) and every walk-only reposition, each with its
/// reach precondition and its rank, in the order the search confirms them. Reach-map and IK only: no planner call, so
/// the order is testable in seconds. The opt-in ordering aids (<see cref="Options.NearestFirst"/>,
/// <see cref="Options.ConstrainedFirst"/>, the TAMP prior, <see cref="Options.Stability"/>) all act here.
/// </summary>
internal sealed class BambooFrontier
{
    readonly BambooStructureGraph graph;
    readonly Grid grid;
    readonly Options o;
    readonly IReadOnlyDictionary<string, List<string>> relayOptions;
    readonly Action<string> log;
    readonly TampSettings? tamp;
    readonly BambooFrameStability.Settings? stability;
    readonly BambooTrussLayout.Stake _start = BambooTrussLayout.StakeNamed(BambooTrussLayout.Current.StartStake);
    readonly Dictionary<(string, string), double> _gap = new();
    readonly Dictionary<string, int>? _stands;
    readonly Func<string, string, double>? _prior;

    // Rank offsets per mode (lower first): direct, then landing, anchor walk, relay, reposition.
    const double LandRank = 10, WalkRank = 15, RelayRank = 20, RepositionRank = 30;
    /// <summary>ConstrainedFirst: rank added per stand spot that reaches a member's stock and place (fewest spots first).</summary>
    const double RankPerStand = 1000;
    /// <summary>BraceFirst: rank taken off a candidate that makes placed members fixed (ahead of every other rank term).</summary>
    const double BraceRank = 1e9;

    public BambooFrontier(BambooStructureGraph graph, Grid grid, Options o, IReadOnlyDictionary<string, List<string>> relayOptions,
        string layoutId, Action<string> log)
    {
        (this.graph, this.grid, this.o, this.relayOptions, this.log) = (graph, grid, o, relayOptions, log);
        tamp = o.Tamp; stability = o.Stability;
        if (o.ConstrainedFirst)
        {
            var finalFixed = Footholds(graph, graph.Edges.Select(e => e.Id).ToList(), o);
            _stands = graph.Edges.ToDictionary(e => e.Id, e =>
            {
                var m = BambooTrussLayout.Def(e.Id)!; var stock = BambooReachMap.Stock(m).Name;
                return Enumerable.Range(0, grid.Sites.Count).Count(s => grid.Owner[s] != e.Id
                    && (BambooTrussLayout.IsSupport(grid.Owner[s]) || finalFixed.Contains(grid.Owner[s])) && grid.Ok(s, stock) && grid.Ok(s, e.Id));
            });
            log("constrained first: stand spots reaching stock + place — " + string.Join(", ", _stands.OrderBy(x => x.Value).Select(x => $"{x.Key} {x.Value}")));
        }
        _prior = tamp?.PriorLogPaths is { } logs && tamp.PriorWeight > 0 ? OutcomeLog.Prior(OutcomeLog.Read(logs), layoutId) : null;
        if (_prior is not null) log("tamp: candidates ordered with the confirm-rate prior");
    }

    /// <summary>
    /// Candidates for the build state, in rank order before resume / skips (ties keep generation order). Eager
    /// preconditions (default) are evaluated here and failing candidates dropped; lazy ones (<see cref="TampSettings.LazyPreconditions"/>)
    /// are memoised and evaluated when called. <paramref name="repositionHere"/>: walk-only repositions may be offered.
    /// </summary>
    public List<(string Id, Mode Mode, string? Relay, double Rank, Func<bool> Pre)> Candidates(List<string> placed,
        BambooReachMap.ClampSite stand, string lStake, bool repositionHere)
    {
        var fixedIds = BambooBuildSearch.Footholds(graph, placed, o);
        // Each candidate carries its reach precondition; eager (default) evaluates it here, lazy when it is next.
        var options = new List<(string Id, Mode Mode, string? Relay, double Rank, Func<bool> Pre)>();
        var map = new BambooReachMap { IkSeeds = 12 };
        foreach (var id in graph.Frontier(placed))
        {
            var m = BambooTrussLayout.Def(id)!;
            var usable = Enumerable.Range(0, grid.Sites.Count)
                .Where(s => grid.Owner[s] != id && (BambooTrussLayout.IsSupport(grid.Owner[s]) || fixedIds.Contains(grid.Owner[s]))).ToList();
            // Far members first: what is built later must not block reaching what is built earlier. NearestFirst: members
            // whose place and stock lie nearest the stand first, so the robot finishes an area before it moves on.
            var far = o.NearestFirst ? -(Dist(stand.Origin, (m.Place.X, m.Place.Y, m.Place.Z)) + Dist(stand.Origin, (m.Store.X, m.Store.Y, m.Store.Z)))
                : Dist(m.Place, _start);
            var stock = BambooReachMap.Stock(m);
            options.Add((id, Mode.Direct, null, -far,
                () => map.Query(stand, stock).Ik && map.Query(stand, BambooReachMap.PlaceOf(m)).Ik));
            // A landing puts L on a fixed member one node-transfer step away (BambooTrussPlanner.LandingCandidates):
            // only member sites within two arm reaches of the stand can be landed on.
            var landable = usable.Where(s => !BambooTrussLayout.IsSupport(grid.Owner[s]) && Near(stand, grid.Sites[s])).ToList();
            options.Add((id, Mode.Land, null, LandRank - far, () => landable.Any(s => grid.Ok(s, stock.Name) && grid.Ok(s, id))));
            if (o.AnchorWalks && BambooTrussLayout.IsSupport(lStake))
                foreach (var a in BambooTrussLayout.Stakes.Where(a => a.Name != lStake))
                    options.Add((id, Mode.Walk, a.Name, WalkRank - far + Dist(m.Place, a),
                        // The walk is at most two steps (R on a via point, then L on the anchor): within two arm reaches.
                        () => Enumerable.Range(0, grid.Sites.Count).Any(s => grid.Owner[s] == a.Name && Near(stand, grid.Sites[s]) && grid.Ok(s, stock.Name) && grid.Ok(s, id))));
            foreach (var r in relayOptions.GetValueOrDefault(id) ?? new List<string>())
            {
                if (!graph.ExistingNodes(placed).Contains(BambooTrussLayout.RelayNode(r))) continue;
                // Stock → park from where the robot stands or one landing; park → place from one landing.
                options.Add((id, Mode.Relay, r, RelayRank - far, () =>
                    ((map.Query(stand, stock).Ik && map.Query(stand, BambooReachMap.PlaceOf(BambooTrussLayout.Def(r)!)).Ik)
                        || usable.Any(s => grid.Ok(s, stock.Name) && grid.Ok(s, r)))
                    && usable.Any(s => grid.Ok(s, r) && grid.Ok(s, id))));
            }
        }
        if (o.Reposition && repositionHere && o.MaxRepositions > 0)
            foreach (var (to, f, travel) in BambooBuildSearch.RepositionTargets(graph, placed, fixedIds, stand, grid, map, _gap).Take(o.MaxRepositions))
                options.Add((to, Mode.Reposition, f.ToString("R", System.Globalization.CultureInfo.InvariantCulture), RepositionRank + travel, () => true));
        if (tamp?.LazyPreconditions != true)
            options = options.Where(x => x.Pre()).Select(x => x with { Pre = () => true }).ToList();
        else
            options = options.Select(x => x with { Pre = Precondition.Once(x.Pre) }).ToList();
        if (_prior is not null)
            options = options.Select(x => x with { Rank = x.Rank + tamp!.PriorWeight * _prior(x.Id, x.Mode.ToString()) }).ToList();
        if (_stands is not null)
            options = options.Select(x => x with { Rank = x.Rank + RankPerStand * _stands.GetValueOrDefault(x.Id) }).ToList();
        if (o.BraceFirst)
        {
            var before = graph.Fixed(placed).Count;
            var braces = options.Where(x => x.Mode != Mode.Reposition).Select(x => x.Id).Distinct()
                .Where(id => graph.Fixed(placed.Append(id)).Count(f => f != id) > before).ToHashSet(StringComparer.Ordinal);
            if (braces.Count > 0) log("  brace first: " + string.Join(", ", braces));
            options = options.Select(x => x.Mode != Mode.Reposition && braces.Contains(x.Id) ? x with { Rank = x.Rank - BraceRank } : x).ToList();
        }
        if (stability is not null)
        {
            var sag = new Dictionary<string, BambooFrameStability.Report>(StringComparer.Ordinal);
            foreach (var id in options.Select(x => x.Id).Distinct())
            {
                var r = sag[id] = BambooFrameStability.Check(graph, placed.Append(id), stability);
                if (!r.Ok) log($"  {id}: dropped by the stability check — {r.Note}");
            }
            options = options.Where(x => sag[x.Id].Ok)
                .Select(x => x with { Rank = x.Rank + stability.RankPerMetre * sag[x.Id].MaxDeflectionM }).ToList();
        }
        return options;
    }
}
