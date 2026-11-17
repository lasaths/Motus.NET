using Motus.Construction;

namespace Motus.Core.Tests;

public partial class BambooTrussTests
{
    /// <summary>Staged confirmation and walk-only reposition are opt-in; their settings are validated.</summary>
    [Fact]
    public void BuildSearch_StagedConfirmAndReposition_AreOptIn()
    {
        var o = new BambooBuildSearch.Options();
        Assert.False(o.StagedConfirm);
        Assert.False(o.Reposition);
        Assert.Equal(150, o.StagedRrtIters);
        Assert.Throws<ArgumentOutOfRangeException>(() => BambooBuildSearch.Run(Cell, o with { StagedConfirm = true, StagedRrtIters = 0 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => BambooBuildSearch.Run(Cell, o with { Reposition = true, MaxRepositions = -1 }));
    }

    /// <summary>
    /// The short RRT stage: when no transit runs out it plans exactly the motion the full budget plans (same random
    /// stream, same first attempt); when a transit runs out it is counted ("not yet"), and the full budget still plans
    /// the member — a short timeout is not a proof that there is no path.
    /// </summary>
    [Fact]
    public void Planner_ShortRrtStage_SameMotionWhenItPasses_NotYetWhenItRunsOut()
    {
        BambooTrussMotion.UseCell(Cell);
        var p = new BambooTrussPlanner(Cell);
        Assert.True(p.Start(), p.Error);
        string Keys() => System.Text.Json.JsonSerializer.Serialize(p.Keys);
        void Short(int iters) { p.RrtAttempts = 1; p.RrtIters = iters; p.StagedIters = iters; }
        void Full() { p.RrtAttempts = 3; p.RrtIters = 1500; p.StagedIters = 0; }

        var cp = p.SaveDeep();
        Short(150);
        var miss0 = p.StagedMisses;
        Assert.True(p.BuildMember(BambooTrussLayout.Def("AT1")!), p.Error);
        Assert.Equal(miss0, p.StagedMisses);
        var shortKeys = Keys();
        p.RollbackDeep(cp);
        Full();
        Assert.True(p.BuildMember(BambooTrussLayout.Def("AT1")!), p.Error);
        Assert.Equal(shortKeys, Keys());

        var cp2 = p.SaveDeep();
        Short(1);
        miss0 = p.StagedMisses;
        p.BuildMember(BambooTrussLayout.Def("T1M")!);
        Assert.True(p.StagedMisses > miss0, "one RRT iteration should run out on T1M");
        p.RollbackDeep(cp2);
        Full();
        Assert.True(p.BuildMember(BambooTrussLayout.Def("T1M")!), p.Error);
    }

    /// <summary>
    /// Staged confirmation on Warren with a one-iteration short stage (every RRT transit runs out): a run-out is "not
    /// yet", never a rejection, and is not re-run at the full budget in the same confirmation; the full budget runs
    /// only on the deepest branch (the current best prefix), and "not yet" candidates off it are left for later.
    /// </summary>
    [Fact]
    public void BuildSearch_StagedConfirm_NotYetIsNotARejection_FullBudgetOnlyOnTheDeepestBranch()
    {
        BambooTrussMotion.UseCell(Cell);
        var r = BambooBuildSearch.Run(Cell, new BambooBuildSearch.Options { StagedConfirm = true, StagedRrtIters = 1, MaxConfirmations = 3 });
        _out.WriteLine(string.Join("\n", r.Log));
        Assert.Equal(3, r.Confirmed);
        Assert.Equal(0, r.Rejected);
        Assert.True(r.Deferred > 0);

        int depth = 0, best = 0, retries = 0, parked = 0;
        string? lastNotYet = null;
        foreach (var line in r.Log)
        {
            if (System.Text.RegularExpressions.Regex.Match(line, @"^(\d+)/7 ") is { Success: true } m) { depth = int.Parse(m.Groups[1].Value); best = Math.Max(best, depth); }
            else if (line.StartsWith("backtrack:")) depth--;
            else if (line.Contains(": retry at the full RRT budget"))
            {
                retries++;
                Assert.Equal(best, depth);
                Assert.NotEqual(lastNotYet, line.Split(':')[0]); // not the very next step after its own short miss
            }
            else if (line.Contains("off the deepest branch")) { parked++; Assert.True(depth < best, line); }
            lastNotYet = line.Contains(": not yet — ") ? line.Split(':')[0] : null;
        }
        Assert.True(retries > 0);
        Assert.True(parked > 0);
        Assert.True(retries < r.Deferred, "the full budget must not run on every deferred candidate");
    }

    /// <summary>
    /// Reposition targets come from the foothold graph (only anchors before anything is fixed), never the stand itself,
    /// nearest route first; a reposition is replayed as a walk block in front of the next member's tasks.
    /// </summary>
    [Fact]
    public void BuildSearch_Reposition_TargetsAndWalkBlock()
    {
        var graph = BambooStructureGraph.FromLayout();
        var grid = BambooBuildSearch.BuildGrid(new BambooBuildSearch.Options(), Array.Empty<string>(), lazy: true);
        var start = BambooTrussLayout.Current.StartStake;
        var stand = BambooReachMap.AnchorSites(start)[0];
        var targets = BambooBuildSearch.RepositionTargets(graph, new List<string>(), new HashSet<string>(), stand, grid,
            new BambooReachMap { IkSeeds = 12 }, new()).ToList();
        Assert.DoesNotContain(targets, t => t.To == start);
        Assert.All(targets, t => Assert.True(BambooTrussLayout.IsSupport(t.To)));
        Assert.Equal(targets.Select(t => t.Travel).Order(), targets.Select(t => t.Travel));

        var c = new BambooBuildSearch.Choice("T1M", BambooBuildSearch.Mode.Direct, null, false, false, "") { RepositionTo = "AM", RepositionF = 1 };
        var with = BambooBuildSearch.TasksFor(c).ToList();
        var without = BambooBuildSearch.TasksFor(c with { RepositionTo = null }).ToList();
        Assert.Equal(without.Count + 1, with.Count);
        Assert.Equal("walk", with[0].Identity);
        Assert.Equal("placed-AM", with[0].Object);
        Assert.Equal(1, BambooTrussMotion.WalkTargetF(with[0]));
    }

    /// <summary>A doomed reposition walk (the target is never placed) stops at the search's clock cap, well under the
    /// 90 s replay walk, and leaves the planner where it stood. On Warren the walk graph is exhausted after 24 steps
    /// (~8 s), so the cap is lowered to show that it, not the graph, ends the walk.</summary>
    [Fact]
    public void BuildSearch_DoomedRepositionWalk_StopsAtTheCap()
    {
        Assert.True(BambooBuildSearch.RepositionWalkSeconds < 90);
        BambooTrussMotion.UseCell(Cell);
        var p = new BambooTrussPlanner(Cell, MotionConstraints.Current with { FixedMemberRule = false });
        Assert.True(p.Start(), p.Error);
        foreach (var id in new[] { "AM", "AT1" }) Assert.True(p.BuildMember(BambooTrussLayout.Def(id)!), p.Error);
        var keys = p.Keys.Count;
        var log = new List<string>();
        var cap = BambooBuildSearch.RepositionWalkSeconds;
        BambooBuildSearch.RepositionWalkSeconds = 1;
        try
        {
            Assert.Null(BambooBuildSearch.Reposition(p, "nowhere", 1, log.Add));
        }
        finally { BambooBuildSearch.RepositionWalkSeconds = cap; }
        _out.WriteLine(string.Join("; ", log));
        var tried = int.Parse(System.Text.RegularExpressions.Regex.Match(log.Single(), @"\((\d+) steps tried\)").Groups[1].Value);
        Assert.InRange(tried, 1, 23);
        Assert.Equal(keys, p.Keys.Count);
    }
}
