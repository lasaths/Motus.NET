using Motus.Construction;

namespace Motus.Core.Tests;

public partial class BambooTrussTests
{
    /// <summary>PlacedFootholds is opt-in: off keeps the rigidity rule, on lets a placed, still flexible member be a foothold.</summary>
    [Fact]
    public void BuildSearch_PlacedFootholds_OptIn_AllowsPlacedNonRigidMembers()
    {
        var graph = BambooStructureGraph.FromLayout();
        var placed = new[] { "AM" };
        Assert.Empty(graph.Fixed(placed)); // one bar on an anchor: not rigid
        var off = new BambooBuildSearch.Options();
        Assert.False(off.PlacedFootholds);
        Assert.Empty(BambooBuildSearch.Footholds(graph, placed, off));
        Assert.Equal(placed, BambooBuildSearch.Footholds(graph, placed, off with { PlacedFootholds = true }));

        BambooTrussMotion.UseCell(Cell);
        // AssemblyPlanner runs (and replays) with the fixed-member rule off for that call only, then restores it.
        var before = MotionConstraints.Current;
        var plan = AssemblyPlanner.Plan(new AssemblyPlanRequest
        {
            Search = new BambooBuildSearch.Options { PlacedFootholds = true, MaxConfirmations = 0 }
        });
        Assert.False(plan.CollisionFree);
        Assert.Same(before, MotionConstraints.Current);
        Assert.True(MotionConstraints.Current.FixedMemberRule);
    }
}
