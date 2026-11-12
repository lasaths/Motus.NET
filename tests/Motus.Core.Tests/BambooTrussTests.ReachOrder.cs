using Motus.Construction;

namespace Motus.Core.Tests;

public partial class BambooTrussTests
{
    /// <summary>Landing precheck and most-constrained-first ordering are opt-in.</summary>
    [Fact]
    public void BuildSearch_LandPrecheckAndConstrainedFirst_AreOptIn()
    {
        var o = new BambooBuildSearch.Options();
        Assert.False(o.LandPrecheck);
        Assert.False(o.ConstrainedFirst);
        Assert.False(o.RelayPrecheck);
    }

    /// <summary>
    /// With both on, the Warren search still finds a complete 7/7 sequence that a fresh planner rebuilds, and the
    /// precheck turns plain landings with no one-step IK candidate into skips (none is rejected with "0 IK candidates";
    /// relay re-grasps are not prechecked, and constrained-first tries the relay member T2D early, so the rejection count
    /// is not lower than the default search's).
    /// </summary>
    [Fact]
    public void BuildSearch_LandPrecheckAndConstrainedFirst_Warren_CompleteAndReplays()
    {
        BambooTrussMotion.UseCell(Cell);
        var r = BambooBuildSearch.Run(Cell, new BambooBuildSearch.Options { LandPrecheck = true, ConstrainedFirst = true }, s => _out.WriteLine(s));
        Assert.True(r.Complete, string.Join("\n", r.Log));
        Assert.DoesNotContain(r.Log, l => l.Contains(" land: rejected") && l.Contains("(0 IK candidates"));
        Assert.Contains(r.Log, l => l.Contains("constrained first: "));
        var b = BambooTrussMotion.RunSequence(r.Tasks);
        Assert.True(string.IsNullOrEmpty(b.Error), b.Error);
        Assert.All(r.Order, c => Assert.Contains(c.MemberId, b.Built));
        _out.WriteLine($"order {string.Join(" → ", r.Order.Select(c => c.MemberId))}; {r.Confirmed} confirmed, {r.Rejected} rejected, {r.Seconds:F0} s");
    }

    /// <summary>
    /// Relay precheck: at the start nothing is placed, so there is no one-step landing, and the T2D relay (its place is
    /// out of reach from anchor A) is skipped with a reason instead of going to the planner.
    /// </summary>
    [Fact]
    public void RelayPrecheck_NoOneStepLanding_SkipsTheRelay()
    {
        BambooTrussMotion.UseCell(Cell);
        var p = new BambooTrussPlanner(Cell);
        Assert.True(p.Start(), p.Error);
        var why = BambooBuildSearch.RelaySkip(p, "T2D", BambooTrussLayout.RelayId("T2D", "A", "z+"));
        _out.WriteLine(why ?? "(not skipped)");
        Assert.NotNull(why);
        Assert.Contains("no one-step landing reaches", why);
    }
}
