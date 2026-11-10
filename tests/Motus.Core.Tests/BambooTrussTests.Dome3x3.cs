using System.Text.Json;
using Motus.Construction;

namespace Motus.Core.Tests;

public partial class BambooTrussTests
{
    static string Dome3x3Path => Path.GetFullPath(Path.Combine(
        AppContext.BaseDirectory, "..", "..", "..", "..", "..", "tests", "fixtures", "gridshell_dome3x3", "dome3x3.site.json"));

    static BambooTrussLayout.Site LoadDome3x3() =>
        JsonSerializer.Deserialize<BambooTrussLayout.Site>(File.ReadAllText(Dome3x3Path))!;

    /// <summary>
    /// The 3×3 dome example is data only: a <see cref="BambooTrussLayout.Site"/> in JSON with 16 nodes, 24 members (each
    /// its own cane length), 12 anchors and a stock spot for every member except the two at the start corner P00.
    /// </summary>
    [Fact]
    public void Dome3x3_Example_LoadsAsSiteData()
    {
        var site = LoadDome3x3();
        var st = site.Structure!;
        Assert.Equal(16, st.Nodes.Count);
        Assert.Equal(24, st.Members.Count);
        Assert.Equal(12, site.Stakes.Count);
        Assert.Equal("anchor-P00", site.StartStake);
        Assert.All(st.Members, m => Assert.InRange(m.Length!.Value, 0.24, 0.257));
        Assert.Equal(new[] { "P00P01", "P00P10" }, st.Members.Where(m => m.Feed is null).Select(m => m.Id).Order());
        Assert.All(st.Members, m => Assert.Contains(st.Nodes, n => n.Name == m.NodeA));
        Assert.All(st.Members, m => Assert.Contains(st.Nodes, n => n.Name == m.NodeB));
    }

    /// <summary>
    /// The 3×3 dome builds 24/24 with <see cref="BambooBuildSearch.Options.LandPrecheck"/> and
    /// <see cref="BambooBuildSearch.Options.ConstrainedFirst"/> (anchor walks on, 250 rejections allowed), a fresh planner
    /// rebuilds the same order, and the 2 mm scan is clear. About 40–50 min on an 8-core box, so it runs only when
    /// MOTUS_DOME_BUILD=1.
    /// </summary>
    [Fact]
    [Trait("Category", "Slow")]
    public void Dome3x3_Example_BuildsComplete_ReplaysAndIsClear()
    {
        if (Environment.GetEnvironmentVariable("MOTUS_DOME_BUILD") != "1") return;
        var old = BambooTrussLayout.Current;
        try
        {
            BambooTrussLayout.Use(LoadDome3x3());
            var cell = BambooIcdCell.Load(File.ReadAllText(UrdfPath), BambooTrussLayout.StrutLength, RobotDefinition.BambooIcd);
            BambooTrussMotion.UseCell(cell);
            _out.WriteLine(BambooSceneCheck.Run(null, BambooTrussLayout.Members).Summary); // as the scratch run did, before the search
            var r = BambooBuildSearch.Run(cell, new BambooBuildSearch.Options
            {
                AnchorWalks = true, LandPrecheck = true, ConstrainedFirst = true, MaxRejected = 250
            }, s => _out.WriteLine(s));
            _out.WriteLine($"placed {r.Order.Count}/24, {r.Confirmed} confirmed, {r.Rejected} rejected, {r.Seconds:F0} s, stop '{r.StopReason}'");
            Assert.True(r.Complete, r.StopReason);
            var b = BambooTrussMotion.RunSequence(r.Tasks);
            Assert.True(string.IsNullOrEmpty(b.Error), b.Error);
            Assert.Equal(r.Order.Select(c => c.MemberId), b.Built);
            var scan = new BambooTrussScan(cell, b.Keys, b.Cycles, BambooTrussLayout.Members.ToList(), BambooTrussLayout.JawClosedMm).RunToEnd();
            Assert.True(scan.FirstHit is null, scan.FirstHit?.Describe());
        }
        finally { BambooTrussLayout.Use(old); BambooTrussMotion.UseCell(Cell); }
    }
}
