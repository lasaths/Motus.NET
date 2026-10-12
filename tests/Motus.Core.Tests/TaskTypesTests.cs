using Motus.Core;

namespace Motus.Core.Tests;

public class TaskTypesTests
{
    [Fact]
    public void Factories_AndTypedViews_RoundTrip()
    {
        var f = new Frame(0.1, 0.2, 0.3);
        var pick = CoreTasks.Pick("beam", 14, new Dictionary<string, Frame> { ["grasp"] = f });
        Assert.Equal(TaskIdentities.Pick, pick.Identity);
        var pv = CoreTasks.AsPick(pick)!;
        Assert.Equal("beam", pv.Object);
        Assert.Equal(14, pv.JawMm);
        Assert.Equal(f, pv.Frames["grasp"]);
        Assert.Null(CoreTasks.AsPlace(pick));

        var walk = CoreTasks.Walk("beam", WalkMode.Land);
        var wv = CoreTasks.AsWalk(walk)!;
        Assert.Equal(WalkMode.Land, wv.Mode);
        Assert.Null(wv.Gait);
        Assert.Equal("node", CoreTasks.Walk("post", gait: "inchworm").Metadata[TaskMetadataKeys.Mode]);
        Assert.Equal("inchworm", CoreTasks.AsWalk(CoreTasks.Walk("post", gait: "inchworm"))!.Gait);

        var park = CoreTasks.AsPark(CoreTasks.Park("beam", "N3", "up", 40))!;
        Assert.Equal(("beam", "N3", "up", 40.0), (park.Object, park.Connector, park.Direction, park.JawMm));
        Assert.Equal(new[] { "pick", "place", "walk", "park" }, TaskIdentities.All);
    }

    [Fact]
    public void SequenceRules_AcceptPickPlace_WalkEmptyHanded_AndARelay()
    {
        var ok = new[]
        {
            CoreTasks.Pick("a", 14), CoreTasks.Place("a", 40),
            CoreTasks.Walk("a", WalkMode.Land),
            CoreTasks.Pick("b", 14), CoreTasks.Park("b", "N1", "up", 40),
            CoreTasks.Walk("a"),
            CoreTasks.Pick("b", 14), CoreTasks.Place("b", 40)
        };
        Assert.Empty(TaskSequenceRules.Validate(ok));
    }

    [Fact]
    public void SequenceRules_FlagWalkingWhileCarrying_PlaceWithoutPick_AndAForgottenPark()
    {
        var bad = new[]
        {
            CoreTasks.Pick("a", 14), CoreTasks.Walk("x"), CoreTasks.Place("a", 40),
            CoreTasks.Place("b", 40),
            CoreTasks.Pick("c", 14), CoreTasks.Park("c", "N1", "up", 40),
            CoreTasks.Pick("d", 14), CoreTasks.Pick("e", 14), CoreTasks.Place("e", 40)
        };
        var msgs = TaskSequenceRules.Validate(bad).Select(p => p.Message).ToList();
        Assert.Contains("walk while carrying a", msgs);
        Assert.Contains("place b without picking it", msgs);
        Assert.Contains("pick e while still holding d", msgs);
        Assert.Contains("c is still parked at the end", msgs);
        Assert.Contains(TaskSequenceRules.Validate(new[] { CoreTasks.Pick("a", 14) }), p => p.Message == "sequence ends holding a");
    }
}
