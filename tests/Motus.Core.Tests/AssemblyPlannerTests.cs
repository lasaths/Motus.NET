using Motus.Construction;

namespace Motus.Core.Tests;

public class AssemblyPlannerTests
{
    [Fact]
    public void WarrenExample_StructureMatchesLayoutDefault()
    {
        var ex = AssemblyPlanner.WarrenTrussExample.Structure!;
        Assert.Equal(BambooTrussLayout.FootNodes.Select(n => (n.Name, n.X, n.Y, n.Z)), ex.Nodes.Select(n => (n.Name, n.X, n.Y, n.Z)));
        Assert.Equal(BambooTrussLayout.Members.Select(m => m.Id), ex.Members.Select(m => m.Id));
        Assert.Equal(new[] { "A", "D" }, BambooTrussLayout.SupportNodes);
    }

    [Fact]
    public void WarrenExample_ShippedPlan_IsCollisionFree()
    {
        var plan = AssemblyPlanner.Plan(AssemblyPlanner.WarrenTrussExample with { UseShippedPlan = true });
        Assert.True(plan.CollisionFree, plan.Verdict);
        Assert.Equal(plan.Members.Select(m => m.Id).OrderBy(x => x), plan.Build.Built.OrderBy(x => x));
        Assert.NotEmpty(plan.Keys);
        Assert.Equal(plan.Build.Tasks.Count, plan.Tasks.Count);
    }
}
