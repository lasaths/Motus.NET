using Motus.Geometry;

namespace Motus.Core.Tests;

public class CartesianLinOptionsTests
{
    [Fact]
    public void When_DistanceOnePointFiveMeters_Then_EffectiveStepIsAboutTenMillimeters()
    {
        var options = new CartesianLinOptions(StepMeters: 0.005, MaxSteps: 150);
        var effective = options.EffectiveStepMeters(1.5);
        Assert.InRange(effective, 0.009, 0.011);
        Assert.Equal(150, options.StepCount(1.5));
    }

    [Fact]
    public void When_ShortMove_Then_UsesRequestedStep()
    {
        var options = new CartesianLinOptions(StepMeters: 0.005, MaxSteps: 150);
        Assert.Equal(0.005, options.EffectiveStepMeters(0.1));
    }
}
