namespace Motus.Geometry;

public sealed record CartesianLinOptions(
    double StepMeters = 0.005,
    int MaxSteps = 150,
    int MaxIkAttemptsPerStep = 8,
    bool ContinueOnIkFailure = false)
{
    public double EffectiveStepMeters(double tcpDistanceMeters)
    {
        if (tcpDistanceMeters < 1e-9) return StepMeters;
        if (MaxSteps <= 0) return StepMeters;
        return Math.Max(StepMeters, tcpDistanceMeters / MaxSteps);
    }

    public int StepCount(double tcpDistanceMeters)
    {
        if (tcpDistanceMeters < 1e-9) return 1;
        var step = EffectiveStepMeters(tcpDistanceMeters);
        return Math.Max(2, (int)Math.Ceiling(tcpDistanceMeters / step));
    }
}
