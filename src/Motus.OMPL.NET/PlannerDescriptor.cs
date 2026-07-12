namespace Motus.OMPL.NET;

/// <summary>Metadata for UI and availability filtering.</summary>
public sealed record PlannerDescriptor(
    SamplingPlannerId Id,
    string Label,
    string ShortName,
    bool NativeSupported,
    bool ManagedSupported,
    string? UnavailableReason);
