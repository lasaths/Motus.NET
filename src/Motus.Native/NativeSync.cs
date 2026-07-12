namespace Motus.Native;

/// <summary>Serializes motus_native entry (OMPL + FCL). Reentrant for OMPL validity callbacks.</summary>
public static class NativeSync
{
    public static readonly object Gate = new();
}
