using Motus.Native;

namespace Motus.OMPL.NET;

/// <summary>Runtime capability probe for hosts (Grasshopper, CLI).</summary>
public static class MotusCapabilities
{
    public static bool NativeOmpl => NativeBindings.OmplIsAvailable();
    public static bool NativeFcl => NativeBindings.FclIsAvailable();
    public static bool AttachSupported => true;
    /// <summary>True when planning/collision run on C# paths (expected on Rhino 8 Win/Mac without OMPL/FCL linked).</summary>
    public static bool UsesManagedFallback => !NativeOmpl || !NativeFcl;

    public static string Describe()
    {
        var ompl = NativeOmpl ? "native OMPL" : "managed RRT-Connect";
        var collision = NativeFcl ? "native FCL" : "C# mesh collision";
        return $"{ompl}, {collision}, attach supported";
    }
}
