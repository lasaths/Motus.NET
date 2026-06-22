namespace Motus.OMPL.Native;

/// <summary>Reserved for future OMPL C++ ABI. Planning ships in Motus.OMPL.NET (pure C# RRT-Connect).</summary>
public static class OmplNativeStatus
{
    public const string Message = "Native OMPL binding not built. Motus.OMPL.NET provides RRT-Connect in managed code.";
    public static bool IsAvailable => false;
}
