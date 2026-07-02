using System.Runtime.InteropServices;

namespace Motus.OMPL.Native;

/// <summary>C ABI to OMPL C++ (stub when native library not built with OMPL).</summary>
public static partial class NativeOmpl
{
    private const string LibName = "motus_ompl_native";

    public const int Ok = 0;
    public const int Err = -1;
    public const int Unavailable = -2;

    public static bool IsAvailable
    {
        get
        {
            try { return motus_ompl_is_available() != 0; }
            catch (DllNotFoundException) { return false; }
            catch (EntryPointNotFoundException) { return false; }
        }
    }

    public static string StatusMessage =>
        IsAvailable
            ? "Native OMPL binding loaded."
            : "Native OMPL not built. Motus.OMPL.NET uses managed RRT-Connect fallback.";

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    private static extern int motus_ompl_is_available();

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate int ValidityCallback(IntPtr state, int dims, IntPtr userdata);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int motus_ompl_rrt_connect(
        int dims,
        double[] low,
        double[] high,
        double[] start,
        double[] goal,
        int max_iterations,
        double step_size,
        double goal_bias,
        ValidityCallback validity,
        IntPtr validity_userdata,
        double[] out_path,
        int max_states,
        out int out_count);
}
