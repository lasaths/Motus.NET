using System.Runtime.InteropServices;

namespace Motus.Native;

/// <summary>P/Invoke to motus_native (OMPL + FCL C ABI).</summary>
public static partial class NativeBindings
{
    private const string LibName = "motus_native";

    public const int Ok = 0;
    public const int Err = -1;
    public const int Unavailable = -2;

    public const int PlannerRrtConnect = 0;
    public const int PlannerRrtStar = 1;
    public const int PlannerAorrtc = 2;
    public const int PlannerLbkpiece = 3;
    public const int PlannerAitStar = 4;
    public const int PlannerEitStar = 5;
    public const int PlannerBlitStar = 6;

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr motus_last_error();

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int motus_ompl_is_available();

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int motus_ompl_planner_available(int planner_id);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int motus_fcl_is_available();

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate int ValidityCallback(IntPtr state, int dims, IntPtr userdata);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate int MotionValidityCallback(IntPtr from, IntPtr to, int dims, IntPtr userdata);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int motus_ompl_plan(
        int dims,
        double[] low,
        double[] high,
        double[] start,
        double[] goal,
        int max_iterations,
        double max_plan_time_sec,
        double step_size,
        double goal_bias,
        int planner_id,
        ValidityCallback validity,
        MotionValidityCallback? motion_validity,
        IntPtr validity_userdata,
        double[] out_path,
        int max_states,
        out int out_count);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int motus_ompl_rrt_connect(
        int dims,
        double[] low,
        double[] high,
        double[] start,
        double[] goal,
        int max_iterations,
        double max_plan_time_sec,
        double step_size,
        double goal_bias,
        int planner_id,
        ValidityCallback validity,
        MotionValidityCallback? motion_validity,
        IntPtr validity_userdata,
        double[] out_path,
        int max_states,
        out int out_count);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int motus_ompl_simplify_path(
        int dims,
        double[] path,
        int path_count,
        double step_size,
        ValidityCallback validity,
        MotionValidityCallback? motion_validity,
        IntPtr validity_userdata,
        double[] out_path,
        int max_states,
        out int out_count);

    public static bool OmplIsAvailable()
    {
        NativeLibraryBootstrap.EnsureResolver();
        try { return motus_ompl_is_available() != 0; }
        catch (DllNotFoundException) { return false; }
        catch (EntryPointNotFoundException) { return false; }
    }

    public static bool FclIsAvailable()
    {
        NativeLibraryBootstrap.EnsureResolver();
        try { return motus_fcl_is_available() != 0; }
        catch (DllNotFoundException) { return false; }
        catch (EntryPointNotFoundException) { return false; }
    }

    /// <summary>True when motus_native was loaded from runtimes/{rid}/native (stub or full).</summary>
    public static bool LibraryLoaded
    {
        get
        {
            NativeLibraryBootstrap.EnsureResolver();
            return NativeLibraryBootstrap.IsLoaded;
        }
    }

    public static string LastError()
    {
        try
        {
            var ptr = motus_last_error();
            return ptr == IntPtr.Zero ? "" : Marshal.PtrToStringUTF8(ptr) ?? "";
        }
        catch { return ""; }
    }
}
