namespace Motus.OMPL.Native;

/// <summary>Backward-compatible facade over Motus.Native.</summary>
public static class NativeOmpl
{
    public const int Ok = Motus.Native.NativeBindings.Ok;
    public const int Err = Motus.Native.NativeBindings.Err;
    public const int Unavailable = Motus.Native.NativeBindings.Unavailable;

    public static bool IsAvailable => Motus.Native.NativeBindings.OmplIsAvailable();

    public static string StatusMessage =>
        IsAvailable
            ? "Native OMPL binding loaded."
            : "Native OMPL not built. Motus.OMPL.NET uses managed RRT-Connect fallback.";

    public static int motus_ompl_rrt_connect(
        int dims, double[] low, double[] high, double[] start, double[] goal,
        int max_iterations, double max_plan_time_sec, double step_size, double goal_bias, int planner_id,
        Motus.Native.NativeBindings.ValidityCallback validity,
        Motus.Native.NativeBindings.MotionValidityCallback motion_validity,
        IntPtr validity_userdata, double[] out_path, int max_states, out int out_count) =>
        Motus.Native.NativeBindings.motus_ompl_rrt_connect(
            dims, low, high, start, goal, max_iterations, max_plan_time_sec, step_size, goal_bias, planner_id,
            validity, motion_validity, validity_userdata, out_path, max_states, out out_count);

    public static int motus_ompl_simplify_path(
        int dims, double[] path, int path_count, double step_size,
        Motus.Native.NativeBindings.ValidityCallback validity,
        Motus.Native.NativeBindings.MotionValidityCallback motion_validity,
        IntPtr validity_userdata, double[] out_path, int max_states, out int out_count) =>
        Motus.Native.NativeBindings.motus_ompl_simplify_path(
            dims, path, path_count, step_size, validity, motion_validity, validity_userdata, out_path, max_states, out out_count);
}
