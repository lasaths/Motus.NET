using System.Runtime.InteropServices;

namespace Motus.Native;

public static partial class NativeBindings
{
    [StructLayout(LayoutKind.Sequential)]
    public struct MotusTransform
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
        public double[] M;

        public static MotusTransform FromMatrix(double[] m) => new() { M = m };
    }

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr motus_fcl_world_create();

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void motus_fcl_world_destroy(IntPtr world);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int motus_fcl_remove(IntPtr world, uint id);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int motus_fcl_upsert_sphere(IntPtr world, uint id, ref MotusTransform pose, double radius);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int motus_fcl_upsert_box(
        IntPtr world, uint id, ref MotusTransform pose, double halfX, double halfY, double halfZ);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int motus_fcl_upsert_capsule(
        IntPtr world, uint id, ref MotusTransform pose, double radius, double halfLength);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int motus_fcl_set_allowed_pair(IntPtr world, uint a, uint b);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int motus_fcl_check(IntPtr world, out int outA, out int outB);
}
