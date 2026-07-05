using System.Runtime.InteropServices;
using Motus.Core;
using Motus.Geometry;

namespace Motus.OMPL.NET;

internal sealed class NativePlanContext
{
    private readonly ICollisionChecker? _collision;
    private readonly CollisionScene _scene;
    private readonly double _stepRadians;
    private readonly Func<double[], JointState>? _toFull;

    public NativePlanContext(
        ICollisionChecker? collision,
        CollisionScene scene,
        double stepRadians,
        Func<double[], JointState>? toFull = null)
    {
        _collision = collision;
        _scene = scene;
        _stepRadians = stepRadians;
        _toFull = toFull;
    }

    private JointState ToFullState(double[] q) =>
        _toFull is not null ? _toFull(q) : new JointState(q);

    public int ValidityCallback(IntPtr statePtr, int dims)
    {
        if (_collision is null) return 1;
        var q = new double[dims];
        Marshal.Copy(statePtr, q, 0, dims);
        return _collision.IsCollisionFree(ToFullState(q), _scene) ? 1 : 0;
    }

    public int MotionValidityCallback(IntPtr fromPtr, IntPtr toPtr, int dims)
    {
        if (_collision is null) return 1;
        var from = new double[dims];
        var to = new double[dims];
        Marshal.Copy(fromPtr, from, 0, dims);
        Marshal.Copy(toPtr, to, 0, dims);
        return _collision.SegmentCollisionFree(ToFullState(from), ToFullState(to), _scene, _stepRadians) ? 1 : 0;
    }
}
