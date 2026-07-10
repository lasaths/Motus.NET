using System.Buffers;
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
        var q = ArrayPool<double>.Shared.Rent(dims);
        try
        {
            Marshal.Copy(statePtr, q, 0, dims);
            if (_collision is SphereCollisionChecker sphere)
                return sphere.IsCollisionFree(new ArraySegment<double>(q, 0, dims), _scene) ? 1 : 0;
            var joints = new double[dims];
            Array.Copy(q, joints, dims);
            return _collision.IsCollisionFree(ToFullState(joints), _scene) ? 1 : 0;
        }
        finally
        {
            ArrayPool<double>.Shared.Return(q);
        }
    }

    public int MotionValidityCallback(IntPtr fromPtr, IntPtr toPtr, int dims)
    {
        if (_collision is null) return 1;
        var from = ArrayPool<double>.Shared.Rent(dims);
        var to = ArrayPool<double>.Shared.Rent(dims);
        try
        {
            Marshal.Copy(fromPtr, from, 0, dims);
            Marshal.Copy(toPtr, to, 0, dims);
            if (_collision is SphereCollisionChecker sphere)
                return sphere.SegmentCollisionFree(
                    new ArraySegment<double>(from, 0, dims),
                    new ArraySegment<double>(to, 0, dims),
                    _scene,
                    _stepRadians) ? 1 : 0;
            var fromJoints = new double[dims];
            var toJoints = new double[dims];
            Array.Copy(from, fromJoints, dims);
            Array.Copy(to, toJoints, dims);
            return _collision.SegmentCollisionFree(ToFullState(fromJoints), ToFullState(toJoints), _scene, _stepRadians) ? 1 : 0;
        }
        finally
        {
            ArrayPool<double>.Shared.Return(from);
            ArrayPool<double>.Shared.Return(to);
        }
    }
}
