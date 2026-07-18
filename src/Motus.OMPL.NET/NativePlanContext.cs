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
    private double[]? _validityQ;
    private double[]? _fromQ;
    private double[]? _toQ;
    private double[]? _fromFullQ;
    private double[]? _toFullQ;
    private JointState? _fromFullState;
    private JointState? _toFullState;
    private JointState? _validityFullState;

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

    public int ValidityCallback(IntPtr statePtr, int dims)
    {
        if (_collision is null) return 1;
        var rented = ArrayPool<double>.Shared.Rent(dims);
        try
        {
            Marshal.Copy(statePtr, rented, 0, dims);
            if (_collision is SphereCollisionChecker sphere)
                return sphere.IsCollisionFree(new ArraySegment<double>(rented, 0, dims), _scene) ? 1 : 0;

            EnsureGroupBuffer(ref _validityQ, dims);
            Array.Copy(rented, _validityQ!, dims);
            var full = MaterializeFull(_validityQ!, ref _fromFullQ, ref _validityFullState);
            return _collision.IsCollisionFree(full, _scene) ? 1 : 0;
        }
        finally
        {
            ArrayPool<double>.Shared.Return(rented);
        }
    }

    public int MotionValidityCallback(IntPtr fromPtr, IntPtr toPtr, int dims)
    {
        if (_collision is null) return 1;
        var fromRented = ArrayPool<double>.Shared.Rent(dims);
        var toRented = ArrayPool<double>.Shared.Rent(dims);
        try
        {
            Marshal.Copy(fromPtr, fromRented, 0, dims);
            Marshal.Copy(toPtr, toRented, 0, dims);
            if (_collision is SphereCollisionChecker sphere)
                return sphere.SegmentCollisionFree(
                    new ArraySegment<double>(fromRented, 0, dims),
                    new ArraySegment<double>(toRented, 0, dims),
                    _scene,
                    _stepRadians) ? 1 : 0;

            EnsureGroupBuffer(ref _fromQ, dims);
            EnsureGroupBuffer(ref _toQ, dims);
            Array.Copy(fromRented, _fromQ!, dims);
            Array.Copy(toRented, _toQ!, dims);

            // Distinct full-state buffers — EmbedGroupState reuses one scratch.
            var fromFull = MaterializeFull(_fromQ!, ref _fromFullQ, ref _fromFullState);
            var toFull = MaterializeFull(_toQ!, ref _toFullQ, ref _toFullState);
            return _collision.SegmentCollisionFree(fromFull, toFull, _scene, _stepRadians) ? 1 : 0;
        }
        finally
        {
            ArrayPool<double>.Shared.Return(fromRented);
            ArrayPool<double>.Shared.Return(toRented);
        }
    }

    private JointState MaterializeFull(double[] groupOrFull, ref double[]? fullBuf, ref JointState? fullState)
    {
        if (_toFull is null)
        {
            EnsureGroupBuffer(ref fullBuf, groupOrFull.Length);
            Array.Copy(groupOrFull, fullBuf!, groupOrFull.Length);
            fullState ??= JointState.Wrap(fullBuf!);
            return fullState;
        }

        var embedded = _toFull(groupOrFull);
        var n = embedded.Positions.Length;
        if (fullBuf is null || fullBuf.Length != n)
        {
            fullBuf = new double[n];
            fullState = JointState.Wrap(fullBuf);
        }
        Array.Copy(embedded.Positions, fullBuf, n);
        return fullState!;
    }

    private static void EnsureGroupBuffer(ref double[]? buf, int dims)
    {
        if (buf is null || buf.Length != dims)
            buf = new double[dims];
    }
}
