using System.Runtime.InteropServices;
using Motus.Core;
using Motus.Geometry;

namespace Motus.OMPL.NET;

internal sealed class NativePlanContext
{
    private readonly ICollisionChecker? _collision;
    private readonly CollisionScene _scene;

    public NativePlanContext(ICollisionChecker? collision, CollisionScene scene)
    {
        _collision = collision;
        _scene = scene;
    }

    public int ValidityCallback(IntPtr statePtr, int dims)
    {
        if (_collision is null) return 1;
        var q = new double[dims];
        Marshal.Copy(statePtr, q, 0, dims);
        return _collision.IsCollisionFree(new JointState(q), _scene) ? 1 : 0;
    }
}
