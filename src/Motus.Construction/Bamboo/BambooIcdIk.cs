using Motus.Core;

namespace Motus.Viewer.Services;

/// <summary>
/// Motus <see cref="IInverseKinematics"/> for the hung bamboo arm: converts a Motus Z-up TCP frame to the
/// viewer plane (y, z, pitch) and solves with the closed-form planar IK used by the viewer motion
/// (<see cref="BambooIcdMotion.IkSolve"/>). Lets <see cref="TaskPathValidator"/> validate bamboo task paths
/// with the same branches the viewer plays. Off-plane (x ≠ 0) or non-pure-pitch targets fail.
/// </summary>
public sealed class BambooIcdIk : IInverseKinematics
{
    public bool TrySolve(CartesianPose target, JointState seed, out JointState solution)
    {
        solution = seed;
        var f = target.Tcp;
        var (x, y, z) = (f.X, f.Z, -f.Y);
        if (Math.Abs(x) > 1e-4) return false;

        // viewer R = Rx(-90°)·R_motus; must be a pure Rx(α).
        var m = Motus.Geometry.Transforms.FromFrame(f);
        var rv = Motus.Geometry.Transforms.Multiply(Motus.Geometry.Transforms.FromAxisAngle(1, 0, 0, -Math.PI / 2), m);
        if (Math.Abs(rv[0] - 1) > 1e-3) return false;
        var alpha = Math.Atan2(rv[9], rv[5]) / BambooIcdMotion.Deg;

        var sols = BambooIcdMotion.IkSolve(y, z, alpha);
        if (sols.Count == 0) return false;
        double[]? best = null; var bestD = double.MaxValue;
        foreach (var q in sols)
        {
            var d = 0.0;
            for (var i = 0; i < q.Length; i++)
            {
                var s = i < seed.Positions.Length ? seed.Positions[i] : 0;
                var e = q[i] * BambooIcdMotion.Deg - s;
                d += e * e;
            }
            if (d < bestD) { bestD = d; best = q; }
        }
        solution = new JointState(BambooIcdCell.RadFromDeg(best!));
        return true;
    }
}
