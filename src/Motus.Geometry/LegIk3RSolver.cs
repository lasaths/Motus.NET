namespace Motus.Geometry;

/// <summary>
/// Analytic 3R insectoid IK wrapper around <see cref="LegIk3R"/> (Lynch &amp; Park planar 2R after coxa).
/// </summary>
public sealed class LegIk3RSolver : ILegIkSolver
{
    public LegIk3RSolver(double coxa, double femur, double tibia)
    {
        if (!(coxa > 0) || !(femur > 0) || !(tibia > 0) ||
            !double.IsFinite(coxa) || !double.IsFinite(femur) || !double.IsFinite(tibia))
            throw new ArgumentException("Coxa/Femur/Tibia must be finite and > 0 (m).");
        Coxa = coxa;
        Femur = femur;
        Tibia = tibia;
        Workspace = new LegIkWorkspace(
            Math.Abs(femur - tibia),
            coxa + femur + tibia,
            $"LegIk3R annulus after coxa; lengths coxa={coxa:F4} femur={femur:F4} tibia={tibia:F4} m");
    }

    public double Coxa { get; }
    public double Femur { get; }
    public double Tibia { get; }
    public LegIkWorkspace Workspace { get; }

    public bool TrySolve(
        Vec3 hipBody,
        Vec3 footTargetBody,
        FootTargetKind kind,
        out double[] q,
        out LegIkFailureCode code)
    {
        q = new double[3];
        if (kind == FootTargetKind.Pose)
        {
            // ponytail: 3R analytic is position-only; Pose rejected rather than silently ignoring R.
            code = LegIkFailureCode.UnsupportedTargetKind;
            return false;
        }

        if (!hipBody.IsFinite || !footTargetBody.IsFinite)
        {
            code = LegIkFailureCode.NonFiniteInput;
            return false;
        }

        if (!LegIk3R.TrySolve(hipBody, footTargetBody, Coxa, Femur, Tibia, out var q0, out var q1, out var q2))
        {
            code = LegIkFailureCode.Unreachable;
            return false;
        }

        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        code = LegIkFailureCode.None;
        return true;
    }

    public bool TryNominalStance(
        Vec3 hipBody,
        double hipYawRad,
        double sideSign,
        double hipStanceRad,
        double femurStanceFallbackRad,
        double tibiaStanceFallbackRad,
        double bodyClearanceMeters,
        out double[] q,
        out Vec3 footBody,
        out LegIkFailureCode code)
    {
        q = new double[3];
        footBody = default;
        if (!hipBody.IsFinite ||
            !double.IsFinite(hipYawRad) || !double.IsFinite(hipStanceRad) ||
            !double.IsFinite(bodyClearanceMeters))
        {
            code = LegIkFailureCode.NonFiniteInput;
            return false;
        }

        var distal = Femur + Tibia;
        var planar = Math.Sqrt(Math.Max(0.0, distal * distal - bodyClearanceMeters * bodyClearanceMeters));
        var plantFromHip = Coxa + 0.70 * planar;
        var heading = hipYawRad + sideSign * hipStanceRad;
        footBody = new Vec3(
            hipBody.X + plantFromHip * Math.Cos(heading),
            hipBody.Y + plantFromHip * Math.Sin(heading),
            0);

        if (TrySolve(hipBody, footBody, FootTargetKind.Position, out q, out code))
            return true;

        // Fallback fixed angles when plant IK fails (e.g. clearance > femur+tibia).
        q = [heading, femurStanceFallbackRad, tibiaStanceFallbackRad];
        footBody = LegIk3R.FootPosition(hipBody, Coxa, Femur, Tibia, q[0], q[1], q[2]);
        code = LegIkFailureCode.None;
        return true;
    }

    public Vec3 FootPosition(Vec3 hipBody, double q0, double q1, double q2) =>
        LegIk3R.FootPosition(hipBody, Coxa, Femur, Tibia, q0, q1, q2);
}
