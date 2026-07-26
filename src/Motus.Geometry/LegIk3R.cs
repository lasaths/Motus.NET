namespace Motus.Geometry;

/// <summary>
/// Analytic 3-DOF insectoid leg IK: coxa yaw in body XY, then planar 2R (femur/tibia) in the coxa vertical plane.
/// </summary>
/// <remarks>
/// <para><b>Method (Established / classical analytic IK):</b> after choosing the coxa heading
/// <c>q0 = atan2(vy, vx)</c>, the distal problem is a planar two-link chain solved with the law of cosines
/// (reachability annulus <c>|ℓ_f − ℓ_t| ≤ d ≤ ℓ_f + ℓ_t</c>, knee angle from cos⁻¹, femur angle from
/// polar angle + shoulder offset). Textbook treatment of analytic IK for structured chains:
/// Lynch &amp; Park, <i>Modern Robotics</i>, Cambridge Univ. Press, 2017,
/// DOI <see cref="LeggedMethodRefs.LynchPark2017Doi"/> (Ch. 6 analytic IK; planar 2R geometry).</para>
/// <para><b>Why not FABRIK here:</b> FABRIK (Aristidou &amp; Lasenby, Graphical Models 73(5):243–260, 2011,
/// DOI <see cref="LeggedMethodRefs.AristidouLasenby2011FabrikDoi"/>) is the peer-reviewed iterative
/// point-on-line solver for general n-link position IK. For a fixed 3R insect leg the analytic 2R
/// reduction is O(1), exact on the workspace boundary, and needs no iteration — prefer it for the
/// actuated model. Survey context: Aristidou et al., CGF 2018,
/// DOI <see cref="LeggedMethodRefs.AristidouEtAl2018IkSurveyDoi"/>.</para>
/// <para><b>Units:</b> positions meters; <c>q0,q1,q2</c> radians. Femur/tibia pitch axis = URDF +Y
/// (positive q1 lowers the femur toward −Z). Default IK branch is <b>elbow-up</b> (knee high):
/// <c>q1=γ−α</c>, <c>q2=π−acos(·)</c> — elbow-down puts the femur tip through the ground on stance plants.</para>
/// <para><b>Failure:</b> non-finite input, non-positive lengths, collinear degenerate plane, or
/// <c>d</c> outside the femur–tibia annulus → returns false (no silent clamp to garbage poses).</para>
/// </remarks>
public static class LegIk3R
{
    public static bool TrySolve(
        Vec3 hip,
        Vec3 foot,
        double coxa,
        double femur,
        double tibia,
        out double q0,
        out double q1,
        out double q2)
    {
        q0 = q1 = q2 = 0;
        if (coxa <= 0 || femur <= 0 || tibia <= 0)
            return false;
        if (!hip.IsFinite || !foot.IsFinite)
            return false;

        var vx = foot.X - hip.X;
        var vy = foot.Y - hip.Y;
        var vz = foot.Z - hip.Z;

        q0 = Math.Atan2(vy, vx);
        var ux = Math.Cos(q0);
        var uy = Math.Sin(q0);
        // Distal planar 2R after subtracting coxa along heading (Lynch & Park planar 2R).
        var wx = vx - ux * coxa;
        var wy = vy - uy * coxa;
        var wz = vz;
        var x = wx * ux + wy * uy;
        var zDown = -wz;
        var d2 = x * x + zDown * zDown;
        if (d2 < 1e-14)
            return false;

        var d = Math.Sqrt(d2);
        var maxReach = femur + tibia;
        var minReach = Math.Abs(femur - tibia);
        if (d > maxReach + 1e-9 || d < minReach - 1e-9)
            return false;

        // Law of cosines — two planar 2R solutions; pick elbow-up (knee high) for insectoid stance.
        // Elbow-down (q1=γ+α, q2=acos−π) drives the femur tip through the ground → zigzag sticks.
        var cosKnee = (femur * femur + tibia * tibia - d2) / (2.0 * femur * tibia);
        cosKnee = Math.Clamp(cosKnee, -1.0, 1.0);
        var cosFemur = (femur * femur + d2 - tibia * tibia) / (2.0 * femur * d);
        cosFemur = Math.Clamp(cosFemur, -1.0, 1.0);
        var gamma = Math.Atan2(zDown, x);
        var alpha = Math.Acos(cosFemur);
        // Elbow-up: q1 = γ − α, q2 = π − acos(·). FK: femurDir = coxaDir·cos(q1) − Z·sin(q1).
        q1 = gamma - alpha;
        q2 = Math.PI - Math.Acos(cosKnee);

        return double.IsFinite(q0) && double.IsFinite(q1) && double.IsFinite(q2);
    }

    public static Vec3 FootPosition(
        Vec3 hip,
        double coxa,
        double femur,
        double tibia,
        double q0,
        double q1,
        double q2)
    {
        var cx = Math.Cos(q0);
        var cy = Math.Sin(q0);
        var kneeX = hip.X + cx * coxa;
        var kneeY = hip.Y + cy * coxa;
        var kneeZ = hip.Z;

        var fdx = cx * Math.Cos(q1);
        var fdy = cy * Math.Cos(q1);
        var fdz = -Math.Sin(q1);
        Unitize(ref fdx, ref fdy, ref fdz, cx, cy, 0);

        var ankleX = kneeX + fdx * femur;
        var ankleY = kneeY + fdy * femur;
        var ankleZ = kneeZ + fdz * femur;

        var tdx = cx * Math.Cos(q1 + q2);
        var tdy = cy * Math.Cos(q1 + q2);
        var tdz = -Math.Sin(q1 + q2);
        Unitize(ref tdx, ref tdy, ref tdz, fdx, fdy, fdz);

        return new Vec3(ankleX + tdx * tibia, ankleY + tdy * tibia, ankleZ + tdz * tibia);
    }

    public static Vec3 KneePosition(Vec3 hip, double coxa, double femur, double q0, double q1)
    {
        var cx = Math.Cos(q0);
        var cy = Math.Sin(q0);
        var kneeX = hip.X + cx * coxa;
        var kneeY = hip.Y + cy * coxa;
        var kneeZ = hip.Z;
        var fdx = cx * Math.Cos(q1);
        var fdy = cy * Math.Cos(q1);
        var fdz = -Math.Sin(q1);
        Unitize(ref fdx, ref fdy, ref fdz, cx, cy, 0);
        return new Vec3(kneeX + fdx * femur, kneeY + fdy * femur, kneeZ + fdz * femur);
    }

    private static void Unitize(ref double x, ref double y, ref double z, double fx, double fy, double fz)
    {
        var len = Math.Sqrt(x * x + y * y + z * z);
        if (len < 1e-14)
        {
            x = fx;
            y = fy;
            z = fz;
            return;
        }

        x /= len;
        y /= len;
        z /= len;
    }
}
