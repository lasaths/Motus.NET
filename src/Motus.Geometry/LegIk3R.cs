namespace Motus.Geometry;

/// <summary>
/// Analytic 3-DOF leg IK (coxa yaw + femur/tibia pitch in coxa vertical plane).
/// q0 = coxa yaw in body XY (includes mount yaw φᵢ).
/// Femur/tibia pitch uses URDF +Y revolute (positive q → −Z in body frame).
/// </summary>
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
        // w = v - u * coxa
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

        var cosKnee = (femur * femur + tibia * tibia - d2) / (2.0 * femur * tibia);
        cosKnee = Math.Clamp(cosKnee, -1.0, 1.0);
        q2 = Math.Acos(cosKnee) - Math.PI;

        var cosFemur = (femur * femur + d2 - tibia * tibia) / (2.0 * femur * d);
        cosFemur = Math.Clamp(cosFemur, -1.0, 1.0);
        // FK: femurDir = coxaDir*cos(q1) - Z*sin(q1) ⇒ q1 = γ + α (not γ − α).
        q1 = Math.Atan2(zDown, x) + Math.Acos(cosFemur);

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

        // femurDir = coxaDir * cos(q1) - Z * sin(q1); already unit when coxaDir is unit
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
