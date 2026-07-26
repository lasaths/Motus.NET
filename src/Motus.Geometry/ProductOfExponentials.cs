namespace Motus.Geometry;

/// <summary>
/// Product-of-exponentials FK (Lynch &amp; Park §4.1) for URDF-style <see cref="SerialJointChain"/>.
/// Space screws and home <c>M</c> are defined at Motus joint zeros (URDF <c>ThetaOffset</c> baked into home).
/// Flange-only (no base/tool) — compose with <see cref="Transforms.FromFrame"/> like <see cref="SerialForwardKinematics"/>.
/// </summary>
public sealed class ProductOfExponentials
{
    /// <summary>Home flange pose at all joint variables = 0 (4×4).</summary>
    public double[] HomeM { get; }

    /// <summary>Space-frame screw axes at home; each length-6 (ω, v).</summary>
    public IReadOnlyList<double[]> SpaceScrews { get; }

    /// <summary>Body-frame screws B_i = Ad_{M⁻¹} S_i.</summary>
    public IReadOnlyList<double[]> BodyScrews { get; }

    public int AxisCount => SpaceScrews.Count;

    private ProductOfExponentials(double[] homeM, double[][] spaceScrews, double[][] bodyScrews)
    {
        HomeM = homeM;
        SpaceScrews = spaceScrews;
        BodyScrews = bodyScrews;
    }

    public static ProductOfExponentials FromSerialChain(SerialJointChain chain)
    {
        ArgumentNullException.ThrowIfNull(chain);
        var n = chain.Joints.Length;
        var space = new double[n][];
        var T = Transforms.Identity();

        for (var i = 0; i < n; i++)
        {
            var j = chain.Joints[i];
            var origin = Transforms.FromRpy(j.OriginX, j.OriginY, j.OriginZ, j.Roll, j.Pitch, j.Yaw);
            var Tjoint = Transforms.Multiply(T, origin);
            var motion0 = j.Motion == JointMotionType.Prismatic
                ? Transforms.FromPrismatic(j.AxisX, j.AxisY, j.AxisZ, j.ThetaOffset)
                : Transforms.FromAxisAngle(j.AxisX, j.AxisY, j.AxisZ, j.ThetaOffset);
            var Thome = Transforms.Multiply(Tjoint, motion0);

            NormalizeAxis(j.AxisX, j.AxisY, j.AxisZ, out var ax, out var ay, out var az);
            // Axis direction in space at home (child frame after origin+offset).
            var wx = Thome[0] * ax + Thome[1] * ay + Thome[2] * az;
            var wy = Thome[4] * ax + Thome[5] * ay + Thome[6] * az;
            var wz = Thome[8] * ax + Thome[9] * ay + Thome[10] * az;
            var qx = Thome[3];
            var qy = Thome[7];
            var qz = Thome[11];

            if (j.Motion == JointMotionType.Prismatic)
            {
                space[i] = [0, 0, 0, wx, wy, wz];
            }
            else
            {
                // v = −ω × q
                var vx = -(wy * qz - wz * qy);
                var vy = -(wz * qx - wx * qz);
                var vz = -(wx * qy - wy * qx);
                space[i] = [wx, wy, wz, vx, vy, vz];
            }

            T = Thome;
        }

        var m = T;
        var mInv = Transforms.Inverse(m);
        var adInv = ScrewMath.Adjoint(mInv);
        var body = new double[n][];
        for (var i = 0; i < n; i++)
            body[i] = ScrewMath.AdjointMultiply(adInv, space[i]);

        return new ProductOfExponentials(m, space, body);
    }

    /// <summary>Space-form PoE: T(θ) = e^[S₁]θ₁ … e^[Sₙ]θₙ M.</summary>
    public double[] FKinSpace(IReadOnlyList<double> theta)
    {
        EnsureTheta(theta);
        var t = (double[])HomeM.Clone();
        // Apply distal → proximal: rightmost exponential hits M first.
        for (var i = AxisCount - 1; i >= 0; i--)
            t = Transforms.Multiply(ScrewMath.MatrixExp6(SpaceScrews[i], theta[i]), t);
        return t;
    }

    /// <summary>Body-form PoE: T(θ) = M e^[B₁]θ₁ … e^[Bₙ]θₙ.</summary>
    public double[] FKinBody(IReadOnlyList<double> theta)
    {
        EnsureTheta(theta);
        var t = (double[])HomeM.Clone();
        for (var i = 0; i < AxisCount; i++)
            t = Transforms.Multiply(t, ScrewMath.MatrixExp6(BodyScrews[i], theta[i]));
        return t;
    }

    private void EnsureTheta(IReadOnlyList<double> theta)
    {
        if (theta.Count != AxisCount)
            throw new ArgumentException($"Expected {AxisCount} joints, got {theta.Count}.");
        for (var i = 0; i < theta.Count; i++)
            if (!double.IsFinite(theta[i]))
                throw new ArgumentException($"Non-finite joint value at index {i}.");
    }

    private static void NormalizeAxis(double ax, double ay, double az, out double nx, out double ny, out double nz)
    {
        var len = Math.Sqrt(ax * ax + ay * ay + az * az);
        if (len < 1e-12)
        {
            nx = 0; ny = 0; nz = 1;
            return;
        }
        nx = ax / len; ny = ay / len; nz = az / len;
    }
}
