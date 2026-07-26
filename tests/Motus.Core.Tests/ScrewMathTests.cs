using Motus.Geometry;

namespace Motus.Core.Tests;

public class ScrewMathTests
{
    [Fact]
    public void MatrixExp3_Log3_RoundTrip_SmallAngle()
    {
        var r = ScrewMath.MatrixExp3(0, 0, 1, 0.3);
        Assert.True(ScrewMath.TryMatrixLog3(r, out var wx, out var wy, out var wz, out var theta));
        Assert.Equal(0.3, theta, 6);
        Assert.Equal(0, wx, 6);
        Assert.Equal(0, wy, 6);
        Assert.Equal(1, wz, 6);
    }

    [Fact]
    public void MatrixLog3_Identity_ReturnsZeroTheta()
    {
        Assert.True(ScrewMath.TryMatrixLog3(ScrewMath.Identity3(), out _, out _, out _, out var theta));
        Assert.Equal(0, theta, 12);
    }

    [Fact]
    public void MatrixExp3_Log3_RoundTrip_NearPi()
    {
        var r = ScrewMath.MatrixExp3(1, 0, 0, Math.PI);
        Assert.True(ScrewMath.TryMatrixLog3(r, out var wx, out var wy, out var wz, out var theta));
        Assert.Equal(Math.PI, theta, 5);
        Assert.Equal(1, Math.Abs(wx), 5);
        Assert.Equal(0, wy, 5);
        Assert.Equal(0, wz, 5);
    }

    [Fact]
    public void MatrixLog3_RejectsNonOrthogonal()
    {
        var bad = new[,] { { 2.0, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
        Assert.False(ScrewMath.TryMatrixLog3(bad, out _, out _, out _, out _));
    }

    [Fact]
    public void MatrixLog3_RejectsNaN()
    {
        var bad = new[,] { { double.NaN, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
        Assert.False(ScrewMath.TryMatrixLog3(bad, out _, out _, out _, out _));
    }

    [Fact]
    public void MatrixExp6_Log6_PureTranslation()
    {
        var s = new double[] { 0, 0, 0, 0, 0, 1 };
        var t = ScrewMath.MatrixExp6(s, 0.25);
        Assert.Equal(0.25, t[11], 9);
        Assert.True(ScrewMath.TryMatrixLog6(t, out var s2, out var theta));
        Assert.Equal(0.25, theta, 9);
        Assert.Equal(0, s2[0], 9);
        Assert.Equal(0, s2[1], 9);
        Assert.Equal(0, s2[2], 9);
        Assert.Equal(0, s2[3], 9);
        Assert.Equal(0, s2[4], 9);
        Assert.Equal(1, s2[5], 9);
    }

    [Fact]
    public void MatrixExp6_Log6_RevoluteScrewRoundTrip()
    {
        // Unit ω about z through origin; v = 0 → pure rotation.
        var s = new double[] { 0, 0, 1, 0, 0, 0 };
        var t = ScrewMath.MatrixExp6(s, 0.7);
        Assert.True(ScrewMath.TryMatrixLog6(t, out var s2, out var theta));
        Assert.Equal(0.7, theta, 6);
        Assert.Equal(0, s2[0], 6);
        Assert.Equal(0, s2[1], 6);
        Assert.Equal(1, s2[2], 6);
    }

    [Fact]
    public void MatrixExp6_Log6_OffsetAxisScrew()
    {
        // Revolute about z through (1,0,0): v = −ω × q = −(0,0,1)×(1,0,0) = (0,−1,0)
        var s = new double[] { 0, 0, 1, 0, -1, 0 };
        var t = ScrewMath.MatrixExp6(s, Math.PI / 2);
        Assert.True(ScrewMath.TryMatrixLog6(t, out var s2, out var theta));
        Assert.Equal(Math.PI / 2, theta, 5);
        Assert.Equal(0, s2[0], 5);
        Assert.Equal(0, s2[1], 5);
        Assert.Equal(1, s2[2], 5);
        Assert.Equal(0, s2[3], 4);
        Assert.Equal(-1, s2[4], 4);
        Assert.Equal(0, s2[5], 4);
    }

    [Fact]
    public void Adjoint_TransformsBodyTwistToSpace()
    {
        var t = Transforms.FromRpy(1, 0, 0, 0, 0, Math.PI / 2);
        var ad = ScrewMath.Adjoint(t);
        var vb = new double[] { 0, 0, 1, 0, 0, 0 };
        var vs = ScrewMath.AdjointMultiply(ad, vb);
        // R rotates body z → space y (approx for yaw=π/2 about z... wait RPY yaw about Z:
        // FromRpyRotation: yaw about Z maps body Z stays Z. Use pitch instead.
        Assert.True(double.IsFinite(vs[0]));
        // Vs = Ad Vb; for pure rotation ω_s = R ω_b
        var r = ScrewMath.ExtractR(t);
        Assert.Equal(r[0, 2], vs[0], 9); // ω_s = R * (0,0,1)
        Assert.Equal(r[1, 2], vs[1], 9);
        Assert.Equal(r[2, 2], vs[2], 9);
    }

    [Fact]
    public void Adjoint_InverseMatchesAdjointOfInverse()
    {
        var t = Transforms.FromRpy(0.2, -0.1, 0.5, 0.1, -0.2, 0.3);
        var ad = ScrewMath.Adjoint(t);
        var adInv = ScrewMath.Adjoint(Transforms.Inverse(t));
        var v = new double[] { 0.1, -0.2, 0.3, 0.4, -0.5, 0.6 };
        var round = ScrewMath.AdjointMultiply(adInv, ScrewMath.AdjointMultiply(ad, v));
        for (var i = 0; i < 6; i++)
            Assert.Equal(v[i], round[i], 8);
    }
}
