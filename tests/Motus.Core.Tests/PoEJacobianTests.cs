using Motus.Geometry;
using Motus.Presets;

namespace Motus.Core.Tests;

public class PoEJacobianTests
{
    private static string FixturePath(string name) =>
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "fixtures", name));

    [Fact]
    public void JacobianBody_AgreesWithFiniteDiff_TwoLink()
    {
        var robot = UrdfRobotLoader.Load(FixturePath("two_link.urdf"), new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tip_link"
        });
        var poe = ProductOfExponentials.FromSerialChain(robot.Chain);
        double[] q = [0.4, -0.6];
        var analytic = PoEJacobian.JacobianBody(poe, q);
        var numeric = FiniteDiffBodyJacobian(poe, q);
        AssertJacobianClose(analytic, numeric, 5e-4);
    }

    [Fact]
    public void JacobianSpace_AgreesWithFiniteDiff_Ur10e()
    {
        var robot = UrdfRobotLoader.Load(FixturePath("ur10e/ur10e.urdf"), new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tool0"
        });
        var poe = ProductOfExponentials.FromSerialChain(robot.Chain);
        double[] q = [0.2, -0.5, 0.7, -0.4, 0.3, -0.2];
        var analytic = PoEJacobian.JacobianSpace(poe, q);
        var numeric = FiniteDiffSpaceJacobian(poe, q);
        AssertJacobianClose(analytic, numeric, 5e-4);
    }

    [Fact]
    public void JacobianSpace_FirstColumn_Equals_FirstScrew()
    {
        var robot = UrdfRobotLoader.Load(FixturePath("ur10e/ur10e.urdf"), new UrdfLoadOptions
        {
            BaseLink = "base_link",
            TipLink = "tool0"
        });
        var poe = ProductOfExponentials.FromSerialChain(robot.Chain);
        double[] q = [0.2, -0.5, 0.7, -0.4, 0.3, -0.2];
        var js = PoEJacobian.JacobianSpace(poe, q);
        var s0 = poe.SpaceScrews[0];
        for (var r = 0; r < 6; r++)
            Assert.Equal(s0[r], js[r, 0], 12);
    }

    private static double[,] FiniteDiffBodyJacobian(ProductOfExponentials poe, double[] q)
    {
        const double h = 1e-6;
        var n = q.Length;
        var j = new double[6, n];
        for (var i = 0; i < n; i++)
        {
            q[i] += h;
            var tPlus = poe.FKinBody(q);
            q[i] -= 2 * h;
            var tMinus = poe.FKinBody(q);
            q[i] += h;
            var t0 = poe.FKinBody(q);
            var fwd = Transforms.Multiply(Transforms.Inverse(t0), tPlus);
            var back = Transforms.Multiply(Transforms.Inverse(t0), tMinus);
            Assert.True(ScrewMath.TryMatrixLog6(fwd, out var sF, out var thF));
            Assert.True(ScrewMath.TryMatrixLog6(back, out var sB, out var thB));
            for (var r = 0; r < 6; r++)
                j[r, i] = (sF[r] * thF - sB[r] * thB) / (2 * h);
        }
        return j;
    }

    private static double[,] FiniteDiffSpaceJacobian(ProductOfExponentials poe, double[] q)
    {
        const double h = 1e-6;
        var n = q.Length;
        var j = new double[6, n];
        for (var i = 0; i < n; i++)
        {
            q[i] += h;
            var tPlus = poe.FKinSpace(q);
            q[i] -= 2 * h;
            var tMinus = poe.FKinSpace(q);
            q[i] += h;
            var t0 = poe.FKinSpace(q);
            var fwd = Transforms.Multiply(tPlus, Transforms.Inverse(t0));
            var back = Transforms.Multiply(tMinus, Transforms.Inverse(t0));
            Assert.True(ScrewMath.TryMatrixLog6(fwd, out var sF, out var thF));
            Assert.True(ScrewMath.TryMatrixLog6(back, out var sB, out var thB));
            for (var r = 0; r < 6; r++)
                j[r, i] = (sF[r] * thF - sB[r] * thB) / (2 * h);
        }
        return j;
    }

    private static void AssertJacobianClose(double[,] a, double[,] b, double tol)
    {
        Assert.Equal(a.GetLength(0), b.GetLength(0));
        Assert.Equal(a.GetLength(1), b.GetLength(1));
        for (var r = 0; r < a.GetLength(0); r++)
        for (var c = 0; c < a.GetLength(1); c++)
            Assert.True(Math.Abs(a[r, c] - b[r, c]) < tol,
                $"J[{r},{c}] analytic={a[r, c]} numeric={b[r, c]}");
    }
}
