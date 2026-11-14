using Motus.Core;
using Motus.Geometry;

namespace Motus.Core.Tests;

/// <summary>The box prefilter may only say "clear" where the exact test also says clear.</summary>
public class BoxPrefilterTests
{
    static Frame RandomPose(Random rng, double spread)
    {
        double qw = rng.NextDouble() * 2 - 1, qx = rng.NextDouble() * 2 - 1, qy = rng.NextDouble() * 2 - 1, qz = rng.NextDouble() * 2 - 1;
        var n = Math.Sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        return new Frame((rng.NextDouble() * 2 - 1) * spread, (rng.NextDouble() * 2 - 1) * spread, (rng.NextDouble() * 2 - 1) * spread,
            qw / n, qx / n, qy / n, qz / n);
    }

    /// <summary>A cane-like 12-sided prism (radius r, along local Z from z0 to z1; off-centre on purpose).</summary>
    static CollisionObject Cane(string name, Frame pose, double r, double z0, double z1)
    {
        var v = new List<double[]>(); var idx = new List<int>();
        for (var i = 0; i < 12; i++)
        {
            var a = i * Math.PI / 6;
            v.Add(new[] { 0.01 + r * Math.Cos(a), r * Math.Sin(a), z0 });
            v.Add(new[] { 0.01 + r * Math.Cos(a), r * Math.Sin(a), z1 });
        }
        for (var i = 0; i < 12; i++)
        {
            int a0 = 2 * i, a1 = 2 * i + 1, b0 = 2 * ((i + 1) % 12), b1 = b0 + 1;
            idx.AddRange(new[] { a0, b0, a1, a1, b0, b1 });
        }
        for (var i = 1; i < 11; i++) idx.AddRange(new[] { 0, 2 * i, 2 * (i + 1), 1, 2 * (i + 1) + 1, 2 * i + 1 });
        return CollisionObject.Mesh(name, pose, v, idx);
    }

    [Fact]
    public void Prefilter_NeverClearsAPairTheExactTestHits()
    {
        var rng = new Random(7);
        var cache = new Dictionary<int, BvhNode>();
        var scratch = new CollisionQueryScratch();
        int cleared = 0, hits = 0, n = 20000;
        for (var k = 0; k < n; k++)
        {
            var obstacle = k % 3 == 2
                ? CollisionObject.Box("box", RandomPose(rng, 0.05), 0.04 * rng.NextDouble() + 0.002, 0.02, 0.1 * rng.NextDouble() + 0.002)
                : Cane("cane", RandomPose(rng, 0.05), 0.01, -0.3, 0.2);
            CollisionObject part = k % 2 == 0
                ? CollisionObject.Box("part", Frame.Identity, 0.005 + 0.03 * rng.NextDouble(), 0.005 + 0.03 * rng.NextDouble(), 0.002 + 0.01 * rng.NextDouble())
                : Cane("part", RandomPose(rng, 0.02), 0.008, -0.05, 0.08);
            var worldM = Transforms.FromFrame(RandomPose(rng, 0.12));
            var exact = CollisionGeometry.IntersectsAtPose(part, worldM, obstacle, cache, scratch);
            if (exact) hits++;
            if (!CollisionGeometry.ObbApart(part, worldM, obstacle, scratch)) continue;
            cleared++;
            Assert.False(exact, $"case {k}: prefilter cleared a colliding pair");
        }
        // The test must exercise both outcomes, and the prefilter must clear most pairs it can.
        Assert.InRange(hits, n / 50, n);
        Assert.InRange(cleared, n / 4, n - hits);
    }

    [Fact]
    public void Prefilter_SaysMaybeForSpheresAndPlanes()
    {
        var part = CollisionObject.Box("p", Frame.Identity, 0.1, 0.1, 0.1);
        var m = Transforms.FromFrame(Frame.Identity);
        Assert.False(CollisionGeometry.ObbApart(part, m, CollisionObject.Sphere("s", new Frame(5, 0, 0), 0.1), new CollisionQueryScratch()));
        Assert.False(CollisionGeometry.ObbApart(part, m, CollisionObject.Plane("g", new Frame(-5, 0, 0)), new CollisionQueryScratch()));
    }
}
