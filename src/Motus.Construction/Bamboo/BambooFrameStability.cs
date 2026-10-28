namespace Motus.Construction;

/// <summary>
/// Static check of a partial structure: the placed members as a linear-elastic 3D Euler–Bernoulli frame with rigid
/// connectors and fully fixed anchor nodes, loaded by self-weight (gravity along viewer −Y) and, optionally, the robot
/// clamped on one member. Answers "does the partial structure stand, and how far does it sag" so the build search can
/// drop or demote build states that would not (<see cref="BambooBuildSearch.Options.Stability"/>).
/// <para>
/// Approximate by design (it is a feasibility filter, not a structural design check): small displacements, no
/// buckling, uniform tube section, connectors treated as rigid and infinitely strong, self-weight lumped at element
/// nodes, robot weight as one vertical point load (its centre-of-mass moment is not modelled), and the robot load
/// placed at the worst interior point of the clamped member rather than at the exact clamp.
/// </para>
/// </summary>
public static class BambooFrameStability
{
    const double Gravity = 9.81;
    const double Poisson = 0.3;

    public sealed record Settings
    {
        /// <summary>Young's modulus of the cane along the fibre (Pa); bamboo culm ≈ 10–20 GPa.</summary>
        public double YoungModulusPa { get; init; } = 15e9;
        /// <summary>Cane density (kg/m³); bamboo culm ≈ 600–800.</summary>
        public double DensityKgM3 { get; init; } = 700;
        /// <summary>Cane outer radius (m); default the cell's strut radius.</summary>
        public double OuterRadiusM { get; init; } = BambooIcdCell.StrutRadius;
        /// <summary>Cane wall thickness (m); at least the outer radius means a solid rod.</summary>
        public double WallM { get; init; } = 0.002;
        /// <summary>Robot weight carried by the member it clamps (kg); the URDF has no masses, so set the real value.</summary>
        public double RobotMassKg { get; init; } = 1.0;
        /// <summary>Reject a build state whose largest node displacement exceeds this (m).</summary>
        public double MaxDeflectionM { get; init; } = 0.005;
        /// <summary>Reject a build state whose largest axial + bending stress exceeds this (Pa); bamboo bending strength ≈ 100–200 MPa.</summary>
        public double MaxStressPa { get; init; } = 50e6;
        /// <summary>Ranking term: added to a candidate's rank per metre of self-weight sag (0 = filter only).</summary>
        public double RankPerMetre { get; init; } = 10;
        /// <summary>Beam elements per member.</summary>
        public int ElementsPerMember { get; init; } = 8;

        internal Settings Validate()
        {
            if (!(YoungModulusPa > 0) || !double.IsFinite(YoungModulusPa)) throw new ArgumentOutOfRangeException(nameof(YoungModulusPa));
            if (!(DensityKgM3 >= 0) || !double.IsFinite(DensityKgM3)) throw new ArgumentOutOfRangeException(nameof(DensityKgM3));
            if (!(OuterRadiusM > 0) || !double.IsFinite(OuterRadiusM)) throw new ArgumentOutOfRangeException(nameof(OuterRadiusM));
            if (!(WallM > 0) || !double.IsFinite(WallM)) throw new ArgumentOutOfRangeException(nameof(WallM));
            if (!(RobotMassKg >= 0) || !double.IsFinite(RobotMassKg)) throw new ArgumentOutOfRangeException(nameof(RobotMassKg));
            if (!(MaxDeflectionM > 0)) throw new ArgumentOutOfRangeException(nameof(MaxDeflectionM));
            if (!(MaxStressPa > 0)) throw new ArgumentOutOfRangeException(nameof(MaxStressPa));
            if (!(RankPerMetre >= 0) || !double.IsFinite(RankPerMetre)) throw new ArgumentOutOfRangeException(nameof(RankPerMetre));
            if (ElementsPerMember is < 1 or > 64) throw new ArgumentOutOfRangeException(nameof(ElementsPerMember));
            return this;
        }

        internal double Area => Math.PI * (OuterRadiusM * OuterRadiusM - Math.Pow(Math.Max(0, OuterRadiusM - WallM), 2));
        internal double I => Math.PI / 4 * (Math.Pow(OuterRadiusM, 4) - Math.Pow(Math.Max(0, OuterRadiusM - WallM), 4));
        /// <summary>Cane mass per metre (kg/m).</summary>
        public double MassPerMetre => DensityKgM3 * Area;
    }

    /// <param name="Ok">Every placed member reaches an anchor and both limits hold.</param>
    /// <param name="MaxDeflectionM">Largest node displacement (m), worst over the robot-load positions.</param>
    /// <param name="MaxStressPa">Largest |N|/A + |M|·r/I over the element ends (Pa).</param>
    /// <param name="Note">One-line summary, with the failed limit when not ok.</param>
    public sealed record Report(bool Ok, double MaxDeflectionM, double MaxStressPa, string Note);

    /// <summary>
    /// Check the frame of <paramref name="placed"/> under self-weight; with <paramref name="clampedMember"/>, also under
    /// the robot (<see cref="Settings.RobotMassKg"/> + <paramref name="payloadKg"/>) at every interior element node of
    /// that member, reporting the worst case.
    /// </summary>
    public static Report Check(BambooStructureGraph graph, IEnumerable<string> placed, Settings? settings = null,
        string? clampedMember = null, double payloadKg = 0)
    {
        ArgumentNullException.ThrowIfNull(graph);
        var s = (settings ?? new Settings()).Validate();
        var ids = placed.ToHashSet(StringComparer.Ordinal);
        var edges = graph.Edges.Where(e => ids.Contains(e.Id)).ToList();
        if (edges.Count == 0) return new Report(true, 0, 0, "empty structure");

        // Every placed member must be connected to an anchor (otherwise it has no support at all).
        var reached = new HashSet<string>(graph.Anchors, StringComparer.Ordinal);
        for (var grown = true; grown;)
        {
            grown = false;
            foreach (var e in edges)
                if (reached.Contains(e.A) != reached.Contains(e.B)) { reached.Add(e.A); reached.Add(e.B); grown = true; }
        }
        var loose = edges.Where(e => !reached.Contains(e.A)).Select(e => e.Id).ToList();
        if (loose.Count > 0) return new Report(false, double.PositiveInfinity, double.PositiveInfinity, $"not connected to an anchor: {string.Join(", ", loose)}");

        // Nodes: graph nodes in use + interior element nodes of each member.
        var pos = new List<(double X, double Y, double Z)>();
        var index = new Dictionary<string, int>(StringComparer.Ordinal);
        int NodeOf(string name)
        {
            if (index.TryGetValue(name, out var i)) return i;
            var n = graph.Nodes.First(x => x.Name == name);
            index[name] = pos.Count; pos.Add((n.X, n.Y, n.Z));
            return pos.Count - 1;
        }
        var ne = s.ElementsPerMember;
        var elements = new List<(int A, int B)>();
        var interior = new Dictionary<string, List<int>>(StringComparer.Ordinal);
        foreach (var e in edges)
        {
            int a = NodeOf(e.A), b = NodeOf(e.B);
            var chain = new List<int> { a };
            for (var k = 1; k < ne; k++)
            {
                var t = (double)k / ne;
                pos.Add((pos[a].X + t * (pos[b].X - pos[a].X), pos[a].Y + t * (pos[b].Y - pos[a].Y), pos[a].Z + t * (pos[b].Z - pos[a].Z)));
                chain.Add(pos.Count - 1);
            }
            chain.Add(b);
            interior[e.Id] = chain.Skip(1).Take(ne - 1).ToList();
            for (var k = 0; k < ne; k++) elements.Add((chain[k], chain[k + 1]));
        }

        var n6 = pos.Count * 6;
        var fixedDof = new bool[n6];
        foreach (var name in graph.Anchors)
            if (index.TryGetValue(name, out var i)) for (var d = 0; d < 6; d++) fixedDof[i * 6 + d] = true;
        var free = Enumerable.Range(0, n6).Where(d => !fixedDof[d]).ToArray();
        var map = new int[n6];
        Array.Fill(map, -1);
        for (var i = 0; i < free.Length; i++) map[free[i]] = i;

        double E = s.YoungModulusPa, G = E / (2 * (1 + Poisson)), A = s.Area, I = s.I, J = 2 * I;
        var K = new double[free.Length, free.Length];
        var selfWeight = new double[free.Length];
        var locals = new List<(double[,] K, double[,] T)>();
        foreach (var (a, b) in elements)
        {
            var (kl, T, L) = Element(pos[a], pos[b], E, G, A, I, J);
            locals.Add((kl, T));
            var kg = Multiply(Transpose(T), Multiply(kl, T));
            for (var r = 0; r < 12; r++)
            {
                var gr = map[(r < 6 ? a : b) * 6 + r % 6];
                if (gr < 0) continue;
                for (var c = 0; c < 12; c++)
                {
                    var gc = map[(c < 6 ? a : b) * 6 + c % 6];
                    if (gc >= 0) K[gr, gc] += kg[r, c];
                }
            }
            var half = s.MassPerMetre * L * Gravity / 2;   // lumped self-weight, −Y
            if (map[a * 6 + 1] >= 0) selfWeight[map[a * 6 + 1]] -= half;
            if (map[b * 6 + 1] >= 0) selfWeight[map[b * 6 + 1]] -= half;
        }

        if (!Cholesky(K)) return new Report(false, double.PositiveInfinity, double.PositiveInfinity, "mechanism (singular stiffness)");

        var cases = new List<double[]> { selfWeight };
        if (clampedMember is not null && interior.TryGetValue(clampedMember, out var spots))
        {
            var w = (s.RobotMassKg + Math.Max(0, payloadKg)) * Gravity;
            foreach (var node in spots)
            {
                var f = (double[])selfWeight.Clone();
                if (map[node * 6 + 1] >= 0) f[map[node * 6 + 1]] -= w;
                cases.Add(f);
            }
        }

        double maxD = 0, maxS = 0;
        var ro = s.OuterRadiusM;
        foreach (var f in cases)
        {
            var uf = Solve(K, f);
            var u = new double[n6];
            for (var i = 0; i < free.Length; i++) u[free[i]] = uf[i];
            for (var i = 0; i < pos.Count; i++)
                maxD = Math.Max(maxD, Math.Sqrt(u[i * 6] * u[i * 6] + u[i * 6 + 1] * u[i * 6 + 1] + u[i * 6 + 2] * u[i * 6 + 2]));
            for (var k = 0; k < elements.Count; k++)
            {
                var (a, b) = elements[k];
                var ue = new double[12];
                for (var d = 0; d < 6; d++) { ue[d] = u[a * 6 + d]; ue[6 + d] = u[b * 6 + d]; }
                var fl = MultiplyVec(locals[k].K, MultiplyVec(locals[k].T, ue));
                for (var end = 0; end < 2; end++)
                {
                    var o = end * 6;
                    var stress = Math.Abs(fl[o]) / A + Math.Sqrt(fl[o + 4] * fl[o + 4] + fl[o + 5] * fl[o + 5]) * ro / I;
                    maxS = Math.Max(maxS, stress);
                }
            }
        }
        var ok = maxD <= s.MaxDeflectionM && maxS <= s.MaxStressPa;
        var note = $"sag {maxD * 1000:F2} mm, stress {maxS / 1e6:F1} MPa" + (clampedMember is null ? "" : $" (robot on {clampedMember})");
        if (!ok) note += maxD > s.MaxDeflectionM ? $" — sag over {s.MaxDeflectionM * 1000:F1} mm" : $" — stress over {s.MaxStressPa / 1e6:F0} MPa";
        return new Report(ok, maxD, maxS, note);
    }

    /// <summary>Local 12×12 stiffness, global→local transform and length of a straight prismatic beam.</summary>
    static (double[,] K, double[,] T, double L) Element((double X, double Y, double Z) p, (double X, double Y, double Z) q,
        double E, double G, double A, double I, double J)
    {
        double dx = q.X - p.X, dy = q.Y - p.Y, dz = q.Z - p.Z;
        var L = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        if (!(L > 1e-9)) throw new ArgumentException("zero-length member element");
        double[] ex = { dx / L, dy / L, dz / L };
        double[] refAxis = Math.Abs(ex[1]) < 0.9 ? new[] { 0.0, 1, 0 } : new[] { 1.0, 0, 0 };
        var ez = Normalize(Cross(ex, refAxis));
        var ey = Cross(ez, ex);
        var T = new double[12, 12];
        for (var blk = 0; blk < 4; blk++)
            for (var c = 0; c < 3; c++)
            {
                T[blk * 3 + 0, blk * 3 + c] = ex[c];
                T[blk * 3 + 1, blk * 3 + c] = ey[c];
                T[blk * 3 + 2, blk * 3 + c] = ez[c];
            }
        var k = new double[12, 12];
        void Set(int r, int c, double v) { k[r, c] += v; if (r != c) k[c, r] += v; }
        double ea = E * A / L, gj = G * J / L, b3 = 12 * E * I / (L * L * L), b2 = 6 * E * I / (L * L), b1 = 4 * E * I / L, b0 = 2 * E * I / L;
        Set(0, 0, ea); Set(6, 6, ea); Set(0, 6, -ea);
        Set(3, 3, gj); Set(9, 9, gj); Set(3, 9, -gj);
        // v / θz (bending in the local x–y plane)
        Set(1, 1, b3); Set(7, 7, b3); Set(1, 7, -b3);
        Set(1, 5, b2); Set(1, 11, b2); Set(5, 7, -b2); Set(7, 11, -b2);
        Set(5, 5, b1); Set(11, 11, b1); Set(5, 11, b0);
        // w / θy (bending in the local x–z plane)
        Set(2, 2, b3); Set(8, 8, b3); Set(2, 8, -b3);
        Set(2, 4, -b2); Set(2, 10, -b2); Set(4, 8, b2); Set(8, 10, b2);
        Set(4, 4, b1); Set(10, 10, b1); Set(4, 10, b0);
        return (k, T, L);
    }

    static double[] Cross(double[] a, double[] b) => new[] { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
    static double[] Normalize(double[] v) { var n = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); return new[] { v[0] / n, v[1] / n, v[2] / n }; }

    static double[,] Transpose(double[,] m)
    {
        var t = new double[m.GetLength(1), m.GetLength(0)];
        for (var r = 0; r < m.GetLength(0); r++) for (var c = 0; c < m.GetLength(1); c++) t[c, r] = m[r, c];
        return t;
    }

    static double[,] Multiply(double[,] a, double[,] b)
    {
        int n = a.GetLength(0), m = b.GetLength(1), inner = a.GetLength(1);
        var r = new double[n, m];
        for (var i = 0; i < n; i++) for (var k = 0; k < inner; k++) { var v = a[i, k]; if (v == 0) continue; for (var j = 0; j < m; j++) r[i, j] += v * b[k, j]; }
        return r;
    }

    static double[] MultiplyVec(double[,] a, double[] x)
    {
        var r = new double[a.GetLength(0)];
        for (var i = 0; i < r.Length; i++) for (var j = 0; j < x.Length; j++) r[i] += a[i, j] * x[j];
        return r;
    }

    /// <summary>In-place lower Cholesky; false when the matrix is not (numerically) positive definite.</summary>
    static bool Cholesky(double[,] a)
    {
        var n = a.GetLength(0);
        var scale = 0.0;
        for (var i = 0; i < n; i++) scale = Math.Max(scale, Math.Abs(a[i, i]));
        for (var j = 0; j < n; j++)
        {
            var d = a[j, j];
            for (var k = 0; k < j; k++) d -= a[j, k] * a[j, k];
            if (!(d > 1e-12 * scale)) return false;
            a[j, j] = Math.Sqrt(d);
            for (var i = j + 1; i < n; i++)
            {
                var v = a[i, j];
                for (var k = 0; k < j; k++) v -= a[i, k] * a[j, k];
                a[i, j] = v / a[j, j];
            }
        }
        return true;
    }

    static double[] Solve(double[,] l, double[] b)
    {
        var n = b.Length;
        var y = new double[n];
        for (var i = 0; i < n; i++) { var v = b[i]; for (var k = 0; k < i; k++) v -= l[i, k] * y[k]; y[i] = v / l[i, i]; }
        var x = new double[n];
        for (var i = n - 1; i >= 0; i--) { var v = y[i]; for (var k = i + 1; k < n; k++) v -= l[k, i] * x[k]; x[i] = v / l[i, i]; }
        return x;
    }
}
