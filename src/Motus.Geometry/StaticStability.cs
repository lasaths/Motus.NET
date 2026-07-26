namespace Motus.Geometry;

/// <summary>
/// Quasi-static support-polygon stability tests for coplanar point contacts.
/// </summary>
/// <remarks>
/// <para><b>Method (Established):</b> McGhee &amp; Frank, “On the Stability Properties of Quadruped
/// Creeping Gaits,” <i>Mathematical Biosciences</i> 3:331–351, 1968,
/// DOI <see cref="LeggedMethodRefs.McGheeFrank1968Doi"/>. Under quasi-static assumptions with
/// sufficient friction, a sufficient condition for static stability is that the gravity projection
/// of the center of mass lies in the convex hull (support polygon) of the stance contacts.
/// The static stability margin (SSM) is the minimum distance from that projection to the hull
/// boundary (negative if outside).</para>
/// <para><b>Limitations:</b> coplanar contacts, negligible inertia, no actuator saturation.
/// Non-coplanar / wrench-feasible equilibrium: Bretl &amp; Lall, IEEE T-RO 24(4):794–807, 2008,
/// DOI <see cref="LeggedMethodRefs.BretlLall2008Doi"/> (not implemented here).</para>
/// <para><b>Adapted for Motus preview:</b> contacts are treated in the horizontal plane (Z ignored);
/// CoM projection is supplied by the caller (often body XY as a geometric stand-in — document that
/// as a heuristic when used).</para>
/// </remarks>
public static class StaticStability
{
    public readonly record struct Result(
        bool IsStable,
        double MarginMeters,
        int ContactCount,
        string? Failure);

    /// <summary>
    /// Compute SSM for horizontal contacts. <paramref name="comXy"/> is the CoM gravity projection (m).
    /// </summary>
    public static Result Evaluate(IReadOnlyList<Vec3> stanceContactsXy, Vec3 comXy)
    {
        if (stanceContactsXy is null || stanceContactsXy.Count == 0)
            return new Result(false, double.NegativeInfinity, 0, "No stance contacts.");
        if (!comXy.IsFinite)
            return new Result(false, double.NegativeInfinity, stanceContactsXy.Count, "CoM projection is not finite.");

        var pts = new List<(double X, double Y)>(stanceContactsXy.Count);
        foreach (var c in stanceContactsXy)
        {
            if (!c.IsFinite)
                return new Result(false, double.NegativeInfinity, stanceContactsXy.Count, "Non-finite stance contact.");
            pts.Add((c.X, c.Y));
        }

        if (pts.Count == 1)
        {
            var d = Math.Sqrt((pts[0].X - comXy.X) * (pts[0].X - comXy.X) + (pts[0].Y - comXy.Y) * (pts[0].Y - comXy.Y));
            return new Result(false, -d, 1, "Degenerate support (1 contact); need ≥3 non-collinear for a polygon.");
        }

        var hull = ConvexHull2D(pts);
        if (hull.Count < 3)
        {
            // Collinear / 2-point: treat as unstable segment (margin = −dist to segment).
            var d = DistanceToPolyline(comXy.X, comXy.Y, hull, closed: false);
            return new Result(false, -Math.Abs(d), stanceContactsXy.Count,
                "Degenerate support (collinear contacts); convex hull has < 3 vertices.");
        }

        var inside = PointInConvexPolygon(comXy.X, comXy.Y, hull);
        var edgeDist = DistanceToPolyline(comXy.X, comXy.Y, hull, closed: true);
        var margin = inside ? edgeDist : -edgeDist;
        return new Result(inside, margin, stanceContactsXy.Count, inside ? null : "CoM projection outside support polygon.");
    }

    /// <summary>Monotone-chain convex hull (Andrew). Returns CCW vertices.</summary>
    public static List<(double X, double Y)> ConvexHull2D(IReadOnlyList<(double X, double Y)> points)
    {
        var pts = points.ToList();
        pts.Sort((a, b) =>
        {
            var c = a.X.CompareTo(b.X);
            return c != 0 ? c : a.Y.CompareTo(b.Y);
        });
        // Dedup
        var uniq = new List<(double X, double Y)>(pts.Count);
        foreach (var p in pts)
        {
            if (uniq.Count == 0 || Math.Abs(uniq[^1].X - p.X) > 1e-12 || Math.Abs(uniq[^1].Y - p.Y) > 1e-12)
                uniq.Add(p);
        }

        if (uniq.Count <= 2)
            return uniq;

        var lower = new List<(double X, double Y)>();
        foreach (var p in uniq)
        {
            while (lower.Count >= 2 && Cross(lower[^2], lower[^1], p) <= 0)
                lower.RemoveAt(lower.Count - 1);
            lower.Add(p);
        }

        var upper = new List<(double X, double Y)>();
        for (var i = uniq.Count - 1; i >= 0; i--)
        {
            var p = uniq[i];
            while (upper.Count >= 2 && Cross(upper[^2], upper[^1], p) <= 0)
                upper.RemoveAt(upper.Count - 1);
            upper.Add(p);
        }

        lower.RemoveAt(lower.Count - 1);
        upper.RemoveAt(upper.Count - 1);
        lower.AddRange(upper);
        return lower;
    }

    public static bool PointInConvexPolygon(double x, double y, IReadOnlyList<(double X, double Y)> hullCcw)
    {
        var n = hullCcw.Count;
        if (n < 3) return false;
        for (var i = 0; i < n; i++)
        {
            var a = hullCcw[i];
            var b = hullCcw[(i + 1) % n];
            // Inside or on boundary: left of each directed edge (CCW).
            if (Cross(a, b, (x, y)) < -1e-12)
                return false;
        }
        return true;
    }

    private static double DistanceToPolyline(double x, double y, IReadOnlyList<(double X, double Y)> poly, bool closed)
    {
        var n = poly.Count;
        if (n == 0) return double.PositiveInfinity;
        if (n == 1)
            return Math.Sqrt((poly[0].X - x) * (poly[0].X - x) + (poly[0].Y - y) * (poly[0].Y - y));

        var min = double.PositiveInfinity;
        var edges = closed ? n : n - 1;
        for (var i = 0; i < edges; i++)
        {
            var a = poly[i];
            var b = poly[(i + 1) % n];
            min = Math.Min(min, DistPointSegment(x, y, a.X, a.Y, b.X, b.Y));
        }
        return min;
    }

    private static double DistPointSegment(double px, double py, double ax, double ay, double bx, double by)
    {
        var abx = bx - ax;
        var aby = by - ay;
        var apx = px - ax;
        var apy = py - ay;
        var ab2 = abx * abx + aby * aby;
        var t = ab2 < 1e-24 ? 0 : Math.Clamp((apx * abx + apy * aby) / ab2, 0, 1);
        var cx = ax + t * abx;
        var cy = ay + t * aby;
        var dx = px - cx;
        var dy = py - cy;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    private static double Cross((double X, double Y) o, (double X, double Y) a, (double X, double Y) b) =>
        (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);
}
