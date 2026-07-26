using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Foot-target gait: duty-factor swing groups, planted stance contacts, per-leg <see cref="LegIk3R"/>.
/// Preview / TreeFK only — not Motus Plan. Path is a polyline in XY (arc-length); optional terrain height at (x,y).
/// </summary>
/// <remarks>
/// <para><b>Gait timing (Established concepts):</b> duty factor and periodic swing phasing follow the
/// analytical gait framework of Song &amp; Waldron, “An Analytical Approach for Gait Study and Its
/// Applications on Wave Gaits,” <i>IJRR</i> 6(2):60–71, 1987,
/// DOI <see cref="LeggedMethodRefs.SongWaldron1987Doi"/>. Swing-group partition is a design choice
/// (tripod for hex); not claimed as uniquely biological.</para>
/// <para><b>Stance plants (Established creeping-gait idea):</b> stance feet held fixed in the world
/// while the body moves — McGhee &amp; Frank creeping-gait contact model,
/// DOI <see cref="LeggedMethodRefs.McGheeFrank1968Doi"/>. Quasi-static support-polygon SSM is evaluated
/// per sample via <see cref="StaticStability"/> (same DOI); body XY stands in for CoM projection
/// (heuristic geometric CoM — labeled in Status).</para>
/// <para><b>IK:</b> <see cref="LegIk3R"/> analytic (Lynch &amp; Park DOI
/// <see cref="LeggedMethodRefs.LynchPark2017Doi"/>), not FABRIK.</para>
/// <para><b>Engineering adaptations (Heuristic — not theorems):</b> sinusoidal swing lift
/// <c>h = H·sin(πs)</c> above lerped terrain; forward land bias along heading; drift-based replant
/// threshold vs step length; event-driven stretch swing when plant drifts from nominal;
/// body Z = terrain(x,y) + BodyZ clearance (mean-stance body height heuristic).</para>
/// </remarks>
public static class LeggedGait
{
    /// <summary>World ground height (m) at horizontal (x, y). Flat ground = always 0.</summary>
    public delegate double TerrainHeight(double x, double y);

    public sealed class Result
    {
        public Trajectory Trajectory { get; }
        public IReadOnlyList<Frame> BasePath { get; }
        public string? Warning { get; }
        /// <summary>Minimum McGhee–Frank SSM over samples (m); negative ⇒ CoM left polygon at some sample.</summary>
        public double MinStaticStabilityMarginMeters { get; }
        /// <summary>DOI-backed method stack string for Status / logs.</summary>
        public string MethodProvenance { get; }

        public Result(
            Trajectory trajectory,
            IReadOnlyList<Frame> basePath,
            string? warning,
            double minStaticStabilityMarginMeters,
            string methodProvenance)
        {
            Trajectory = trajectory;
            BasePath = basePath;
            Warning = warning;
            MinStaticStabilityMarginMeters = minStaticStabilityMarginMeters;
            MethodProvenance = methodProvenance;
        }
    }

    public static bool TryBuild(
        LeggedLayout layout,
        IReadOnlyList<Vec3> pathXy,
        double speed,
        double stepLength,
        double stepHeight,
        double hipStance,
        double femurStance,
        double tibiaStance,
        RobotModel model,
        out Result? result,
        out string error,
        TerrainHeight? terrain = null)
    {
        result = null;
        error = "";
        // ponytail: null terrain = flat Z=0 (same as pre-terrain contract).
        terrain ??= static (_, _) => 0;

        if (layout.Validate() is { } layoutErr)
        {
            error = layoutErr;
            return false;
        }

        if (!double.IsFinite(speed) || speed <= 0)
        {
            error = "Speed must be finite and > 0 (m/s).";
            return false;
        }

        if (!double.IsFinite(stepLength) || stepLength <= 0)
        {
            error = "Step must be finite and > 0 (m).";
            return false;
        }

        if (!double.IsFinite(stepHeight) || stepHeight < 0)
        {
            error = "Lift must be finite and ≥ 0 (m).";
            return false;
        }

        if (pathXy is null || pathXy.Count < 2)
        {
            error = "Path empty — need ≥ 2 polyline points (m, XY used for arc-length).";
            return false;
        }

        if (!TryBuildPolyline(pathXy, out var cumLen, out var pathLength, out error))
            return false;

        if (pathLength < 0.05)
        {
            error = $"Path too short ({pathLength:F3} m) — need ≥ 0.05 m for gait preview.";
            return false;
        }

        var n = layout.LegCount;
        var dof = layout.DriverCount;
        if (model.Preset.AxisCount != dof)
        {
            error = $"RobotModel AxisCount ({model.Preset.AxisCount}) must equal layout drivers ({dof}).";
            return false;
        }

        var stanceQ = BuildStanceQ(layout, hipStance, femurStance, tibiaStance);
        var duration = pathLength / speed;
        const double sampleHz = 30.0;
        var dt = 1.0 / sampleHz;
        var sampleCount = Math.Max(2, (int)Math.Ceiling(duration / dt) + 1);

        var cyclesPerPath = Math.Max(1.0, pathLength / stepLength);
        if (cyclesPerPath > 200)
        {
            error = $"Step {stepLength:F4} m is too small for path {pathLength:F3} m (>{200:F0} cycles) — increase Step.";
            return false;
        }

        var points = new List<TrajectoryPoint>(sampleCount);
        var basePath = new List<Frame>(sampleCount);

        SampleAt(pathXy, cumLen, pathLength, 0, out var startX, out var startY, out var startYaw);
        if (!TryHeight(terrain, startX, startY, out var startZ, out error))
            return false;
        var startFrame = new MobilityModel.HolonomicSE2(startX, startY, startYaw, startZ).BaseFrame;

        var plants = InitializePlants(layout, startFrame, stanceQ, terrain, out var nominalFootBody, out var initErr);
        if (plants is null)
        {
            error = initErr;
            return false;
        }

        // Planar hip–foot over-reach (foot near body floor): coxa + sqrt((femur+tibia)² − BodyZ²).
        // Using sum-of-links ignored BodyZ and falsely marked every stance leg as stretched.
        var distal = layout.Femur + layout.Tibia;
        var maxHoriz = layout.Coxa + Math.Sqrt(Math.Max(0, distal * distal - layout.BodyZ * layout.BodyZ));
        var maxStanceReach = 1.05 * maxHoriz;
        var driftReplant = Math.Max(0.02, 0.55 * stepLength);
        var groupCount = layout.SwingGroups.Count;
        var swingPeriodSec = duration / (cyclesPerPath * groupCount);
        // Land a bit ahead of nominal along heading so stance doesn't start already aft.
        var landBias = 0.30 * stepLength;

        var qPrev = (double[])stanceQ.Clone();
        var legSwingPhase = new double[n];
        Array.Fill(legSwingPhase, -1.0);
        var swingFrom = new Vec3[n];
        var swingTo = new Vec3[n];
        var ikFailSamples = 0;
        var minSsm = double.PositiveInfinity;
        var unstableSamples = 0;

        for (var i = 0; i < sampleCount; i++)
        {
            var tSec = Math.Min(duration, i * dt);
            var arcLen = speed * tSec;
            SampleAt(pathXy, cumLen, pathLength, arcLen, out var px, out var py, out var yaw);
            if (!TryHeight(terrain, px, py, out var bodyGroundZ, out error))
                return false;
            var baseFrame = new MobilityModel.HolonomicSE2(px, py, yaw, bodyGroundZ).BaseFrame;

            var pathPhase = duration > 1e-9 ? tSec / duration : 0;
            var cyclePhase = (pathPhase * cyclesPerPath) % 1.0;
            var q = (double[])qPrev.Clone();
            var headingX = Math.Cos(yaw);
            var headingY = Math.Sin(yaw);
            var stanceContacts = new List<Vec3>(n);

            for (var leg = 0; leg < n; leg++)
            {
                var (phaseSwinging, phaseLocal) = LegSwingPhase(layout, leg, cyclePhase);
                var hipWorld = HipWorld(layout, leg, baseFrame);
                if (!TryNominalFootWorld(leg, nominalFootBody, baseFrame, terrain, out var nominalLand, out error))
                    return false;
                // Chase current body — frozen swing-start targets leave aft legs dragging.
                var landX = nominalLand.X + headingX * landBias;
                var landY = nominalLand.Y + headingY * landBias;
                if (!TryHeight(terrain, landX, landY, out var landZ, out error))
                    return false;
                var landTarget = new Vec3(landX, landY, landZ);
                var drifted = HorizontalDistance(plants[leg], landTarget) > driftReplant;
                var overReach = HorizontalDistance(hipWorld, plants[leg]) > maxStanceReach;
                var stretched = drifted || overReach;
                var shouldSwing = phaseSwinging || stretched;
                Vec3 footWorld;

                if (!shouldSwing)
                {
                    footWorld = plants[leg];
                    legSwingPhase[leg] = -1.0;
                    stanceContacts.Add(footWorld);
                }
                else
                {
                    if (legSwingPhase[leg] < 0)
                    {
                        swingFrom[leg] = plants[leg];
                        swingTo[leg] = landTarget;
                        legSwingPhase[leg] = phaseSwinging ? phaseLocal : 0.0;
                    }
                    else
                    {
                        swingTo[leg] = landTarget;
                        if (phaseSwinging)
                            legSwingPhase[leg] = phaseLocal;
                        else
                            legSwingPhase[leg] = Math.Min(1.0, legSwingPhase[leg] + dt / swingPeriodSec);
                    }

                    var local = legSwingPhase[leg];
                    footWorld = SwingFoot(swingFrom[leg], swingTo[leg], local, stepHeight);
                    if (local >= 0.999)
                    {
                        plants[leg] = swingTo[leg];
                        if (!phaseSwinging)
                            legSwingPhase[leg] = -1.0;
                    }
                }

                var footBody = WorldToBody(footWorld, baseFrame);
                var hipBody = HipBody(layout, leg);

                if (LegIk3R.TrySolve(hipBody, footBody, layout.Coxa, layout.Femur, layout.Tibia, out var q0, out var q1, out var q2))
                {
                    q[leg * 3 + 0] = q0;
                    q[leg * 3 + 1] = q1;
                    q[leg * 3 + 2] = q2;
                }
                else
                {
                    q[leg * 3 + 0] = qPrev[leg * 3 + 0];
                    q[leg * 3 + 1] = qPrev[leg * 3 + 1];
                    q[leg * 3 + 2] = qPrev[leg * 3 + 2];
                    ikFailSamples++;
                }
            }

            // McGhee–Frank SSM when ≥3 stance contacts (skip all-swing / degenerate samples).
            // CoM stand-in = body origin XY (heuristic geometric CoM — labeled in Status).
            // Projection ignores non-coplanar contacts (classic SSM limit — labeled).
            if (stanceContacts.Count >= 3)
            {
                var ssm = StaticStability.Evaluate(stanceContacts, new Vec3(px, py, bodyGroundZ));
                if (double.IsFinite(ssm.MarginMeters) && ssm.MarginMeters < minSsm)
                    minSsm = ssm.MarginMeters;
                if (!ssm.IsStable)
                    unstableSamples++;
            }

            if (!AllFinite(q))
            {
                error = "Gait sample produced non-finite joint values (NaN/Inf).";
                return false;
            }

            qPrev = q;
            points.Add(new TrajectoryPoint(tSec, new JointState(q)));
            basePath.Add(baseFrame);
        }

        if (double.IsPositiveInfinity(minSsm))
            minSsm = double.NaN;

        var warnParts = new List<string>();
        if (ikFailSamples > 0)
            warnParts.Add($"Foot-target IK failed on {ikFailSamples} leg×sample(s); held previous q (rad).");
        if (unstableSamples > 0)
            warnParts.Add($"McGhee–Frank SSM unstable on {unstableSamples}/{sampleCount} samples (min margin {minSsm:F4} m; CoM≈body XY heuristic).");
        warnParts.Add("Preview gait only — Trajectory → Preview; not Motus Plan.");
        warnParts.Add(LeggedMethodRefs.DescribeStack());

        result = new Result(
            new Trajectory(model, points),
            basePath,
            string.Join(" ", warnParts),
            minSsm,
            LeggedMethodRefs.DescribeStack());
        return true;
    }

    /// <summary>
    /// Stance joint vector: coxa heading = hip yaw ± <paramref name="hipStance"/>,
    /// femur/tibia from analytic plant IK at Z=0 (meters). Fixed <paramref name="femurStance"/> /
    /// <paramref name="tibiaStance"/> are fallbacks only if plant IK fails.
    /// </summary>
    public static double[] BuildStanceQ(
        LeggedLayout layout, double hipStance, double femurStance, double tibiaStance)
    {
        var n = layout.LegCount;
        var q = new double[n * 3];
        var distal = layout.Femur + layout.Tibia;
        // Mid-workspace plant reach (horizontal). Elbow-up IK keeps the knee high.
        var planar = Math.Sqrt(Math.Max(0.0, distal * distal - layout.BodyZ * layout.BodyZ));
        var plantFromHip = layout.Coxa + 0.70 * planar;

        for (var leg = 0; leg < n; leg++)
        {
            var side = layout.LegIsLeft(leg) ? 1.0 : -1.0;
            var heading = layout.HipYawsRad[leg] + side * hipStance;
            var hip = HipBody(layout, leg);
            var foot = new Vec3(
                hip.X + plantFromHip * Math.Cos(heading),
                hip.Y + plantFromHip * Math.Sin(heading),
                0);

            if (LegIk3R.TrySolve(
                    hip, foot, layout.Coxa, layout.Femur, layout.Tibia,
                    out var q0, out var q1, out var q2))
            {
                q[leg * 3 + 0] = q0;
                q[leg * 3 + 1] = q1;
                q[leg * 3 + 2] = q2;
            }
            else
            {
                // ponytail: fixed angles only if plant IK fails (e.g. BodyZ > femur+tibia).
                q[leg * 3 + 0] = heading;
                q[leg * 3 + 1] = femurStance;
                q[leg * 3 + 2] = tibiaStance;
            }
        }

        return q;
    }

    private static bool TryBuildPolyline(
        IReadOnlyList<Vec3> pathXy, out double[] cumLen, out double pathLength, out string error)
    {
        error = "";
        cumLen = new double[pathXy.Count];
        pathLength = 0;
        cumLen[0] = 0;
        for (var i = 1; i < pathXy.Count; i++)
        {
            var a = pathXy[i - 1];
            var b = pathXy[i];
            if (!a.IsFinite || !b.IsFinite)
            {
                error = "Path points must be finite (m).";
                return false;
            }
            var dx = b.X - a.X;
            var dy = b.Y - a.Y;
            pathLength += Math.Sqrt(dx * dx + dy * dy);
            cumLen[i] = pathLength;
        }

        if (pathLength < 1e-9)
        {
            error = "Path length is zero.";
            return false;
        }

        return true;
    }

    private static void SampleAt(
        IReadOnlyList<Vec3> pathXy, double[] cumLen, double pathLength, double arcLen,
        out double x, out double y, out double yaw)
    {
        arcLen = Math.Clamp(arcLen, 0, pathLength);
        var seg = 1;
        while (seg < cumLen.Length && cumLen[seg] < arcLen - 1e-12)
            seg++;
        if (seg >= cumLen.Length)
            seg = cumLen.Length - 1;

        var a = pathXy[seg - 1];
        var b = pathXy[seg];
        var segLen = cumLen[seg] - cumLen[seg - 1];
        var u = segLen > 1e-12 ? (arcLen - cumLen[seg - 1]) / segLen : 1.0;
        x = a.X + (b.X - a.X) * u;
        y = a.Y + (b.Y - a.Y) * u;
        var tx = b.X - a.X;
        var ty = b.Y - a.Y;
        if (tx * tx + ty * ty < 1e-16)
        {
            // Degenerate segment — look ahead/back for tangent.
            for (var i = seg; i < pathXy.Count; i++)
            {
                tx = pathXy[i].X - a.X;
                ty = pathXy[i].Y - a.Y;
                if (tx * tx + ty * ty > 1e-16) break;
            }
            if (tx * tx + ty * ty < 1e-16)
            {
                tx = 1; ty = 0;
            }
        }
        yaw = Math.Atan2(ty, tx);
    }

    private static Vec3[]? InitializePlants(
        LeggedLayout layout,
        Frame startBase,
        double[] stanceQ,
        TerrainHeight terrain,
        out Vec3[] nominalFootBody,
        out string error)
    {
        error = "";
        var n = layout.LegCount;
        nominalFootBody = new Vec3[n];
        var plants = new Vec3[n];
        for (var leg = 0; leg < n; leg++)
        {
            var hipBody = HipBody(layout, leg);
            var footBody = LegIk3R.FootPosition(
                hipBody, layout.Coxa, layout.Femur, layout.Tibia,
                stanceQ[leg * 3 + 0], stanceQ[leg * 3 + 1], stanceQ[leg * 3 + 2]);
            // Body-floor plant (Z=0 relative); world Z comes from terrain under that XY.
            var footTargetBody = new Vec3(footBody.X, footBody.Y, 0);
            nominalFootBody[leg] = footTargetBody;

            if (!LegIk3R.TrySolve(hipBody, footTargetBody, layout.Coxa, layout.Femur, layout.Tibia, out _, out _, out _))
            {
                error = $"Leg {layout.LegNames[leg]}: stance foot unreachable (BodyZ={layout.BodyZ:F3} m too low or geometry infeasible).";
                return null;
            }

            var xy = BodyToWorld(footTargetBody, startBase);
            if (!TryHeight(terrain, xy.X, xy.Y, out var gz, out error))
                return null;
            plants[leg] = new Vec3(xy.X, xy.Y, gz);
        }

        return plants;
    }

    private static Vec3 HipWorld(LeggedLayout layout, int leg, Frame baseFrame) =>
        BodyToWorld(HipBody(layout, leg), baseFrame);

    private static bool TryNominalFootWorld(
        int leg, Vec3[] nominalFootBody, Frame baseFrame, TerrainHeight terrain,
        out Vec3 world, out string error)
    {
        var w = BodyToWorld(nominalFootBody[leg], baseFrame);
        if (!TryHeight(terrain, w.X, w.Y, out var gz, out error))
        {
            world = default;
            return false;
        }
        world = new Vec3(w.X, w.Y, gz);
        return true;
    }

    private static bool TryHeight(TerrainHeight terrain, double x, double y, out double z, out string error)
    {
        z = terrain(x, y);
        if (!double.IsFinite(z))
        {
            error = $"Terrain height non-finite at ({x:F3}, {y:F3}) m.";
            return false;
        }
        error = "";
        return true;
    }

    private static double HorizontalDistance(Vec3 a, Vec3 b)
    {
        var dx = a.X - b.X;
        var dy = a.Y - b.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    private static Vec3 HipBody(LeggedLayout layout, int leg)
    {
        var yaw = layout.HipYawsRad[leg];
        return new Vec3(layout.BodyR * Math.Cos(yaw), layout.BodyR * Math.Sin(yaw), layout.BodyZ);
    }

    private static Vec3 WorldToBody(Vec3 world, Frame baseFrame)
    {
        var yaw = YawFromFrame(baseFrame);
        var dx = world.X - baseFrame.X;
        var dy = world.Y - baseFrame.Y;
        var c = Math.Cos(-yaw);
        var s = Math.Sin(-yaw);
        return new Vec3(c * dx - s * dy, s * dx + c * dy, world.Z - baseFrame.Z);
    }

    private static Vec3 BodyToWorld(Vec3 body, Frame baseFrame)
    {
        var yaw = YawFromFrame(baseFrame);
        var c = Math.Cos(yaw);
        var s = Math.Sin(yaw);
        return new Vec3(
            baseFrame.X + c * body.X - s * body.Y,
            baseFrame.Y + s * body.X + c * body.Y,
            baseFrame.Z + body.Z);
    }

    private static double YawFromFrame(Frame f) => 2.0 * Math.Atan2(f.Qz, f.Qw);

    /// <summary>
    /// Swing path: linear XY blend + sinusoidal lift (engineering heuristic, not from FABRIK/McGhee).
    /// </summary>
    private static Vec3 SwingFoot(Vec3 start, Vec3 end, double phase01, double liftMeters)
    {
        var t = Math.Clamp(phase01, 0, 1);
        var x = start.X + (end.X - start.X) * t;
        var y = start.Y + (end.Y - start.Y) * t;
        // Heuristic clearance above lerped terrain: z = lerp + H·sin(πs).
        var zGround = start.Z + (end.Z - start.Z) * t;
        var z = zGround + (liftMeters > 0 ? liftMeters * Math.Sin(t * Math.PI) : 0);
        return new Vec3(x, y, z);
    }

    /// <summary>
    /// Periodic swing window for leg's group — Song &amp; Waldron-style duty phasing (DOI in <see cref="LeggedMethodRefs"/>).
    /// cyclePhase ∈ [0,1); group g of G swings in [g/G,(g+1)/G).
    /// </summary>
    private static (bool Swinging, double LocalPhase01) LegSwingPhase(
        LeggedLayout layout, int leg, double cyclePhase01)
    {
        var groupCount = layout.SwingGroups.Count;
        var groupIndex = -1;
        for (var g = 0; g < groupCount; g++)
        {
            if (Array.IndexOf(layout.SwingGroups[g], leg) >= 0)
            {
                groupIndex = g;
                break;
            }
        }

        if (groupIndex < 0)
            return (false, 0);

        var slot = 1.0 / groupCount;
        var start = groupIndex * slot;
        var end = start + slot;
        var swinging = cyclePhase01 >= start && (groupIndex == groupCount - 1 || cyclePhase01 < end);
        if (!swinging) return (false, 0);

        var local = (cyclePhase01 - start) / slot;
        return (true, Math.Clamp(local, 0, 1));
    }

    private static bool AllFinite(IReadOnlyList<double> v)
    {
        foreach (var x in v)
            if (!double.IsFinite(x)) return false;
        return true;
    }
}
