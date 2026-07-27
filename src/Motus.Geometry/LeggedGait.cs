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
/// body rides a <b>smoothed</b> support plane from terrain under all nominal feet (not the
/// instantaneous stance set — that jumps when legs lift and looks like 2-leg climbing).</para>
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

        SampleAt(pathXy, cumLen, pathLength, 0, out var startX, out var startY, out var startYaw);
        var stanceQ = BuildStanceQ(layout, hipStance, femurStance, tibiaStance);
        if (!TryNominalFeetBody(layout, stanceQ, out var nominalFootBody, out error))
            return false;
        if (!TryFrameFromNominalTerrain(
                layout, nominalFootBody, startX, startY, startYaw, terrain, out var startFrame, out error))
            return false;

        var plants = InitializePlants(layout, startFrame, nominalFootBody, terrain, out var initErr);
        if (plants is null)
        {
            error = initErr;
            return false;
        }

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

        // Planar hip–foot over-reach (foot near body floor): coxa + sqrt((femur+tibia)² − BodyZ²).
        var distal = layout.Femur + layout.Tibia;
        var maxHoriz = layout.Coxa + Math.Sqrt(Math.Max(0, distal * distal - layout.BodyZ * layout.BodyZ));
        // Loose enough that hills don't cascade stretch-swings; tight enough that plants still replant.
        var maxStanceReach = 1.12 * maxHoriz;
        var driftReplant = Math.Max(0.02, 0.55 * stepLength);
        var groupCount = layout.SwingGroups.Count;
        var swingPeriodSec = duration / (cyclesPerPath * groupCount);
        var landBias = 0.30 * stepLength;
        // Body slew: ~0.25 m/s vertical + soft slerp — kills stance-set discontinuities.
        var maxDzPerSample = Math.Max(0.004, 0.25 * dt);
        const double bodyBlend = 0.28;

        var qPrev = (double[])stanceQ.Clone();
        var legSwingPhase = new double[n];
        Array.Fill(legSwingPhase, -1.0);
        var swingFrom = new Vec3[n];
        var swingTo = new Vec3[n];
        var footWorld = new Vec3[n];
        var ikFailSamples = 0;
        var minSsm = double.PositiveInfinity;
        var unstableSamples = 0;
        var prevBase = startFrame;

        for (var i = 0; i < sampleCount; i++)
        {
            var tSec = Math.Min(duration, i * dt);
            var arcLen = speed * tSec;
            SampleAt(pathXy, cumLen, pathLength, arcLen, out var px, out var py, out var yaw);

            // Continuous desired pose from all 6 nominal feet on terrain (stable as legs swing).
            if (!TryFrameFromNominalTerrain(
                    layout, nominalFootBody, px, py, yaw, terrain, out var desiredBase, out error))
                return false;
            var baseFrame = i == 0
                ? desiredBase
                : SmoothBodyToward(prevBase, desiredBase, px, py, yaw, bodyBlend, maxDzPerSample);

            var pathPhase = duration > 1e-9 ? tSec / duration : 0;
            var cyclePhase = (pathPhase * cyclesPerPath) % 1.0;
            var headingX = Math.Cos(yaw);
            var headingY = Math.Sin(yaw);
            var stanceContacts = new List<Vec3>(n);

            for (var leg = 0; leg < n; leg++)
            {
                var (phaseSwinging, phaseLocal) = LegSwingPhase(layout, leg, cyclePhase);
                var hipWorld = HipWorld(layout, leg, baseFrame);
                if (!TryNominalFootWorld(leg, nominalFootBody, baseFrame, terrain, out var nominalLand, out error))
                    return false;
                var landX = nominalLand.X + headingX * landBias;
                var landY = nominalLand.Y + headingY * landBias;
                if (!TryHeight(terrain, landX, landY, out var landZ, out error))
                    return false;
                var landTarget = new Vec3(landX, landY, landZ);
                var hipDist = HorizontalDistance(hipWorld, plants[leg]);
                var drifted = HorizontalDistance(plants[leg], landTarget) > driftReplant;
                var overReach = hipDist > maxStanceReach;
                var stretched = drifted || overReach;
                var shouldSwing = phaseSwinging || stretched;

                if (!shouldSwing)
                {
                    footWorld[leg] = plants[leg];
                    legSwingPhase[leg] = -1.0;
                    stanceContacts.Add(footWorld[leg]);
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
                    footWorld[leg] = SwingFoot(swingFrom[leg], swingTo[leg], local, stepHeight);
                    if (local >= 0.999)
                    {
                        plants[leg] = swingTo[leg];
                        if (!phaseSwinging)
                            legSwingPhase[leg] = -1.0;
                    }
                }
            }

            var q = (double[])qPrev.Clone();
            for (var leg = 0; leg < n; leg++)
            {
                var footBody = WorldToBody(footWorld[leg], baseFrame);
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

            if (stanceContacts.Count >= 3)
            {
                var ssm = StaticStability.Evaluate(stanceContacts, new Vec3(px, py, baseFrame.Z));
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
            prevBase = baseFrame;
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
    /// Adapter-only Plan gate for an already-built legged gait trajectory. Does not synthesize or modify gait;
    /// it validates SSM and optional collision, then returns a <see cref="PlanningResult"/> for shared Status UI.
    /// </summary>
    public static PlanningResult ValidateForPlan(
        Result gait,
        PlanningOptions? options = null,
        double minStaticStabilityMarginMeters = 0.0)
    {
        if (gait is null)
        {
            return PlanningResult.Failed(new[]
            {
                new PlanningMessage(
                    PlanningMessageCodes.InvalidInput,
                    "Legged gait result is null.",
                    PlanningMessageSeverity.Error)
            });
        }

        if (!Units.IsLegged(gait.Trajectory.Robot.Preset))
        {
            return PlanningResult.Failed(new[]
            {
                new PlanningMessage(
                    PlanningMessageCodes.InvalidOptions,
                    $"Legged Plan adapter requires RobotPreset.Family='{Units.LeggedFamily}'.",
                    PlanningMessageSeverity.Error)
            });
        }

        if (!double.IsFinite(gait.MinStaticStabilityMarginMeters) ||
            gait.MinStaticStabilityMarginMeters < minStaticStabilityMarginMeters)
        {
            return PlanningResult.Failed(new[]
            {
                new PlanningMessage(
                    PlanningMessageCodes.ConstraintViolation,
                    $"Legged SSM below threshold: min={gait.MinStaticStabilityMarginMeters:F4} m, " +
                    $"required>={minStaticStabilityMarginMeters:F4} m (McGhee&Frank doi:{LeggedMethodRefs.McGheeFrank1968Doi}).",
                    PlanningMessageSeverity.Error)
            });
        }

        options ??= new PlanningOptions();
        if (options.CollisionChecker is not null && options.CollisionScene is not null)
        {
            var collisionFail = PlanningCollision.ValidateTrajectory(
                gait.Trajectory,
                options.CollisionScene,
                options.CollisionChecker,
                options.MaxJointStepRadians);
            if (collisionFail is not null) return collisionFail;
        }

        var warnings = new List<string> { gait.MethodProvenance };
        if (!string.IsNullOrWhiteSpace(gait.Warning))
            warnings.Add(gait.Warning);
        warnings.Add("LeggedGait.ValidateForPlan: adapter-only validation; gait generation unchanged.");
        return PlanningResult.Succeeded(gait.Trajectory, warnings);
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

    private static bool TryNominalFeetBody(
        LeggedLayout layout,
        double[] stanceQ,
        out Vec3[] nominalFootBody,
        out string error)
    {
        error = "";
        var n = layout.LegCount;
        nominalFootBody = new Vec3[n];
        for (var leg = 0; leg < n; leg++)
        {
            var hipBody = HipBody(layout, leg);
            var footBody = LegIk3R.FootPosition(
                hipBody, layout.Coxa, layout.Femur, layout.Tibia,
                stanceQ[leg * 3 + 0], stanceQ[leg * 3 + 1], stanceQ[leg * 3 + 2]);
            // Body-floor plant (Z=0 relative); world Z comes from terrain under that XY.
            var footTargetBody = new Vec3(footBody.X, footBody.Y, 0);
            if (!LegIk3R.TrySolve(hipBody, footTargetBody, layout.Coxa, layout.Femur, layout.Tibia, out _, out _, out _))
            {
                error = $"Leg {layout.LegNames[leg]}: stance foot unreachable (BodyZ={layout.BodyZ:F3} m too low or geometry infeasible).";
                return false;
            }
            nominalFootBody[leg] = footTargetBody;
        }
        return true;
    }

    /// <summary>
    /// Vertical projection of the walk pose onto terrain: body base Z = mean ground under nominal feet.
    /// Keeps relative foot Z near body-floor so LegIk3R stays in workspace on slopes / Amp hills.
    /// </summary>
    private static bool TrySupportPlaneZ(
        LeggedLayout layout,
        Vec3[] nominalFootBody,
        double px,
        double py,
        double yaw,
        TerrainHeight terrain,
        out double bodyGroundZ,
        out string error)
    {
        bodyGroundZ = 0;
        var flat = new MobilityModel.HolonomicSE2(px, py, yaw, 0).BaseFrame;
        var sum = 0.0;
        var n = layout.LegCount;
        for (var leg = 0; leg < n; leg++)
        {
            var w = BodyToWorld(nominalFootBody[leg], flat);
            if (!TryHeight(terrain, w.X, w.Y, out var gz, out error))
                return false;
            sum += gz;
        }
        bodyGroundZ = sum / n;
        error = "";
        return true;
    }

    private static Vec3[]? InitializePlants(
        LeggedLayout layout,
        Frame startBase,
        Vec3[] nominalFootBody,
        TerrainHeight terrain,
        out string error)
    {
        error = "";
        var n = layout.LegCount;
        var plants = new Vec3[n];
        for (var leg = 0; leg < n; leg++)
        {
            var xy = BodyToWorld(nominalFootBody[leg], startBase);
            if (!TryHeight(terrain, xy.X, xy.Y, out var gz, out error))
                return null;
            plants[leg] = new Vec3(xy.X, xy.Y, gz);
        }

        return plants;
    }

    /// <summary>
    /// Continuous body pose: plane through terrain under all nominal feet (stable while tripod swings).
    /// </summary>
    private static bool TryFrameFromNominalTerrain(
        LeggedLayout layout,
        Vec3[] nominalFootBody,
        double px,
        double py,
        double pathYaw,
        TerrainHeight terrain,
        out Frame frame,
        out string error)
    {
        frame = default;
        var flat = new MobilityModel.HolonomicSE2(px, py, pathYaw, 0).BaseFrame;
        var n = layout.LegCount;
        var pts = new Vec3[n];
        for (var leg = 0; leg < n; leg++)
        {
            var w = BodyToWorld(nominalFootBody[leg], flat);
            if (!TryHeight(terrain, w.X, w.Y, out var gz, out error))
                return false;
            pts[leg] = new Vec3(w.X, w.Y, gz);
        }

        frame = FrameFromSupportPoints(px, py, pathYaw, pts);
        error = "";
        return true;
    }

    private static Frame FrameFromSupportPoints(
        double px, double py, double pathYaw, IReadOnlyList<Vec3> pts)
    {
        if (pts.Count < 3 || !TryFitHeightPlane(pts, out var a, out var b, out var c))
        {
            var z = 0.0;
            for (var i = 0; i < pts.Count; i++)
                z += pts[i].Z;
            z = pts.Count > 0 ? z / pts.Count : 0;
            return new MobilityModel.HolonomicSE2(px, py, pathYaw, z).BaseFrame;
        }

        var zBody = a * px + b * py + c;
        if (!double.IsFinite(zBody))
            return new MobilityModel.HolonomicSE2(px, py, pathYaw, pts[0].Z).BaseFrame;

        var nx = -a;
        var ny = -b;
        var nz = 1.0;
        var nLen = Math.Sqrt(nx * nx + ny * ny + nz * nz);
        nx /= nLen;
        ny /= nLen;
        nz /= nLen;
        if (nz < 0.45)
            return new MobilityModel.HolonomicSE2(px, py, pathYaw, zBody).BaseFrame;

        var hx = Math.Cos(pathYaw);
        var hy = Math.Sin(pathYaw);
        var hd = hx * nx + hy * ny;
        var xx = hx - nx * hd;
        var xy = hy - ny * hd;
        var xz = -nz * hd;
        var xLen = Math.Sqrt(xx * xx + xy * xy + xz * xz);
        if (xLen < 1e-9)
            return new MobilityModel.HolonomicSE2(px, py, pathYaw, zBody).BaseFrame;
        xx /= xLen;
        xy /= xLen;
        xz /= xLen;

        var yx = ny * xz - nz * xy;
        var yy = nz * xx - nx * xz;
        var yz = nx * xy - ny * xx;

        return Transforms.ToFrame(
        [
            xx, yx, nx, px,
            xy, yy, ny, py,
            xz, yz, nz, zBody,
            0, 0, 0, 1
        ]);
    }

    /// <summary>EMA + vertical slew toward desired support pose (path yaw forced).</summary>
    private static Frame SmoothBodyToward(
        Frame prev, Frame desired, double px, double py, double pathYaw,
        double blend, double maxDz)
    {
        var z = prev.Z + Math.Clamp(desired.Z - prev.Z, -maxDz, maxDz);
        z += blend * (desired.Z - z);

        // Soft-slerp orientation, then rebuild with path yaw projected onto blended normal.
        var t = Math.Clamp(blend, 0, 1);
        SlerpQuat(
            prev.Qw, prev.Qx, prev.Qy, prev.Qz,
            desired.Qw, desired.Qx, desired.Qy, desired.Qz,
            t,
            out var qw, out var qx, out var qy, out var qz);
        var blended = new Frame(px, py, z, qw, qx, qy, qz);

        // Re-apply path yaw on the blended Z-up so heading stays on the walk path.
        var m = Transforms.FromFrame(blended);
        var nx = m[2];
        var ny = m[6];
        var nz = m[10];
        var nLen = Math.Sqrt(nx * nx + ny * ny + nz * nz);
        if (nLen < 1e-9 || nz / nLen < 0.45)
            return new MobilityModel.HolonomicSE2(px, py, pathYaw, z).BaseFrame;
        nx /= nLen;
        ny /= nLen;
        nz /= nLen;

        var hx = Math.Cos(pathYaw);
        var hy = Math.Sin(pathYaw);
        var hd = hx * nx + hy * ny;
        var xx = hx - nx * hd;
        var xy = hy - ny * hd;
        var xz = -nz * hd;
        var xLen = Math.Sqrt(xx * xx + xy * xy + xz * xz);
        if (xLen < 1e-9)
            return new MobilityModel.HolonomicSE2(px, py, pathYaw, z).BaseFrame;
        xx /= xLen;
        xy /= xLen;
        xz /= xLen;
        var yx = ny * xz - nz * xy;
        var yy = nz * xx - nx * xz;
        var yz = nx * xy - ny * xx;
        return Transforms.ToFrame(
        [
            xx, yx, nx, px,
            xy, yy, ny, py,
            xz, yz, nz, z,
            0, 0, 0, 1
        ]);
    }

    private static void SlerpQuat(
        double aw, double ax, double ay, double az,
        double bw, double bx, double by, double bz,
        double t,
        out double w, out double x, out double y, out double z)
    {
        var dot = aw * bw + ax * bx + ay * by + az * bz;
        if (dot < 0)
        {
            bw = -bw; bx = -bx; by = -by; bz = -bz;
            dot = -dot;
        }

        if (dot > 0.9995)
        {
            w = aw + t * (bw - aw);
            x = ax + t * (bx - ax);
            y = ay + t * (by - ay);
            z = az + t * (bz - az);
            var n = Math.Sqrt(w * w + x * x + y * y + z * z);
            if (n < 1e-15) { w = 1; x = y = z = 0; return; }
            w /= n; x /= n; y /= n; z /= n;
            return;
        }

        var theta0 = Math.Acos(Math.Clamp(dot, -1, 1));
        var theta = theta0 * t;
        var s0 = Math.Sin(theta0);
        var s1 = Math.Sin(theta0 - theta) / s0;
        var s2 = Math.Sin(theta) / s0;
        w = s1 * aw + s2 * bw;
        x = s1 * ax + s2 * bx;
        y = s1 * ay + s2 * by;
        z = s1 * az + s2 * bz;
    }

    /// <summary>Least-squares plane z = a x + b y + c through support points.</summary>
    private static bool TryFitHeightPlane(IReadOnlyList<Vec3> pts, out double a, out double b, out double c)
    {
        a = b = c = 0;
        var n = pts.Count;
        if (n < 3)
            return false;

        double sx = 0, sy = 0, sz = 0, sxx = 0, syy = 0, sxy = 0, sxz = 0, syz = 0;
        for (var i = 0; i < n; i++)
        {
            var p = pts[i];
            sx += p.X;
            sy += p.Y;
            sz += p.Z;
            sxx += p.X * p.X;
            syy += p.Y * p.Y;
            sxy += p.X * p.Y;
            sxz += p.X * p.Z;
            syz += p.Y * p.Z;
        }

        var m00 = sxx;
        var m01 = sxy;
        var m02 = sx;
        var m10 = sxy;
        var m11 = syy;
        var m12 = sy;
        var m20 = sx;
        var m21 = sy;
        var m22 = n;
        var d0 = sxz;
        var d1 = syz;
        var d2 = sz;

        var det =
            m00 * (m11 * m22 - m12 * m21) -
            m01 * (m10 * m22 - m12 * m20) +
            m02 * (m10 * m21 - m11 * m20);
        if (Math.Abs(det) < 1e-14)
            return false;

        a = (
            d0 * (m11 * m22 - m12 * m21) -
            m01 * (d1 * m22 - m12 * d2) +
            m02 * (d1 * m21 - m11 * d2)) / det;
        b = (
            m00 * (d1 * m22 - m12 * d2) -
            d0 * (m10 * m22 - m12 * m20) +
            m02 * (m10 * d2 - d1 * m20)) / det;
        c = (
            m00 * (m11 * d2 - d1 * m21) -
            m01 * (m10 * d2 - d1 * m20) +
            d0 * (m10 * m21 - m11 * m20)) / det;
        return double.IsFinite(a) && double.IsFinite(b) && double.IsFinite(c);
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
        var m = Transforms.FromFrame(baseFrame);
        var inv = Transforms.Inverse(m);
        Transforms.TransformPointInto(inv, world.X, world.Y, world.Z, out var x, out var y, out var z);
        return new Vec3(x, y, z);
    }

    private static Vec3 BodyToWorld(Vec3 body, Frame baseFrame)
    {
        var m = Transforms.FromFrame(baseFrame);
        Transforms.TransformPointInto(m, body.X, body.Y, body.Z, out var x, out var y, out var z);
        return new Vec3(x, y, z);
    }

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
