using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Foot-target gait: duty-factor swing schedule, planted stance contacts, per-leg <see cref="ILegIkSolver"/>.
/// Preview / TreeFK only — not Motus Plan. Path is a polyline in XY (arc-length); optional terrain height at (x,y).
/// </summary>
/// <remarks>
/// <para><b>Gait timing:</b> Song &amp; Waldron DOI <see cref="LeggedMethodRefs.SongWaldron1987Doi"/> via
/// <see cref="GaitSchedule"/>.</para>
/// <para><b>Stance plants:</b> McGhee &amp; Frank DOI <see cref="LeggedMethodRefs.McGheeFrank1968Doi"/>;
/// SSM via <see cref="StaticStability"/>.</para>
/// <para><b>IK:</b> <see cref="ILegIkSolver"/> (3R → <see cref="LegIk3R"/>).</para>
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
        /// <summary>Samples where stance contact count &lt; 3 (degenerate support polygon).</summary>
        public int DegenerateSupportSamples { get; }

        public Result(
            Trajectory trajectory,
            IReadOnlyList<Frame> basePath,
            string? warning,
            double minStaticStabilityMarginMeters,
            string methodProvenance,
            int degenerateSupportSamples = 0)
        {
            Trajectory = trajectory;
            BasePath = basePath;
            Warning = warning;
            MinStaticStabilityMarginMeters = minStaticStabilityMarginMeters;
            MethodProvenance = methodProvenance;
            DegenerateSupportSamples = degenerateSupportSamples;
        }
    }

    /// <summary>
        /// Legacy layout adapter → <see cref="LeggedLayout.ToMechanism"/> + TerrainSupport (matches prior body plane).
    /// </summary>
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
        if (layout.Validate() is { } layoutErr)
        {
            error = layoutErr;
            return false;
        }

        var mechanism = layout.ToMechanism();
        // Legacy body: origin on support plane (clearance offset 0); hip Z lives in HipInBody.
        IBodyPoseSolver body = new TerrainSupportBodyPose(clearanceMeters: 0);
        return TryBuild(
            mechanism, body, pathXy, speed, stepLength, stepHeight,
            hipStance, femurStance, tibiaStance, model, out result, out error, terrain);
    }

    public static bool TryBuild(
        LeggedMechanism mechanism,
        IBodyPoseSolver? bodyPose,
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
        terrain ??= static (_, _) => 0;
        bodyPose ??= new TerrainSupportBodyPose(clearanceMeters: 0);

        if (mechanism.Validate() is { } mechErr)
        {
            error = mechErr;
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

        var n = mechanism.LegCount;
        var dof = mechanism.DriverCount;
        if (model.Preset.AxisCount != dof)
        {
            error = $"RobotModel AxisCount ({model.Preset.AxisCount}) must equal mechanism drivers ({dof}).";
            return false;
        }

        SampleAt(pathXy, cumLen, pathLength, 0, out var startX, out var startY, out var startYaw);
        var stanceQ = BuildStanceQ(mechanism, hipStance, femurStance, tibiaStance);
        if (!TryNominalFeetBody(mechanism, stanceQ, out var nominalFootBody, out error))
            return false;

        var hips = new Vec3[n];
        for (var leg = 0; leg < n; leg++)
            hips[leg] = mechanism.HipBody(leg);

        var poseSession = bodyPose.CreateSession();
        if (!poseSession.TryPose(
                startX, startY, startYaw, nominalFootBody, hips, terrain.Invoke,
                isFirstSample: true, out var startFrame, out error))
            return false;

        var plants = InitializePlants(nominalFootBody, startFrame, terrain, out var initErr);
        if (plants is null)
        {
            error = initErr;
            return false;
        }

        var duration = pathLength / speed;
        // Swing-resolved sample rate: denser when swing windows are short.
        var groupCount = Math.Max(1, mechanism.Gait.SwingGroups?.Count ??
            (int)Math.Round(1.0 / Math.Max(1e-6, 1.0 - mechanism.Gait.DutyFactor)));
        var sampleHz = Math.Clamp(30.0 * Math.Max(1.0, groupCount / 2.0), 30.0, 60.0);
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

        var maxStanceReach = EstimateMaxStanceReach(mechanism);
        var driftReplant = Math.Max(0.02, 0.55 * stepLength);
        var swingPeriodSec = duration / (cyclesPerPath * groupCount);
        var landBias = 0.30 * stepLength;

        var qPrev = (double[])stanceQ.Clone();
        var legSwingPhase = new double[n];
        Array.Fill(legSwingPhase, -1.0);
        var swingFrom = new Vec3[n];
        var swingTo = new Vec3[n];
        var footWorld = new Vec3[n];
        var ikFailSamples = 0;
        var minSsm = double.PositiveInfinity;
        var unstableSamples = 0;
        var degenerateSupportSamples = 0;
        var gait = mechanism.Gait;
        var offsets = mechanism.DriverOffsets;

        for (var i = 0; i < sampleCount; i++)
        {
            var tSec = Math.Min(duration, i * dt);
            var arcLen = speed * tSec;
            SampleAt(pathXy, cumLen, pathLength, arcLen, out var px, out var py, out var yaw);

            Frame baseFrame;
            if (i == 0)
            {
                baseFrame = startFrame;
            }
            else if (!poseSession.TryPose(
                    px, py, yaw, nominalFootBody, hips, terrain.Invoke,
                    isFirstSample: false, out baseFrame, out error))
            {
                return false;
            }

            var pathPhase = duration > 1e-9 ? tSec / duration : 0;
            var cyclePhase = (pathPhase * cyclesPerPath) % 1.0;
            var headingX = Math.Cos(yaw);
            var headingY = Math.Sin(yaw);
            var stanceContacts = new List<Vec3>(n);
            var stanceMask = new bool[n];

            // Scheduled swing windows (Song–Waldron).
            var phaseSwing = new bool[n];
            var phaseLocalArr = new double[n];
            for (var leg = 0; leg < n; leg++)
                phaseSwing[leg] = gait.IsSwinging(leg, cyclePhase, out phaseLocalArr[leg]);

            for (var leg = 0; leg < n; leg++)
            {
                var phaseSwinging = phaseSwing[leg];
                var phaseLocal = phaseLocalArr[leg];
                var hipWorld = BodyToWorld(hips[leg], baseFrame);
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
                // ponytail: free stretch-replant (legacy). DegenerateSupportSamples tracks MinStance dips.
                var shouldSwing = phaseSwinging || stretched;

                if (!shouldSwing)
                {
                    footWorld[leg] = plants[leg];
                    legSwingPhase[leg] = -1.0;
                    stanceMask[leg] = true;
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
                var hipBody = hips[leg];
                var off = offsets[leg];
                var dofLeg = mechanism.Legs[leg].DriverDof;

                if (mechanism.Legs[leg].Ik.TrySolve(
                        hipBody, footBody, FootTargetKind.Position, out var qLeg, out _))
                {
                    for (var j = 0; j < dofLeg && j < qLeg.Length; j++)
                        q[off + j] = qLeg[j];
                }
                else
                {
                    for (var j = 0; j < dofLeg; j++)
                        q[off + j] = qPrev[off + j];
                    ikFailSamples++;
                    stanceMask[leg] = false;
                }
            }

            stanceContacts.Clear();
            for (var leg = 0; leg < n; leg++)
            {
                if (stanceMask[leg])
                    stanceContacts.Add(footWorld[leg]);
            }

            if (stanceContacts.Count < 3)
                degenerateSupportSamples++;

            if (stanceContacts.Count >= 3)
            {
                // SSM CoM: for high-β crawl, body XY sits on the parallelogram diagonal (margin 0).
                // Blend toward stance-contact centroid (McGhee–Frank CoM-in-triangle heuristic).
                double comX = px, comY = py;
                var beta = gait.DutyFactor;
                if (beta > 0.5 + 1e-9)
                {
                    double sx = 0, sy = 0;
                    foreach (var c in stanceContacts)
                    {
                        sx += c.X;
                        sy += c.Y;
                    }
                    var inv = 1.0 / stanceContacts.Count;
                    var alpha = Math.Clamp(2.0 * (beta - 0.5), 0, 1);
                    comX = px + alpha * (sx * inv - px);
                    comY = py + alpha * (sy * inv - py);
                }

                var ssm = StaticStability.Evaluate(stanceContacts, new Vec3(comX, comY, baseFrame.Z));
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
        if (mechanism.AllowDynamicGait)
            warnParts.Add($"AllowDynamicGait: MinStanceCount={gait.MinStanceCount} (dynamic gait; SSM may be weak).");
        if (ikFailSamples > 0)
            warnParts.Add($"Foot-target IK failed on {ikFailSamples} leg×sample(s); held previous q (rad); excluded from support.");
        if (degenerateSupportSamples > 0)
            warnParts.Add($"Degenerate support (<3 stance) on {degenerateSupportSamples}/{sampleCount} samples.");
        if (unstableSamples > 0)
            warnParts.Add($"SSM unstable on {unstableSamples}/{sampleCount} samples (min margin {minSsm:F4} m).");
        warnParts.Add($"Gait β={gait.DutyFactor:F3} MethodId={gait.MethodId}; body={bodyPose.MethodId}.");
        warnParts.Add("Preview gait only — Trajectory → Preview; not Motus Plan.");

        result = new Result(
            new Trajectory(model, points),
            basePath,
            string.Join(" ", warnParts),
            minSsm,
            LeggedMethodRefs.DescribeStack(),
            degenerateSupportSamples);
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
                    $"required>={minStaticStabilityMarginMeters:F4} m.",
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

        var warnings = new List<string>();
        if (!string.IsNullOrWhiteSpace(gait.Warning))
            warnings.Add(gait.Warning);
        warnings.Add("LeggedGait.ValidateForPlan: adapter-only validation; gait generation unchanged.");
        return PlanningResult.Succeeded(gait.Trajectory, warnings);
    }

    public static double[] BuildStanceQ(
        LeggedLayout layout, double hipStance, double femurStance, double tibiaStance) =>
        BuildStanceQ(layout.ToMechanism(), hipStance, femurStance, tibiaStance);

    public static double[] BuildStanceQ(
        LeggedMechanism mechanism, double hipStance, double femurStance, double tibiaStance)
    {
        var q = new double[mechanism.DriverCount];
        for (var leg = 0; leg < mechanism.LegCount; leg++)
        {
            var hip = mechanism.HipBody(leg);
            var side = mechanism.LegIsLeft(leg) ? 1.0 : -1.0;
            var yaw = mechanism.HipYawRad(leg);
            var off = mechanism.DriverOffsets[leg];
            if (!mechanism.Legs[leg].Ik.TryNominalStance(
                    hip, yaw, side, hipStance, femurStance, tibiaStance,
                    mechanism.NominalBodyClearance,
                    out var qLeg, out _, out _))
            {
                continue;
            }

            var dof = mechanism.Legs[leg].DriverDof;
            for (var j = 0; j < dof && j < qLeg.Length; j++)
                q[off + j] = qLeg[j];
        }

        return q;
    }

    private static double EstimateMaxStanceReach(LeggedMechanism mechanism)
    {
        var maxHoriz = 0.0;
        for (var leg = 0; leg < mechanism.LegCount; leg++)
        {
            if (mechanism.Legs[leg].Ik is LegIk3RSolver s)
            {
                var distal = s.Femur + s.Tibia;
                var bz = mechanism.NominalBodyClearance;
                var h = s.Coxa + Math.Sqrt(Math.Max(0, distal * distal - bz * bz));
                if (h > maxHoriz) maxHoriz = h;
            }
            else
            {
                var w = mechanism.Legs[leg].Ik.Workspace;
                if (w.MaxReachMeters > maxHoriz) maxHoriz = w.MaxReachMeters;
            }
        }

        return 1.12 * Math.Max(maxHoriz, 1e-3);
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
        LeggedMechanism mechanism,
        double[] stanceQ,
        out Vec3[] nominalFootBody,
        out string error)
    {
        error = "";
        var n = mechanism.LegCount;
        nominalFootBody = new Vec3[n];
        for (var leg = 0; leg < n; leg++)
        {
            var hipBody = mechanism.HipBody(leg);
            var off = mechanism.DriverOffsets[leg];
            if (mechanism.Legs[leg].Ik is not LegIk3RSolver s3)
            {
                error = $"Leg {mechanism.Legs[leg].Name}: nominal foot requires LegIk3RSolver in v1.";
                return false;
            }

            var footBody = s3.FootPosition(
                hipBody, stanceQ[off], stanceQ[off + 1], stanceQ[off + 2]);
            var footTargetBody = new Vec3(footBody.X, footBody.Y, 0);
            if (!s3.TrySolve(hipBody, footTargetBody, FootTargetKind.Position, out _, out _))
            {
                error = $"Leg {mechanism.Legs[leg].Name}: stance foot unreachable (clearance={mechanism.NominalBodyClearance:F3} m too low or geometry infeasible).";
                return false;
            }
            nominalFootBody[leg] = footTargetBody;
        }
        return true;
    }

    private static Vec3[]? InitializePlants(
        Vec3[] nominalFootBody,
        Frame startBase,
        TerrainHeight terrain,
        out string error)
    {
        error = "";
        var n = nominalFootBody.Length;
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

    private static Vec3 SwingFoot(Vec3 start, Vec3 end, double phase01, double liftMeters)
    {
        var t = Math.Clamp(phase01, 0, 1);
        var x = start.X + (end.X - start.X) * t;
        var y = start.Y + (end.Y - start.Y) * t;
        var zGround = start.Z + (end.Z - start.Z) * t;
        var z = zGround + (liftMeters > 0 ? liftMeters * Math.Sin(t * Math.PI) : 0);
        return new Vec3(x, y, z);
    }

    private static bool AllFinite(IReadOnlyList<double> v)
    {
        foreach (var x in v)
            if (!double.IsFinite(x)) return false;
        return true;
    }
}
