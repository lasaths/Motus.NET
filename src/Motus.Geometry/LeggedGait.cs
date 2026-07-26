using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Duty-cycle foot-target gait with planted feet + per-leg analytic IK (preview only — not Motus Plan).
/// Path is a planar polyline (Z ignored); samples along arc length at the given speed (m/s).
/// </summary>
public static class LeggedGait
{
    public sealed class Result
    {
        public Trajectory Trajectory { get; }
        public IReadOnlyList<Frame> BasePath { get; }
        public string? Warning { get; }

        public Result(Trajectory trajectory, IReadOnlyList<Frame> basePath, string? warning)
        {
            Trajectory = trajectory;
            BasePath = basePath;
            Warning = warning;
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
        out string error)
    {
        result = null;
        error = "";

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
            error = "Path empty — need ≥ 2 polyline points (m, Z ignored).";
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
        var startFrame = new MobilityModel.HolonomicSE2(startX, startY, startYaw).BaseFrame;

        var plants = InitializePlants(layout, startFrame, stanceQ, out var nominalFootBody, out var initErr);
        if (plants is null)
        {
            error = initErr;
            return false;
        }

        // ponytail: horizontal stretch cap before forced re-plant; full FABRIK if gaits need more
        var maxStanceReach = 0.85 * (layout.Coxa + layout.Femur + layout.Tibia);
        var groupCount = layout.SwingGroups.Count;
        var swingPeriodSec = duration / (cyclesPerPath * groupCount);

        var qPrev = (double[])stanceQ.Clone();
        var legSwingPhase = new double[n];
        Array.Fill(legSwingPhase, -1.0);
        var swingFrom = new Vec3[n];
        var swingTo = new Vec3[n];
        var ikFailSamples = 0;
        string? ikWarning = null;

        for (var i = 0; i < sampleCount; i++)
        {
            var tSec = Math.Min(duration, i * dt);
            var arcLen = speed * tSec;
            SampleAt(pathXy, cumLen, pathLength, arcLen, out var px, out var py, out var yaw);
            var baseFrame = new MobilityModel.HolonomicSE2(px, py, yaw).BaseFrame;

            var pathPhase = duration > 1e-9 ? tSec / duration : 0;
            var cyclePhase = (pathPhase * cyclesPerPath) % 1.0;
            var q = (double[])qPrev.Clone();

            for (var leg = 0; leg < n; leg++)
            {
                var (phaseSwinging, phaseLocal) = LegSwingPhase(layout, leg, cyclePhase);
                var hipWorld = HipWorld(layout, leg, baseFrame);
                var nominalLand = NominalFootWorld(leg, nominalFootBody, baseFrame);
                var stretched = HorizontalDistance(hipWorld, plants[leg]) > maxStanceReach;
                var shouldSwing = phaseSwinging || stretched;
                Vec3 footWorld;

                if (!shouldSwing)
                {
                    footWorld = new Vec3(plants[leg].X, plants[leg].Y, 0);
                    legSwingPhase[leg] = -1.0;
                }
                else
                {
                    if (legSwingPhase[leg] < 0)
                    {
                        swingFrom[leg] = plants[leg];
                        swingTo[leg] = nominalLand;
                        legSwingPhase[leg] = phaseSwinging ? phaseLocal : 0.0;
                    }
                    else if (phaseSwinging)
                    {
                        legSwingPhase[leg] = phaseLocal;
                    }
                    else
                    {
                        legSwingPhase[leg] = Math.Min(1.0, legSwingPhase[leg] + dt / swingPeriodSec);
                    }

                    var local = legSwingPhase[leg];
                    footWorld = SwingFoot(swingFrom[leg], swingTo[leg], local, stepHeight);
                    if (local >= 0.999)
                    {
                        plants[leg] = swingTo[leg];
                        if (!phaseSwinging && !stretched)
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

            if (!AllFinite(q))
            {
                error = "Gait sample produced non-finite joint values (NaN/Inf).";
                return false;
            }

            qPrev = q;
            points.Add(new TrajectoryPoint(tSec, new JointState(q)));
            basePath.Add(baseFrame);
        }

        if (ikFailSamples > 0)
            ikWarning = $"Foot-target IK failed on {ikFailSamples} leg×sample(s); held previous q (rad).";

        result = new Result(
            new Trajectory(model, points),
            basePath,
            ikWarning ?? "Foot-target duty gait (preview only — wire Trajectory → Preview; not Motus Plan).");
        return true;
    }

    public static double[] BuildStanceQ(LeggedLayout layout, double hip, double femur, double tibia)
    {
        var n = layout.LegCount;
        var q = new double[n * 3];
        for (var leg = 0; leg < n; leg++)
        {
            var side = layout.LegIsLeft(leg) ? 1.0 : -1.0;
            q[leg * 3 + 0] = layout.HipYawsRad[leg] + side * hip;
            q[leg * 3 + 1] = femur;
            q[leg * 3 + 2] = tibia;
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
            var footTargetBody = new Vec3(footBody.X, footBody.Y, 0);
            nominalFootBody[leg] = footTargetBody;

            if (!LegIk3R.TrySolve(hipBody, footTargetBody, layout.Coxa, layout.Femur, layout.Tibia, out _, out _, out _))
            {
                error = $"Leg {layout.LegNames[leg]}: stance foot at Z=0 unreachable (BodyZ={layout.BodyZ:F3} m too low or geometry infeasible).";
                return null;
            }

            plants[leg] = BodyToWorld(footTargetBody, startBase);
        }

        return plants;
    }

    private static Vec3 HipWorld(LeggedLayout layout, int leg, Frame baseFrame) =>
        BodyToWorld(HipBody(layout, leg), baseFrame);

    private static Vec3 NominalFootWorld(int leg, Vec3[] nominalFootBody, Frame baseFrame)
    {
        var w = BodyToWorld(nominalFootBody[leg], baseFrame);
        return new Vec3(w.X, w.Y, 0);
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
        return new Vec3(c * dx - s * dy, s * dx + c * dy, world.Z);
    }

    private static Vec3 BodyToWorld(Vec3 body, Frame baseFrame)
    {
        var yaw = YawFromFrame(baseFrame);
        var c = Math.Cos(yaw);
        var s = Math.Sin(yaw);
        return new Vec3(
            baseFrame.X + c * body.X - s * body.Y,
            baseFrame.Y + s * body.X + c * body.Y,
            body.Z);
    }

    private static double YawFromFrame(Frame f) => 2.0 * Math.Atan2(f.Qz, f.Qw);

    private static Vec3 SwingFoot(Vec3 start, Vec3 end, double phase01, double liftMeters)
    {
        var t = Math.Clamp(phase01, 0, 1);
        var x = start.X + (end.X - start.X) * t;
        var y = start.Y + (end.Y - start.Y) * t;
        var z = liftMeters > 0 ? liftMeters * Math.Sin(t * Math.PI) : 0;
        return new Vec3(x, y, z);
    }

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
