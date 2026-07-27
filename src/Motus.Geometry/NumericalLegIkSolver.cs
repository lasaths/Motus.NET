using Motus.Core;

namespace Motus.Geometry;

/// <summary>
/// Numerical serial-leg IK via <see cref="NumericalInverseKinematics.TrySolveNear"/> (seed-only, no multi-start).
/// Position targets keep seed FK orientation so residual is translation-dominant.
/// </summary>
public sealed class NumericalLegIkSolver : ILegIkSolver
{
    private readonly NumericalInverseKinematics _ik;
    private readonly IFkSolver _fk;
    private readonly int _dof;
    private readonly double[] _seedHome;

    public NumericalLegIkSolver(
        KinematicTree legChain,
        string baseLink,
        string tipLink,
        NumericalIkOptions? options = null)
    {
        if (legChain is null) throw new ArgumentNullException(nameof(legChain));
        var tip = legChain.ExtractSerialTip(baseLink, tipLink);
        _dof = tip.Chain.Joints.Length;
        if (_dof < 1)
            throw new ArgumentException("Leg chain has no drivers.", nameof(legChain));

        Chain = legChain;
        BaseLink = baseLink;
        TipLink = tipLink;
        SerialTip = tip;

        var limits = new List<JointLimit>(_dof);
        var actuated = CollectActuated(legChain, tip);
        for (var i = 0; i < _dof; i++)
        {
            var j = actuated[i];
            limits.Add(new JointLimit(j.Lower, j.Upper, j.Velocity ?? Math.PI, (j.Velocity ?? Math.PI) * 2));
        }

        var preset = new RobotPreset
        {
            Manufacturer = RobotManufacturer.Unknown,
            ModelName = "leg_numerical",
            AxisCount = _dof,
            JointLimits = limits,
            BaseFrame = BaseFrame.Identity,
            ToolFrame = tip.TipToolOffset is { } off ? new ToolFrame(off, "tip") : ToolFrame.Identity,
        };

        _fk = KinematicsResolver.CreateFkSolver(preset, tip.Chain);
        _ik = new NumericalInverseKinematics(_fk, preset, tip.Chain, options);
        _seedHome = new double[_dof];
        _tool = preset.ToolFrame;
        Workspace = new LegIkWorkspace(
            0,
            EstimateMaxReach(tip.Chain),
            $"NumericalLegIk {_dof}R seed-only TrySolveNear");
    }

    private readonly ToolFrame _tool;

    public KinematicTree Chain { get; }
    public string BaseLink { get; }
    public string TipLink { get; }
    public SerialTipExtraction SerialTip { get; }
    public LegIkWorkspace Workspace { get; }

    public bool TrySolve(
        Vec3 hipBody,
        Vec3 footTargetBody,
        FootTargetKind kind,
        out double[] q,
        out LegIkFailureCode code)
    {
        q = (double[])_seedHome.Clone();
        if (!hipBody.IsFinite || !footTargetBody.IsFinite)
        {
            code = LegIkFailureCode.NonFiniteInput;
            return false;
        }

        var local = new Vec3(
            footTargetBody.X - hipBody.X,
            footTargetBody.Y - hipBody.Y,
            footTargetBody.Z - hipBody.Z);
        var seed = new JointState(_seedHome);
        var seedFk = _fk.ComputeTcp(seed, BaseFrame.Identity, _tool).Tcp;
        var orient = kind == FootTargetKind.Pose
            ? Frame.Identity
            : new Frame(0, 0, 0, seedFk.Qw, seedFk.Qx, seedFk.Qy, seedFk.Qz);
        var target = new CartesianPose(new Frame(local.X, local.Y, local.Z, orient.Qw, orient.Qx, orient.Qy, orient.Qz));

        if (!_ik.TrySolveNear(target, seed, out var sol))
        {
            code = LegIkFailureCode.Unreachable;
            return false;
        }

        q = sol.Positions.ToArray();
        for (var i = 0; i < q.Length; i++)
            _seedHome[i] = q[i];
        code = LegIkFailureCode.None;
        return true;
    }

    public bool TryNominalStance(
        Vec3 hipBody,
        double hipYawRad,
        double sideSign,
        double hipStanceRad,
        double femurStanceFallbackRad,
        double tibiaStanceFallbackRad,
        double bodyClearanceMeters,
        out double[] q,
        out Vec3 footBody,
        out LegIkFailureCode code)
    {
        q = new double[_dof];
        footBody = default;
        if (!hipBody.IsFinite || !double.IsFinite(hipYawRad) || !double.IsFinite(bodyClearanceMeters))
        {
            code = LegIkFailureCode.NonFiniteInput;
            return false;
        }

        var reach = Math.Max(0.05, Workspace.MaxReachMeters * 0.45);
        var heading = hipYawRad + sideSign * hipStanceRad;
        footBody = new Vec3(
            hipBody.X + reach * Math.Cos(heading),
            hipBody.Y + reach * Math.Sin(heading),
            0);

        if (TrySolve(hipBody, footBody, FootTargetKind.Position, out q, out code))
            return true;

        q = (double[])_seedHome.Clone();
        var fk = _fk.ComputeTcp(new JointState(q), BaseFrame.Identity, _tool).Tcp;
        footBody = new Vec3(hipBody.X + fk.X, hipBody.Y + fk.Y, hipBody.Z + fk.Z);
        code = LegIkFailureCode.None;
        return true;
    }

    private static List<KinematicJoint> CollectActuated(KinematicTree tree, SerialTipExtraction tip)
    {
        // Match driver order of ExtractSerialTip joint names.
        var byName = tree.Joints.ToDictionary(j => j.Name, StringComparer.OrdinalIgnoreCase);
        var list = new List<KinematicJoint>(tip.JointNames.Count);
        foreach (var name in tip.JointNames)
        {
            if (!byName.TryGetValue(name, out var j) || !j.IsActuated)
                throw new InvalidOperationException($"NumericalLegIk: missing actuated joint '{name}'.");
            list.Add(j);
        }

        return list;
    }

    private static double EstimateMaxReach(SerialJointChain chain)
    {
        var r = 0.0;
        foreach (var j in chain.Joints)
            r += Math.Abs(j.OriginX) + Math.Abs(j.OriginY) + Math.Abs(j.OriginZ);
        return Math.Max(r, 0.1);
    }
}
