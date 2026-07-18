using Motus.Core;

namespace Motus.Geometry;

/// <summary>FK from URDF-style joint origins (xyz + rpy) and revolute axes.</summary>
public sealed class SerialForwardKinematics : IFkSolver
{
    private readonly SerialJointChain _chain;

    public SerialForwardKinematics(SerialJointChain chain) => _chain = chain;

    public double[] LinkRadiiMeters => _chain.LinkRadiiMeters;

    public CartesianPose ComputeTcp(JointState state, BaseFrame baseFrame, ToolFrame toolFrame) =>
        new(Transforms.ToFrame(ComputeTcpTransform(state.Positions, baseFrame.Frame, toolFrame.Frame)));

    public double[] ComputeFlangeTransform(IReadOnlyList<double> joints)
    {
        if (joints.Count != _chain.Joints.Length)
            throw new ArgumentException($"Expected {_chain.Joints.Length} joints, got {joints.Count}.");

        var cumulative = Transforms.Identity();
        for (var i = 0; i < joints.Count; i++)
        {
            var j = _chain.Joints[i];
            var origin = Transforms.FromRpy(j.OriginX, j.OriginY, j.OriginZ, j.Roll, j.Pitch, j.Yaw);
            var motion = j.Motion == JointMotionType.Prismatic
                ? Transforms.FromPrismatic(j.AxisX, j.AxisY, j.AxisZ, joints[i] + j.ThetaOffset)
                : Transforms.FromAxisAngle(j.AxisX, j.AxisY, j.AxisZ, joints[i] + j.ThetaOffset);
            cumulative = Transforms.Multiply(cumulative, Transforms.Multiply(origin, motion));
        }
        return cumulative;
    }

    public double[] ComputeTcpTransform(IReadOnlyList<double> joints, Frame baseFrame, Frame toolFrame) =>
        Transforms.Multiply(
            Transforms.Multiply(Transforms.FromFrame(baseFrame), ComputeFlangeTransform(joints)),
            Transforms.FromFrame(toolFrame));

    public IReadOnlyList<Frame> ComputeLinkOrigins(IReadOnlyList<double> joints, Frame baseFrame)
    {
        var mats = ComputeLinkTransforms(joints);
        var baseM = Transforms.FromFrame(baseFrame);
        var frames = new Frame[mats.Count];
        for (var i = 0; i < mats.Count; i++)
            frames[i] = Transforms.ToFrame(Transforms.Multiply(baseM, mats[i]));
        return frames;
    }

    public IReadOnlyList<double[]> ComputeLinkTransforms(IReadOnlyList<double> joints)
    {
        var mats = new double[joints.Count][];
        for (var i = 0; i < mats.Length; i++)
            mats[i] = new double[16];
        ComputeLinkTransformsInto(joints, mats);
        return mats;
    }

    public void ComputeLinkTransformsInto(IReadOnlyList<double> joints, double[][] mats)
    {
        if (joints.Count != _chain.Joints.Length)
            throw new ArgumentException($"Expected {_chain.Joints.Length} joints, got {joints.Count}.");
        if (mats.Length < joints.Count)
            throw new ArgumentException($"Expected at least {joints.Count} matrix slots, got {mats.Length}.");

        var cumulative = Transforms.Identity();
        for (var i = 0; i < joints.Count; i++)
        {
            var j = _chain.Joints[i];
            var origin = Transforms.FromRpy(j.OriginX, j.OriginY, j.OriginZ, j.Roll, j.Pitch, j.Yaw);
            var motion = j.Motion == JointMotionType.Prismatic
                ? Transforms.FromPrismatic(j.AxisX, j.AxisY, j.AxisZ, joints[i] + j.ThetaOffset)
                : Transforms.FromAxisAngle(j.AxisX, j.AxisY, j.AxisZ, joints[i] + j.ThetaOffset);
            cumulative = Transforms.Multiply(cumulative, Transforms.Multiply(origin, motion));
            Array.Copy(cumulative, mats[i], 16);
        }
    }
}
