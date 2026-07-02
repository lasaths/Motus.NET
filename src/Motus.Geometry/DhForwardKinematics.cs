using Motus.Core;

namespace Motus.Geometry;

public sealed class DhForwardKinematics : IFkSolver
{
    private readonly KinematicsChain _chain;

    public DhForwardKinematics(KinematicsChain chain) => _chain = chain;

    public DhForwardKinematics(RobotPreset preset) : this(KinematicsProfiles.GetRequired(preset)) { }

    public CartesianPose ComputeTcp(JointState state, BaseFrame baseFrame, ToolFrame toolFrame)
    {
        var tcp = ComputeTcpTransform(state.Positions, baseFrame.Frame, toolFrame.Frame);
        return new CartesianPose(Transforms.ToFrame(tcp));
    }

    public double[] ComputeTcpTransform(IReadOnlyList<double> joints, Frame baseFrame, Frame toolFrame)
    {
        var linkMats = ComputeLinkTransforms(joints);
        var flange = linkMats[^1];
        return Transforms.Multiply(Transforms.Multiply(Transforms.FromFrame(baseFrame), flange), Transforms.FromFrame(toolFrame));
    }

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
        var links = _chain.Links;
        if (joints.Count != links.Length)
            throw new ArgumentException($"Expected {links.Length} joints, got {joints.Count}.");

        var mats = new double[joints.Count][];
        var cumulative = Transforms.Identity();
        for (var i = 0; i < joints.Count; i++)
        {
            var link = links[i];
            var local = Transforms.FromDh(joints[i] + link.ThetaOffset, link.D, link.A, link.Alpha);
            cumulative = Transforms.Multiply(cumulative, local);
            mats[i] = (double[])cumulative.Clone();
        }
        return mats;
    }

    public double[] LinkRadiiMeters => _chain.LinkRadiiMeters;
}
