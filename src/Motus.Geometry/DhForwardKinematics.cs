using Motus.Core;

namespace Motus.Geometry;

public sealed class DhForwardKinematics : IFkSolver
{
    private readonly KinematicsChain _chain;
    private double[]? _local;
    private double[]? _accum;
    private double[]? _temp;

    public DhForwardKinematics(KinematicsChain chain) => _chain = chain;

    public DhForwardKinematics(RobotPreset preset) : this(KinematicsProfiles.GetRequired(preset)) { }

    public CartesianPose ComputeTcp(JointState state, BaseFrame baseFrame, ToolFrame toolFrame)
    {
        var tcp = ComputeTcpTransform(state.Positions, baseFrame.Frame, toolFrame.Frame);
        return new CartesianPose(Transforms.ToFrame(tcp));
    }

    public double[] ComputeFlangeTransform(IReadOnlyList<double> joints)
    {
        var links = _chain.Links;
        if (joints.Count != links.Length)
            throw new ArgumentException($"Expected {links.Length} joints, got {joints.Count}.");

        EnsureScratch();
        Transforms.IdentityInto(_accum!);
        for (var i = 0; i < joints.Count; i++)
        {
            var link = links[i];
            Transforms.FromDhInto(_local!, joints[i] + link.ThetaOffset, link.D, link.A, link.Alpha);
            Transforms.MultiplyInto(_temp!, _accum!, _local!);
            (_accum, _temp) = (_temp, _accum);
        }
        return (double[])_accum!.Clone();
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
        var links = _chain.Links;
        if (joints.Count != links.Length)
            throw new ArgumentException($"Expected {links.Length} joints, got {joints.Count}.");
        if (mats.Length < joints.Count)
            throw new ArgumentException($"Expected at least {joints.Count} matrix slots, got {mats.Length}.");

        EnsureScratch();
        Transforms.IdentityInto(_accum!);
        for (var i = 0; i < joints.Count; i++)
        {
            var link = links[i];
            Transforms.FromDhInto(_local!, joints[i] + link.ThetaOffset, link.D, link.A, link.Alpha);
            Transforms.MultiplyInto(_temp!, _accum!, _local!);
            (_accum, _temp) = (_temp, _accum);
            Array.Copy(_accum!, mats[i], 16);
        }
    }

    public double[] LinkRadiiMeters => _chain.LinkRadiiMeters;

    private void EnsureScratch()
    {
        _local ??= new double[16];
        _accum ??= new double[16];
        _temp ??= new double[16];
    }
}
