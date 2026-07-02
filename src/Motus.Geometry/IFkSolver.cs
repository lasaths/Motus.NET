using Motus.Core;

namespace Motus.Geometry;

/// <summary>FK backend for IK and collision (DH or URDF serial chain).</summary>
public interface IFkSolver : IForwardKinematics
{
    double[] ComputeTcpTransform(IReadOnlyList<double> joints, Frame baseFrame, Frame toolFrame);
    IReadOnlyList<Frame> ComputeLinkOrigins(IReadOnlyList<double> joints, Frame baseFrame);
    IReadOnlyList<double[]> ComputeLinkTransforms(IReadOnlyList<double> joints);
    double[] LinkRadiiMeters { get; }
}
