namespace Motus.Geometry;

public enum JointMotionType
{
    Revolute,
    Prismatic
}

/// <summary>URDF-style joint: fixed origin then revolute or prismatic motion about/along axis.</summary>
public readonly record struct JointDefinition(
    double OriginX, double OriginY, double OriginZ,
    double Roll, double Pitch, double Yaw,
    double AxisX, double AxisY, double AxisZ,
    double ThetaOffset = 0,
    JointMotionType Motion = JointMotionType.Revolute);

public sealed class SerialJointChain
{
    public JointDefinition[] Joints { get; }
    public double[] LinkRadiiMeters { get; }

    public SerialJointChain(JointDefinition[] joints, double[]? linkRadiiMeters = null)
    {
        Joints = joints;
        LinkRadiiMeters = linkRadiiMeters ?? Enumerable.Repeat(0.08, joints.Length).ToArray();
    }
}
