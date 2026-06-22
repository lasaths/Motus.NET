namespace Motus.Core;

/// <summary>Position in meters, orientation as unit quaternion (w, x, y, z).</summary>
public readonly struct Frame : IEquatable<Frame>
{
    public double X { get; }
    public double Y { get; }
    public double Z { get; }
    public double Qw { get; }
    public double Qx { get; }
    public double Qy { get; }
    public double Qz { get; }

    public Frame(double x, double y, double z, double qw = 1, double qx = 0, double qy = 0, double qz = 0)
    {
        X = x; Y = y; Z = z; Qw = qw; Qx = qx; Qy = qy; Qz = qz;
    }

    public static Frame Identity => new(0, 0, 0);

    public bool Equals(Frame other) =>
        X == other.X && Y == other.Y && Z == other.Z &&
        Qw == other.Qw && Qx == other.Qx && Qy == other.Qy && Qz == other.Qz;

    public override bool Equals(object? obj) => obj is Frame f && Equals(f);
    public override int GetHashCode() => HashCode.Combine(X, Y, Z, Qw, Qx, Qy, Qz);
    public override string ToString() => $"({X:F3}, {Y:F3}, {Z:F3})";
}

public sealed class BaseFrame
{
    public Frame Frame { get; }
    public BaseFrame(Frame frame) => Frame = frame;
    public static BaseFrame Identity => new(Frame.Identity);
}

public sealed class ToolFrame
{
    public Frame Frame { get; }
    public string? Name { get; }
    public ToolFrame(Frame frame, string? name = null) { Frame = frame; Name = name; }
    public static ToolFrame Identity => new(Frame.Identity, "flange");
}
