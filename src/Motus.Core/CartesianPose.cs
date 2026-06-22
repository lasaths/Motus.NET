namespace Motus.Core;

public sealed class CartesianPose
{
    public Frame Tcp { get; }
    public CartesianPose(Frame tcp) => Tcp = tcp;
}
