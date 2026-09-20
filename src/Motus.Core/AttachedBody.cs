namespace Motus.Core;

/// <summary>
/// Runtime grasped / carried object geometry in parent-local frame.
/// Serial planners: <see cref="TcpLocalPose"/> is TCP-local.
/// HolonomicSE3 / free-flyer (ADR 0002 Phase B): same field is <b>base-local</b> when used with
/// <c>BaseFrameAttachCollisionChecker</c> / <c>FreeFlyerHullCollisionChecker.WithAttached</c>.
/// </summary>
public sealed class AttachedBody
{
    public string Name { get; }
    /// <summary>Parent-local pose (TCP for serial; base for aerial free-flyer).</summary>
    public Frame TcpLocalPose { get; }
    public CollisionObject Geometry { get; }
    /// <summary>When set, this scene obstacle name is hidden while attached.</summary>
    public string? SourceSceneObjectName { get; }

    public AttachedBody(string name, Frame tcpLocalPose, CollisionObject geometry, string? sourceSceneObjectName = null)
    {
        Name = name ?? throw new ArgumentNullException(nameof(name));
        TcpLocalPose = tcpLocalPose;
        Geometry = geometry ?? throw new ArgumentNullException(nameof(geometry));
        SourceSceneObjectName = sourceSceneObjectName;
    }
}
