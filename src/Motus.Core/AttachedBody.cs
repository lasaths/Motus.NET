namespace Motus.Core;

/// <summary>Runtime grasped object geometry in TCP-local frame.</summary>
public sealed class AttachedBody
{
    public string Name { get; }
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
