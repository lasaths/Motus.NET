using Motus.Core;

namespace Motus.Geometry;

/// <summary>Convenience pair of validated Stewart geometry + planning <see cref="RobotModel"/>.</summary>
public sealed class StewartRobot
{
    public StewartPlatform Platform { get; }
    public RobotModel Model { get; }
    public StewartInverseKinematics InverseKinematics { get; }
    public StewartForwardKinematics ForwardKinematics { get; }
    public StewartCartesianPathPlanner PathPlanner { get; }

    public StewartRobot(StewartPlatform platform)
    {
        Platform = platform ?? throw new ArgumentNullException(nameof(platform));
        Model = platform.ToModel();
        InverseKinematics = new StewartInverseKinematics(platform);
        ForwardKinematics = new StewartForwardKinematics(platform);
        PathPlanner = new StewartCartesianPathPlanner(platform);
    }

    public static StewartRobot CreateClassic(
        string modelName = "stewart_classic",
        double baseRadiusMeters = 0.5,
        double platformRadiusMeters = 0.3,
        double minStrokeMeters = 0.35,
        double maxStrokeMeters = 0.90) =>
        new(StewartPlatform.CreateClassic(
            modelName, baseRadiusMeters, platformRadiusMeters, minStrokeMeters, maxStrokeMeters));

    public static StewartRobot LoadFile(string path) => new(StewartPlatformLoader.LoadFile(path));
}
