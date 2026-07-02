using Motus.Core;

namespace Motus.Geometry;

/// <summary>Cartesian goal via IK, then joint-space linear interpolation.</summary>
public sealed class CartesianLinearPlanner
{
    private readonly RobotPreset _preset;
    private readonly IInverseKinematics _ik;
    private readonly JointLinearPlanner _jointPlanner = new();

    public CartesianLinearPlanner(RobotPreset preset)
    {
        _preset = preset;
        _ik = KinematicsResolver.CreateInverseKinematics(preset);
    }

    public CartesianLinearPlanner(IInverseKinematics ik, RobotPreset preset)
    {
        _preset = preset;
        _ik = ik;
    }

    public PlanningResult Plan(CartesianPlanningRequest request)
    {
        var robot = request.Robot;
        var scene = request.CollisionScene ?? request.Options.CollisionScene;

        if (!KinematicsResolver.SupportsModel(robot.Preset))
            return PlanningResult.Failed(new[] { $"No kinematics profile for '{robot.Preset.ModelName}'." });

        if (!_ik.TrySolve(request.Goal, request.Start, out var goalJoints))
            return PlanningResult.Failed(new[] { "IK failed to reach Cartesian goal from seed configuration." });

        var goalVal = goalJoints.Validate(robot.Preset.JointLimits);
        if (!goalVal.IsValid)
            return PlanningResult.Failed(goalVal.Errors.Select(e => $"IK goal: {e}"));

        var checker = request.Options.CollisionChecker;
        if (PlanningCollision.SceneHasObstacles(scene) && checker is null)
            checker = new SphereCollisionChecker(robot.Preset);

        if (PlanningCollision.SceneHasObstacles(scene) && checker is null)
        {
            return PlanningResult.Failed(new[]
            {
                "Collision scene provided but no kinematics checker available for Cartesian planning."
            });
        }

        var options = new PlanningOptions
        {
            MaxJointStepRadians = request.Options.MaxJointStepRadians,
            TimeStepSeconds = request.Options.TimeStepSeconds,
            MaxJointVelocityRadiansPerSecond = request.Options.MaxJointVelocityRadiansPerSecond,
            CollisionScene = scene,
            CollisionChecker = checker
        };

        var jointResult = _jointPlanner.Plan(new PlanningRequest(robot, request.Start, goalJoints, options));
        if (!jointResult.Success) return jointResult;

        var warnings = jointResult.Warnings.ToList();
        warnings.Add("CartesianLinearPlanner: Cartesian goal via IK; path is joint-linear, not TCP-linear. Use CartesianLinearPathPlanner for LIN.");
        return PlanningResult.Succeeded(jointResult.Trajectory!, warnings);
    }
}
