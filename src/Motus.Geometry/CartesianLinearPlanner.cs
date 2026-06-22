using Motus.Core;

namespace Motus.Geometry;

/// <summary>Cartesian goal via IK, then joint-space linear interpolation.</summary>
public sealed class CartesianLinearPlanner
{
    private readonly IInverseKinematics _ik;
    private readonly JointLinearPlanner _jointPlanner = new();

    public CartesianLinearPlanner(RobotPreset preset) =>
        _ik = KinematicsResolver.CreateInverseKinematics(preset);

    public CartesianLinearPlanner(IInverseKinematics ik) => _ik = ik;

    public PlanningResult Plan(CartesianPlanningRequest request)
    {
        var errors = new List<string>();
        var robot = request.Robot;

        if (!KinematicsProfiles.TryGet(robot.Preset, out _))
            return PlanningResult.Failed(new[] { $"No kinematics profile for '{robot.Preset.ModelName}'." });

        if (!_ik.TrySolve(request.Goal, request.Start, out var goalJoints))
            return PlanningResult.Failed(new[] { "IK failed to reach Cartesian goal from seed configuration." });

        var goalVal = goalJoints.Validate(robot.Preset.JointLimits);
        if (!goalVal.IsValid)
            return PlanningResult.Failed(goalVal.Errors.Select(e => $"IK goal: {e}"));

        var jointResult = _jointPlanner.Plan(new PlanningRequest(robot, request.Start, goalJoints, request.Options));
        if (!jointResult.Success) return jointResult;

        var warnings = jointResult.Warnings.ToList();
        warnings.Add("CartesianLinearPlanner: Cartesian goal solved via IK; path is joint-linear, not Cartesian-linear.");
        return PlanningResult.Succeeded(jointResult.Trajectory!, warnings);
    }
}
