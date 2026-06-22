namespace Motus.Core;

public sealed class JointState
{
    public double[] Positions { get; }

    public JointState(IReadOnlyList<double> positions)
    {
        Positions = positions.ToArray();
    }

    public int AxisCount => Positions.Length;

    public ValidationResult Validate(IReadOnlyList<JointLimit> limits)
    {
        var errors = new List<string>();
        if (limits.Count != Positions.Length)
            errors.Add($"Joint count mismatch: state has {Positions.Length}, limits have {limits.Count}.");
        else
        {
            for (var i = 0; i < Positions.Length; i++)
            {
                if (!limits[i].Contains(Positions[i]))
                    errors.Add($"Joint {i + 1} value {Positions[i]:F4} rad is outside [{limits[i].MinRadians:F4}, {limits[i].MaxRadians:F4}].");
            }
        }
        return errors.Count == 0 ? ValidationResult.Ok() : ValidationResult.Fail(errors);
    }
}
