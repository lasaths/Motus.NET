namespace Motus.Core;

public sealed class JointState
{
    public double[] Positions { get; }

    public JointState(IReadOnlyList<double> positions)
    {
        Positions = positions.ToArray();
    }

    /// <summary>Wrap an existing buffer without copying. Mutating <see cref="Positions"/> mutates the buffer.</summary>
    public static JointState Wrap(double[] positions) => new(positions, copy: false);

    private JointState(double[] positions, bool copy)
    {
        Positions = copy ? positions.ToArray() : positions;
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
                {
                    var u = limits[i].UnitLabel;
                    errors.Add($"Joint {i + 1} value {Positions[i]:F4} {u} is outside [{limits[i].Min:F4}, {limits[i].Max:F4}] {u}.");
                }
            }
        }
        return errors.Count == 0 ? ValidationResult.Ok() : ValidationResult.Fail(errors);
    }
}
