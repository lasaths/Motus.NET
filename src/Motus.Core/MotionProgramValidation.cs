namespace Motus.Core;

/// <summary>Validates tool state goals on motion program segments.</summary>
public static class MotionProgramValidation
{
    public static IReadOnlyList<string> ValidateToolStates(
        IReadOnlyList<MotionSegment> segments,
        ToolCapabilities? capabilities)
    {
        if (capabilities is null) return Array.Empty<string>();

        var errors = new List<string>();
        for (var i = 0; i < segments.Count; i++)
        {
            var segment = segments[i];
            EndEffectorState? state = segment switch
            {
                SetToolStateSegment set => set.State,
                _ => segment.TargetState
            };
            if (state is null) continue;

            foreach (var err in capabilities.Validate(state))
                errors.Add($"Segment {i + 1}: {err}");
        }

        return errors;
    }
}
