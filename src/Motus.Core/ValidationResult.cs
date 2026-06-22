namespace Motus.Core;

public sealed class ValidationResult
{
    public bool IsValid { get; }
    public IReadOnlyList<string> Errors { get; }
    public IReadOnlyList<string> Warnings { get; }

    private ValidationResult(bool isValid, IReadOnlyList<string> errors, IReadOnlyList<string> warnings)
    {
        IsValid = isValid;
        Errors = errors;
        Warnings = warnings;
    }

    public static ValidationResult Ok(IReadOnlyList<string>? warnings = null) =>
        new(true, Array.Empty<string>(), warnings ?? Array.Empty<string>());

    public static ValidationResult Fail(IEnumerable<string> errors, IEnumerable<string>? warnings = null) =>
        new(false, errors.ToList(), warnings?.ToList() ?? new List<string>());
}
