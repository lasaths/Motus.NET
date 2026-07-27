namespace Motus.Core;

/// <summary>Method references for managed time-optimal trajectory generation.</summary>
public static class TotgMethodRefs
{
    public const string PhamPham2018ToppraDoi = "10.1109/TRO.2018.2819195";

    public static string DescribeStack() =>
        $"TOTG/TOPP-RA-style retiming: controllable path-velocity intervals with separable joint velocity/acceleration limits (Pham & Pham 2018, DOI {PhamPham2018ToppraDoi}).";
}
