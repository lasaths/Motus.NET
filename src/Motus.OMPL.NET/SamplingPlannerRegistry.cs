using Motus.Core;
using Motus.Geometry;
using Motus.Native;
using Motus.OMPL.Native;

namespace Motus.OMPL.NET;

/// <summary>Future SIMD validation backend hook (Phase 5 stub).</summary>
public static class VampValidationBackend
{
    public static bool IsSupported(RobotModel robot) => false;

    public static string StatusMessage =>
        "VAMP validation backend not integrated. Use default mesh/FCL collision checking.";
}

internal delegate PlanningResult PlannerDispatch(
    PlanningRequest request,
    SamplingPlannerOptions options,
    ICollisionChecker? defaultChecker,
    PlannerBackend backend);

internal sealed record PlannerBackend(
    SamplingPlannerId Id,
    string Label,
    string ShortName,
    int NativePlannerId,
    PlannerDispatch? CustomDispatch,
    Func<PlanningRequest, SamplingPlannerOptions, ICollisionChecker?, PlanningResult>? ManagedPlan);

/// <summary>Drop-in registration for sampling-based motion planners.</summary>
public static class SamplingPlannerRegistry
{
    private static readonly Dictionary<SamplingPlannerId, PlannerBackend> Backends = new();
    private static readonly object Gate = new();
    private static bool _initialized;

    static SamplingPlannerRegistry() => EnsureInitialized();

    public static IReadOnlyList<PlannerDescriptor> ListAvailable()
    {
        EnsureInitialized();
        lock (Gate)
        {
            return Backends.Values
                .Select(ToDescriptor)
                .Where(d => d.Id == SamplingPlannerId.ParallelRace
                    ? d.NativeSupported && d.ManagedSupported
                    : d.NativeSupported || d.ManagedSupported)
                .OrderBy(d => d.Id)
                .ToList();
        }
    }

    public static PlannerDescriptor? Resolve(SamplingPlannerId id)
    {
        EnsureInitialized();
        lock (Gate)
            return Backends.TryGetValue(id, out var backend) ? ToDescriptor(backend) : null;
    }

    public static bool TryParse(string text, out SamplingPlannerId id)
    {
        EnsureInitialized();
        id = SamplingPlannerId.RrtConnect;
        if (string.IsNullOrWhiteSpace(text)) return false;

        var normalized = text.Trim();
        lock (Gate)
        {
            foreach (var backend in Backends.Values)
            {
                if (backend.ShortName.Equals(normalized, StringComparison.OrdinalIgnoreCase) ||
                    backend.Label.Equals(normalized, StringComparison.OrdinalIgnoreCase))
                {
                    id = backend.Id;
                    return true;
                }
            }
        }

        return normalized switch
        {
            "RRT*" or "Star" or "RrtStar" => TrySet(SamplingPlannerId.RrtStar, out id),
            "RRT-Connect" or "Connect" => TrySet(SamplingPlannerId.RrtConnect, out id),
            "AORRTC" or "Aorrtc" => TrySet(SamplingPlannerId.Aorrtc, out id),
            "LBKPIECE" or "Lbkpiece" => TrySet(SamplingPlannerId.Lbkpiece, out id),
            "AIT*" or "AitStar" => TrySet(SamplingPlannerId.AitStar, out id),
            "EIT*" or "EitStar" => TrySet(SamplingPlannerId.EitStar, out id),
            "BLIT*" or "BlitStar" => TrySet(SamplingPlannerId.BlitStar, out id),
            "ParallelRace" or "Parallel" => TrySet(SamplingPlannerId.ParallelRace, out id),
            _ => false
        };
    }

    internal static bool IsNativeAvailable(SamplingPlannerId id)
    {
        var desc = Resolve(id);
        return desc?.NativeSupported == true;
    }

    internal static PlanningResult Dispatch(
        PlanningRequest request,
        SamplingPlannerOptions options,
        ICollisionChecker? defaultChecker)
    {
        EnsureInitialized();
        if (!Backends.TryGetValue(options.PlannerId, out var backend))
            return PlanningResult.Failed(new[] { $"Unknown planner: {options.PlannerId}." });

        var descriptor = ToDescriptor(backend);
        if (!descriptor.NativeSupported && !descriptor.ManagedSupported)
            return PlanningResult.Failed(new[] { descriptor.UnavailableReason ?? $"Planner {backend.Label} is unavailable." });

        if (backend.CustomDispatch is not null)
            return backend.CustomDispatch(request, options, defaultChecker, backend);

        var preferManaged = options.PreferManaged ||
            string.Equals(Environment.GetEnvironmentVariable("MOTUS_PREFER_MANAGED_PLANNER"), "1", StringComparison.Ordinal);

        if (!preferManaged && descriptor.NativeSupported && backend.NativePlannerId >= 0)
        {
            var native = NativeOmplPlanner.TryPlan(request, options, defaultChecker, backend.NativePlannerId, backend.Label);
            if (native is not null) return native;
        }

        if (backend.ManagedPlan is not null)
            return backend.ManagedPlan(request, options, defaultChecker);

        return PlanningResult.Failed(new[] { $"{backend.Label} requires native OMPL (not available in this build)." });
    }

    private static bool TrySet(SamplingPlannerId candidate, out SamplingPlannerId id)
    {
        id = candidate;
        var desc = Resolve(candidate);
        return desc is not null && (desc.NativeSupported || desc.ManagedSupported);
    }

    private static PlannerDescriptor ToDescriptor(PlannerBackend backend) => ToDescriptor(backend.Id, backend);

    private static PlannerDescriptor ToDescriptor(SamplingPlannerId id, PlannerBackend backend)
    {
        var nativeSupported = backend.NativePlannerId >= 0 &&
                              NativeOmpl.IsAvailable &&
                              NativeOmpl.motus_ompl_planner_available(backend.NativePlannerId) != 0;
        var managedSupported = backend.ManagedPlan is not null;

        if (backend.Id == SamplingPlannerId.ParallelRace)
        {
            var connect = Backends[SamplingPlannerId.RrtConnect];
            managedSupported = connect.ManagedPlan is not null ||
                               (NativeOmpl.IsAvailable &&
                                NativeOmpl.motus_ompl_planner_available(NativeBindings.PlannerRrtConnect) != 0);
            nativeSupported = NativeOmpl.IsAvailable &&
                              NativeOmpl.motus_ompl_planner_available(NativeBindings.PlannerAorrtc) != 0;
        }

        string? reason = null;
        if (!nativeSupported && !managedSupported)
            reason = $"{backend.Label} is not available (native OMPL missing or planner unsupported).";
        else if (!nativeSupported && backend.ManagedPlan is null && backend.CustomDispatch is null)
            reason = $"{backend.Label} requires native OMPL.";
        else if (backend.Id == SamplingPlannerId.ParallelRace && !nativeSupported)
            reason = "Parallel race requires native AORRTC (OMPL 2.0+).";

        return new PlannerDescriptor(id, backend.Label, backend.ShortName, nativeSupported, managedSupported, reason);
    }

    private static void EnsureInitialized()
    {
        if (_initialized) return;
        lock (Gate)
        {
            if (_initialized) return;
            RegisterDefaults();
            _initialized = true;
        }
    }

    private static void RegisterDefaults()
    {
        Register(new PlannerBackend(
            SamplingPlannerId.RrtConnect,
            "RRT-Connect",
            "RrtConnect",
            NativeBindings.PlannerRrtConnect,
            CustomDispatch: null,
            ManagedPlan: (req, opts, checker) => ManagedRrtConnect.Plan(req, opts, checker)));

        Register(new PlannerBackend(
            SamplingPlannerId.RrtStar,
            "RRT*",
            "RrtStar",
            NativeBindings.PlannerRrtStar,
            CustomDispatch: null,
            ManagedPlan: null));

        Register(new PlannerBackend(
            SamplingPlannerId.Aorrtc,
            "AORRTC",
            "Aorrtc",
            NativeBindings.PlannerAorrtc,
            CustomDispatch: null,
            ManagedPlan: null));

        Register(new PlannerBackend(
            SamplingPlannerId.Lbkpiece,
            "LBKPIECE",
            "Lbkpiece",
            NativeBindings.PlannerLbkpiece,
            CustomDispatch: null,
            ManagedPlan: null));

        Register(new PlannerBackend(
            SamplingPlannerId.AitStar,
            "AIT*",
            "AitStar",
            NativeBindings.PlannerAitStar,
            CustomDispatch: null,
            ManagedPlan: null));

        Register(new PlannerBackend(
            SamplingPlannerId.EitStar,
            "EIT*",
            "EitStar",
            NativeBindings.PlannerEitStar,
            CustomDispatch: null,
            ManagedPlan: null));

        Register(new PlannerBackend(
            SamplingPlannerId.BlitStar,
            "BLIT*",
            "BlitStar",
            NativeBindings.PlannerBlitStar,
            CustomDispatch: null,
            ManagedPlan: null));

        Register(new PlannerBackend(
            SamplingPlannerId.ParallelRace,
            "Parallel (Connect+AORRTC)",
            "ParallelRace",
            NativePlannerId: -1,
            CustomDispatch: (req, opts, checker, _) => ParallelRacePlanner.Plan(req, opts, checker),
            ManagedPlan: null));
    }

    internal static void Register(PlannerBackend backend) => Backends[backend.Id] = backend;
}
