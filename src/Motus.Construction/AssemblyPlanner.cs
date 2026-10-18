using Motus.Core;

namespace Motus.Construction;

/// <summary>
/// Input for <see cref="AssemblyPlanner.Plan"/>. Every field is optional; all null plans the shipped bamboo Warren truss.
/// </summary>
public sealed record AssemblyPlanRequest
{
    /// <summary>Structure to assemble (nodes, members in default order). Null: the Warren truss sized from <see cref="Site"/>.</summary>
    public BambooTrussLayout.StructureDefinition? Structure { get; init; }

    /// <summary>
    /// Site: member length, node gap, support level, pit depth, anchor columns ("anchor-&lt;node&gt;" marks a support
    /// node), stock location and start support. Null: <see cref="SiteDefaults.Chosen"/>.
    /// </summary>
    public BambooTrussLayout.Site? Site { get; init; }

    /// <summary>Extra fixed obstacles (site geometry) added to the anchor columns, stock pegs and pit.</summary>
    public IReadOnlyList<CollisionObject>? Obstacles { get; init; }

    /// <summary>Robot URDF XML (base_link → tool0, L/R grippers as in bamboo_icd.urdf). Null: the bound or fixture cell.</summary>
    public string? RobotUrdfXml { get; init; }

    /// <summary>Planner settings (clearance margin, search/verify steps). Null: <see cref="MotionConstraints.Current"/>.</summary>
    public MotionConstraints? Settings { get; init; }

    /// <summary>IK random-seed multiplier (<see cref="BambooGripKinematics.SeedScale"/>). Null: unchanged.</summary>
    public int? SeedScale { get; init; }

    /// <summary>Task sequence to run. Null: the searched default sequence (<see cref="BambooTrussMotion.ShippedTasks"/>).</summary>
    public IReadOnlyList<BambooIcdMotion.TaskEdit>? Tasks { get; init; }

    /// <summary>Return the embedded pre-computed plan when the input matches it (fast) instead of planning live.</summary>
    public bool UseShippedPlan { get; init; }
}

/// <summary>Result of <see cref="AssemblyPlanner.Plan"/>.</summary>
/// <param name="Build">Raw planner output (keys, cycles, grips, notes, built members, error).</param>
/// <param name="Tasks">Ordered tasks as Motus.Core task instances.</param>
/// <param name="Keys">Joint keyframes of the robot (degrees, with gripper openings and the clamped base per segment).</param>
/// <param name="Members">Members with their stock and placed cane poses.</param>
/// <param name="CollisionFree">True when the plan has no error and the replay scan finds no contact or tracking miss.</param>
/// <param name="Verdict">Human-readable collision verdict.</param>
public sealed record AssemblyPlan(
    BambooTrussMotion.TrussBuild Build,
    IReadOnlyList<TaskInstance> Tasks,
    IReadOnlyList<BambooIcdMotion.MotionKey> Keys,
    IReadOnlyList<BambooTrussLayout.MemberDef> Members,
    bool CollisionFree,
    string Verdict);

/// <summary>
/// Public entry point (e.g. for Motus.Grasshopper): plan the robotic assembly of a structure and replay-check it.
/// Wraps the existing planner (<see cref="BambooTrussMotion"/>) and scan (<see cref="BambooTrussScan"/>); no new logic.
/// The planner uses process-wide layout state, so calls must not run concurrently.
/// </summary>
public static class AssemblyPlanner
{
    static readonly object Gate = new();

    /// <summary>The example input: the shipped bamboo Warren truss.</summary>
    public static AssemblyPlanRequest WarrenTrussExample => new()
    {
        Site = SiteDefaults.Chosen,
        Structure = BambooTrussLayout.WarrenTruss(SiteDefaults.Chosen.Length + 2 * SiteDefaults.Chosen.Gap, SiteDefaults.Chosen.NodeY)
    };

    public static AssemblyPlan Plan(AssemblyPlanRequest? request = null)
    {
        request ??= new AssemblyPlanRequest();
        lock (Gate)
        {
            var site = (request.Site ?? SiteDefaults.Chosen) with
            {
                Structure = request.Structure ?? request.Site?.Structure,
                Obstacles = request.Obstacles ?? request.Site?.Obstacles
            };
            var oldSite = BambooTrussLayout.Current;
            var oldConstraints = MotionConstraints.Current;
            var oldSeeds = BambooGripKinematics.SeedScale;
            try
            {
                BambooTrussLayout.Use(site);
                if (request.Settings is not null) MotionConstraints.Current = request.Settings;
                if (request.SeedScale is { } s) BambooGripKinematics.SeedScale = s;
                BambooIcdCell? cell = null;
                if (request.RobotUrdfXml is not null)
                {
                    cell = BambooIcdCell.Load(request.RobotUrdfXml, BambooTrussLayout.StrutLength);
                    BambooTrussMotion.UseCell(cell);
                }
                cell ??= BambooTrussMotion.CurrentCell;
                var build = BambooTrussMotion.BuildAsync(request.Tasks ?? BambooTrussMotion.ShippedTasks(), null, request.UseShippedPlan)
                    .GetAwaiter().GetResult();
                var members = BambooTrussLayout.Members.ToList();
                bool clean; string verdict;
                if (build.Error.Length > 0) { clean = false; verdict = "plan failed: " + build.Error; }
                else if (cell is null) { clean = false; verdict = "not checked: no Motus cell"; }
                else
                {
                    var scan = new BambooTrussScan(cell, build.Keys, build.Cycles, members, BambooTrussLayout.JawClosedMm).RunToEnd();
                    clean = scan.FirstHit is null && scan.TrackMiss is null;
                    verdict = scan.FirstHit is { } h ? $"collision at t={h.T:F4} ({h.Label}): {h.Describe()}"
                        : scan.TrackMiss is { } m ? "tracking miss: " + m
                        : "collision-free";
                }
                return new AssemblyPlan(build, BambooTaskModel.ToCore(build.Tasks), build.Keys, members, clean, verdict);
            }
            finally
            {
                BambooTrussLayout.Use(oldSite);
                MotionConstraints.Current = oldConstraints;
                BambooGripKinematics.SeedScale = oldSeeds;
            }
        }
    }
}
