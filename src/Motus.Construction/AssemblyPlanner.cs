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

    /// <summary>Explicit ground surfaces / pit boxes. Null: the site's, else derived from the supports.</summary>
    public BambooTrussLayout.GroundDefinition? Ground { get; init; }

    /// <summary>Robot URDF XML. Null: the bound or fixture cell (bamboo_icd).</summary>
    public string? RobotUrdfXml { get; init; }

    /// <summary>Grippers and chain of <see cref="RobotUrdfXml"/>. Null: <see cref="RobotDefinition.BambooIcd"/>.</summary>
    public RobotDefinition? Robot { get; init; }

    /// <summary>Jaw strokes and fixture geometry. Null: unchanged (<see cref="BambooTrussLayout.Scene"/>).</summary>
    public BambooTrussLayout.SceneDescription? Scene { get; init; }

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
/// The planner uses process-wide state (site, scene, settings, robot, bound cell); each call sets it and restores the
/// previous state afterwards, and calls are serialised.
/// </summary>
public static class AssemblyPlanner
{
    static readonly object Gate = new();

    /// <summary>The example input: the shipped bamboo Warren truss.</summary>
    public static AssemblyPlanRequest WarrenTrussExample => new()
    {
        Site = SiteDefaults.Chosen,
        Structure = SiteDefaults.WarrenTruss(SiteDefaults.Chosen.Length + 2 * SiteDefaults.Chosen.Gap, SiteDefaults.Chosen.NodeY)
    };

    public static AssemblyPlan Plan(AssemblyPlanRequest? request = null)
    {
        request ??= new AssemblyPlanRequest();
        lock (Gate)
        {
            var site = (request.Site ?? SiteDefaults.Chosen) with
            {
                Structure = request.Structure ?? request.Site?.Structure,
                Obstacles = request.Obstacles ?? request.Site?.Obstacles,
                Ground = request.Ground ?? request.Site?.Ground
            };
            var oldSite = BambooTrussLayout.Current;
            var oldScene = BambooTrussLayout.Scene;
            var oldConstraints = MotionConstraints.Current;
            var oldSeeds = BambooGripKinematics.SeedScale;
            var oldRobot = RobotDefinition.Current;
            var oldSeg = BambooIcdMotion.Seg;
            var oldLimits = BambooIcdMotion.LimitsDeg;
            var oldCell = BambooTrussMotion.BoundCell;
            try
            {
                if (request.Scene is not null) BambooTrussLayout.UseScene(request.Scene);
                BambooTrussLayout.Use(site);
                if (request.Settings is not null) MotionConstraints.Current = request.Settings;
                if (request.SeedScale is { } s) BambooGripKinematics.SeedScale = s;
                BambooIcdCell? cell;
                if (request.RobotUrdfXml is not null)
                {
                    var robot = request.Robot ?? RobotDefinition.BambooIcd;
                    cell = BambooIcdCell.Load(request.RobotUrdfXml, BambooTrussLayout.StrutLength, robot);
                    RobotDefinition.Current = robot;
                    BambooIcdMotion.Seg = cell.Seg;
                    BambooIcdMotion.LimitsDeg = cell.LimitsDeg;
                }
                else cell = oldCell ?? BambooTrussMotion.CurrentCell;
                // Bind for this call, configured for this site (ground pads, start clamp).
                if (cell is not null) BambooTrussMotion.UseCell(cell);
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
                RobotDefinition.Current = oldRobot;
                BambooIcdMotion.Seg = oldSeg;
                BambooIcdMotion.LimitsDeg = oldLimits;
                BambooTrussLayout.UseScene(oldScene);
                BambooTrussLayout.Use(oldSite);
                MotionConstraints.Current = oldConstraints;
                BambooGripKinematics.SeedScale = oldSeeds;
                BambooTrussMotion.RestoreCell(oldCell);
            }
        }
    }
}
