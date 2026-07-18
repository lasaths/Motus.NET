namespace Motus.Core;

/// <summary>Named joint chain from SRDF or host configuration.</summary>
public sealed class PlanningGroup
{
    public string Name { get; }
    public string BaseLink { get; }
    public string TipLink { get; }
    public IReadOnlyList<string> JointNames { get; }

    public PlanningGroup(string name, string baseLink, string tipLink, IReadOnlyList<string> jointNames)
    {
        Name = name ?? throw new ArgumentNullException(nameof(name));
        BaseLink = baseLink ?? throw new ArgumentNullException(nameof(baseLink));
        TipLink = tipLink ?? throw new ArgumentNullException(nameof(tipLink));
        JointNames = jointNames ?? throw new ArgumentNullException(nameof(jointNames));
    }
}

/// <summary>Maps planning-group joints to full robot joint indices. Internal planning use.</summary>
public sealed class JointIndexMap
{
    public IReadOnlyList<int> GroupToFull { get; }
    public IReadOnlyList<int> LockedFullIndices { get; }

    public JointIndexMap(IReadOnlyList<int> groupToFull, IReadOnlyList<int> lockedFullIndices)
    {
        GroupToFull = groupToFull;
        LockedFullIndices = lockedFullIndices;
    }

    public static JointIndexMap Resolve(RobotModel robot, PlanningGroup group)
    {
        var jointNames = robot.JointNames;
        if (jointNames is null || jointNames.Count == 0)
            throw new InvalidOperationException("RobotModel has no joint names for group mapping.");

        var map = new List<int>();
        if (group.JointNames.Count == 1 && group.JointNames[0].Contains("..", StringComparison.Ordinal))
        {
            for (var i = 0; i < jointNames.Count; i++)
                map.Add(i);
        }
        else
        {
            foreach (var jn in group.JointNames)
            {
                var idx = -1;
                for (var i = 0; i < jointNames.Count; i++)
                {
                    if (string.Equals(jointNames[i], jn, StringComparison.OrdinalIgnoreCase))
                    {
                        idx = i;
                        break;
                    }
                }
                if (idx < 0) throw new InvalidOperationException($"Joint '{jn}' not found on robot.");
                map.Add(idx);
            }
        }

        var locked = new List<int>();
        for (var i = 0; i < jointNames.Count; i++)
        {
            if (!map.Contains(i)) locked.Add(i);
        }
        return new JointIndexMap(map, locked);
    }

    // Scratch reused across OMPL validity callbacks (planning is single-threaded / gated).
    private double[]? _embedScratch;
    private JointState? _embedState;

    public JointState EmbedGroupState(JointState seed, IReadOnlyList<double> groupPositions)
    {
        if (groupPositions.Count != GroupToFull.Count)
            throw new ArgumentException("Group position count mismatch.");
        var n = seed.Positions.Length;
        if (_embedScratch is null || _embedScratch.Length != n)
        {
            _embedScratch = new double[n];
            _embedState = JointState.Wrap(_embedScratch);
        }
        for (var i = 0; i < n; i++)
            _embedScratch[i] = seed.Positions[i];
        for (var i = 0; i < GroupToFull.Count; i++)
            _embedScratch[GroupToFull[i]] = groupPositions[i];
        return _embedState!;
    }

    public double[] ExtractGroupPositions(JointState full)
    {
        var q = new double[GroupToFull.Count];
        for (var i = 0; i < GroupToFull.Count; i++)
            q[i] = full.Positions[GroupToFull[i]];
        return q;
    }
}
