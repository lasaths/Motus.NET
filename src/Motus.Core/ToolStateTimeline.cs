namespace Motus.Core;

/// <summary>Assigns per-waypoint tool states from motion program segment metadata.</summary>
public static class ToolStateTimeline
{
    public readonly record struct SegmentSpan(
        int SegmentIndex,
        MotionSegment Segment,
        int FirstPointIndex,
        int LastPointIndex);

    public static IReadOnlyList<TrajectoryPoint> Apply(
        IReadOnlyList<TrajectoryPoint> points,
        IReadOnlyList<MotionSegment> segments,
        IReadOnlyList<SegmentSpan> spans,
        EndEffectorState? initialState)
    {
        if (points.Count == 0) return points;

        var current = initialState ?? new EndEffectorState(new Dictionary<string, double>());
        var annotated = new TrajectoryPoint?[points.Count];
        var spanBySegment = spans.ToDictionary(s => s.SegmentIndex);

        annotated[0] = CopyWithToolState(points[0], current);

        for (var segIdx = 0; segIdx < segments.Count; segIdx++)
        {
            if (!spanBySegment.TryGetValue(segIdx, out var span)) continue;
            var segment = segments[segIdx];
            var first = Math.Max(span.FirstPointIndex, 0);
            var last = Math.Max(span.LastPointIndex, first);

            switch (segment)
            {
                case SetToolStateSegment set:
                    ApplySetSegment(points, annotated, first, last, set, ref current);
                    continue;
                case WaitSegment:
                case AttachSegment:
                case DetachSegment:
                    ApplyHoldSegment(points, annotated, first, last, current);
                    continue;
            }

            var target = segment.TargetState;
            var mode = segment.ToolStateMode;
            if (target is null || mode == ToolStateMode.Hold)
            {
                ApplyHoldSegment(points, annotated, first, last, current);
                continue;
            }

            if (mode == ToolStateMode.Instant)
            {
                current = target;
                for (var p = first; p <= last; p++)
                    annotated[p] = CopyWithToolState(points[p], current);
                continue;
            }

            var startState = current;
            var endState = target;
            current = endState;
            if (first == last)
            {
                annotated[first] = CopyWithToolState(points[first], endState);
                continue;
            }

            for (var p = first; p <= last; p++)
            {
                var alpha = (points[p].TimeSeconds - points[first].TimeSeconds) /
                            Math.Max(points[last].TimeSeconds - points[first].TimeSeconds, 1e-12);
                var state = EndEffectorState.Lerp(startState, endState, alpha);
                annotated[p] = CopyWithToolState(points[p], state);
            }
        }

        for (var i = 0; i < points.Count; i++)
        {
            if (annotated[i] is not null) continue;
            annotated[i] = CopyWithToolState(points[i], current);
        }

        return annotated.Select(p => p!).ToArray();
    }

    private static void ApplySetSegment(
        IReadOnlyList<TrajectoryPoint> points,
        TrajectoryPoint?[] annotated,
        int first,
        int last,
        SetToolStateSegment set,
        ref EndEffectorState current)
    {
        var startState = current;
        var endState = set.State;

        if (set.DurationSeconds <= 1e-12)
        {
            current = endState;
            for (var p = first; p <= last; p++)
                annotated[p] = CopyWithToolState(points[p], current);
            return;
        }

        current = endState;
        if (first == last)
        {
            annotated[first] = CopyWithToolState(points[first], endState);
            return;
        }

        for (var p = first; p <= last; p++)
        {
            var alpha = (points[p].TimeSeconds - points[first].TimeSeconds) /
                        Math.Max(points[last].TimeSeconds - points[first].TimeSeconds, 1e-12);
            var state = EndEffectorState.Lerp(startState, endState, alpha);
            annotated[p] = CopyWithToolState(points[p], state);
        }
    }

    private static void ApplyHoldSegment(
        IReadOnlyList<TrajectoryPoint> points,
        TrajectoryPoint?[] annotated,
        int first,
        int last,
        EndEffectorState current)
    {
        for (var p = first; p <= last; p++)
            annotated[p] = CopyWithToolState(points[p], current);
    }

    private static TrajectoryPoint CopyWithToolState(TrajectoryPoint source, EndEffectorState toolState) =>
        new(
            source.TimeSeconds,
            source.JointState,
            source.MotionType,
            source.SegmentIndex,
            source.BlendRadiusMeters,
            toolState);
}
