using Motus.Core;

namespace Motus.Core.Tests;

public class TaskInstanceTests
{
    [Fact]
    public void IdentityIsString()
    {
        var task = new TaskInstance("pick", new Dictionary<string, Frame>());
        Assert.Equal("pick", task.Identity);
    }

    [Fact]
    public void IdentityIsReusable_TwoPickInstancesWithDifferentPlanes()
    {
        var approachA = new Frame(0.1, 0.2, 0.3);
        var graspA = new Frame(0.1, 0.2, 0.25);
        var pickA = new TaskInstance("pick", new Dictionary<string, Frame>
        {
            ["approach"] = approachA,
            ["grasp"] = graspA
        });

        var approachB = new Frame(0.5, 0.6, 0.7);
        var graspB = new Frame(0.5, 0.6, 0.65);
        var pickB = new TaskInstance("pick", new Dictionary<string, Frame>
        {
            ["approach"] = approachB,
            ["grasp"] = graspB
        });

        Assert.Equal("pick", pickA.Identity);
        Assert.Equal("pick", pickB.Identity);
        Assert.NotEqual(pickA.Frames["approach"], pickB.Frames["approach"]);
        Assert.NotEqual(pickA.Frames["grasp"], pickB.Frames["grasp"]);
    }

    [Fact]
    public void EditingFrameKeepsIdentity()
    {
        var upright = new Frame(0.3, 0.4, 0.5);
        var release = new Frame(0.3, 0.4, 0.45);
        var place = new TaskInstance("place", new Dictionary<string, Frame>
        {
            ["upright"] = upright,
            ["release"] = release
        });

        var newUpright = new Frame(0.8, 0.9, 1.0);
        var updated = place.WithFrame("upright", newUpright);

        Assert.Equal("place", place.Identity);
        Assert.Equal("place", updated.Identity);
        Assert.Equal(newUpright, updated.Frames["upright"]);
        Assert.Equal(release, updated.Frames["release"]);
    }

    [Fact]
    public void EditingMetadataKeepsIdentity()
    {
        var task = new TaskInstance("pick", new Dictionary<string, Frame>(), new Dictionary<string, object>
        {
            ["object"] = "strut",
            ["jaw_mm"] = 10.0
        });

        var updated = task.WithMetadataValue("jaw_mm", 15.0);

        Assert.Equal("pick", task.Identity);
        Assert.Equal("pick", updated.Identity);
        Assert.Equal(10.0, task.Metadata["jaw_mm"]);
        Assert.Equal(15.0, updated.Metadata["jaw_mm"]);
    }

    [Fact]
    public void FramesAreReadOnly()
    {
        var frames = new Dictionary<string, Frame> { ["test"] = new Frame(1, 2, 3) };
        var task = new TaskInstance("test", frames);

        Assert.Throws<NotSupportedException>(() => 
            ((IDictionary<string, Frame>)task.Frames).Add("new", new Frame(4, 5, 6)));
    }

    [Fact]
    public void MetadataIsReadOnly()
    {
        var metadata = new Dictionary<string, object> { ["key"] = "value" };
        var task = new TaskInstance("test", new Dictionary<string, Frame>(), metadata);

        Assert.Throws<NotSupportedException>(() => 
            ((IDictionary<string, object>)task.Metadata).Add("new", "value"));
    }

    [Fact]
    public void EmptyIdentityThrows()
    {
        Assert.Throws<ArgumentException>(() => 
            new TaskInstance("", new Dictionary<string, Frame>()));
        Assert.Throws<ArgumentException>(() => 
            new TaskInstance("   ", new Dictionary<string, Frame>()));
    }
}

public class BambooArmStrutMoveTests
{
    /// <summary>
    /// Demonstrates the bamboo-arm strut move as data only:
    /// - Pick task: approach and grasp planes, strut metadata, closed jaw
    /// - Place task: upright pose and release planes, open jaw metadata
    /// </summary>
    [Fact]
    public void StrutMove_PickAndPlaceDataOnly()
    {
        // Pick task: approach and grasp frames
        var pickApproach = new Frame(0.2, 0.1, 0.5, qw: 0.707, qx: 0, qy: 0.707, qz: 0);
        var pickGrasp = new Frame(0.2, 0.1, 0.45, qw: 0.707, qx: 0, qy: 0.707, qz: 0);
        var pick = new TaskInstance("pick", 
            new Dictionary<string, Frame>
            {
                ["approach"] = pickApproach,
                ["grasp"] = pickGrasp
            },
            new Dictionary<string, object>
            {
                ["object"] = "bamboo_strut",
                ["jaw_mm"] = 8.0
            });

        // Place task: upright and release frames
        var placeUpright = new Frame(0.5, 0.6, 0.8);
        var placeRelease = new Frame(0.5, 0.6, 0.75);
        var place = new TaskInstance("place",
            new Dictionary<string, Frame>
            {
                ["upright"] = placeUpright,
                ["release"] = placeRelease
            },
            new Dictionary<string, object>
            {
                ["jaw_mm"] = 40.0
            });

        // A path is an ordered list of task instances
        var path = new TaskPath(pick, place);

        // Verify the path structure
        Assert.Equal(2, path.Tasks.Count);
        Assert.Equal("pick", path.Tasks[0].Identity);
        Assert.Equal("place", path.Tasks[1].Identity);

        // Verify pick task
        Assert.Equal(2, pick.Frames.Count);
        Assert.True(pick.Frames.ContainsKey("approach"));
        Assert.True(pick.Frames.ContainsKey("grasp"));
        Assert.Equal("bamboo_strut", pick.Metadata["object"]);
        Assert.Equal(8.0, pick.Metadata["jaw_mm"]);

        // Verify place task
        Assert.Equal(2, place.Frames.Count);
        Assert.True(place.Frames.ContainsKey("upright"));
        Assert.True(place.Frames.ContainsKey("release"));
        Assert.Equal(40.0, place.Metadata["jaw_mm"]);
    }

    [Fact]
    public void StrutMove_MultiplePicksInSequence()
    {
        var pick1 = new TaskInstance("pick", 
            new Dictionary<string, Frame>
            {
                ["approach"] = new Frame(0.1, 0.2, 0.5),
                ["grasp"] = new Frame(0.1, 0.2, 0.45)
            },
            new Dictionary<string, object> { ["object"] = "strut_A", ["jaw_mm"] = 8.0 });

        var place1 = new TaskInstance("place",
            new Dictionary<string, Frame>
            {
                ["upright"] = new Frame(0.5, 0.6, 0.8),
                ["release"] = new Frame(0.5, 0.6, 0.75)
            },
            new Dictionary<string, object> { ["jaw_mm"] = 40.0 });

        var pick2 = new TaskInstance("pick",
            new Dictionary<string, Frame>
            {
                ["approach"] = new Frame(0.3, 0.4, 0.6),
                ["grasp"] = new Frame(0.3, 0.4, 0.55)
            },
            new Dictionary<string, object> { ["object"] = "strut_B", ["jaw_mm"] = 8.0 });

        var place2 = new TaskInstance("place",
            new Dictionary<string, Frame>
            {
                ["upright"] = new Frame(0.7, 0.8, 0.9),
                ["release"] = new Frame(0.7, 0.8, 0.85)
            },
            new Dictionary<string, object> { ["jaw_mm"] = 40.0 });

        // Two complete strut moves
        var path = new TaskPath(pick1, place1, pick2, place2);

        Assert.Equal(4, path.Tasks.Count);
        Assert.Equal("pick", path.Tasks[0].Identity);
        Assert.Equal("place", path.Tasks[1].Identity);
        Assert.Equal("pick", path.Tasks[2].Identity);
        Assert.Equal("place", path.Tasks[3].Identity);

        // Different pick instances carry different data
        Assert.NotEqual(pick1.Frames["approach"], pick2.Frames["approach"]);
        Assert.Equal("strut_A", pick1.Metadata["object"]);
        Assert.Equal("strut_B", pick2.Metadata["object"]);
    }

    [Fact]
    public void StrutMove_EditPlaceFrame_IdentityRemains()
    {
        var place = new TaskInstance("place",
            new Dictionary<string, Frame>
            {
                ["upright"] = new Frame(0.5, 0.6, 0.8),
                ["release"] = new Frame(0.5, 0.6, 0.75)
            },
            new Dictionary<string, object> { ["jaw_mm"] = 40.0 });

        // Adjust the upright frame (e.g., for better placement)
        var adjustedUpright = new Frame(0.52, 0.61, 0.82);
        var updatedPlace = place.WithFrame("upright", adjustedUpright);

        // Identity stays "place"
        Assert.Equal("place", place.Identity);
        Assert.Equal("place", updatedPlace.Identity);
        Assert.Equal(adjustedUpright, updatedPlace.Frames["upright"]);
        Assert.Equal(place.Frames["release"], updatedPlace.Frames["release"]);

        // Can still use in a path
        var pick = new TaskInstance("pick",
            new Dictionary<string, Frame>
            {
                ["approach"] = new Frame(0.1, 0.2, 0.5),
                ["grasp"] = new Frame(0.1, 0.2, 0.45)
            },
            new Dictionary<string, object> { ["object"] = "strut", ["jaw_mm"] = 8.0 });

        var path = new TaskPath(pick, updatedPlace);
        Assert.Equal(2, path.Tasks.Count);
    }

    [Fact]
    public void FramesAreGeometricNotJoints()
    {
        // Frames represent position and orientation (geometric frames), not joint angles
        var frame = new Frame(x: 0.5, y: 0.3, z: 0.8, qw: 1, qx: 0, qy: 0, qz: 0);
        var task = new TaskInstance("test", new Dictionary<string, Frame> { ["pose"] = frame });

        // Frame has position components
        Assert.Equal(0.5, task.Frames["pose"].X);
        Assert.Equal(0.3, task.Frames["pose"].Y);
        Assert.Equal(0.8, task.Frames["pose"].Z);

        // Frame has orientation as quaternion
        Assert.Equal(1, task.Frames["pose"].Qw);
        Assert.Equal(0, task.Frames["pose"].Qx);
    }
}

public class TaskPathTests
{
    [Fact]
    public void EmptyPathIsValid()
    {
        var path = new TaskPath();
        Assert.Empty(path.Tasks);
    }

    [Fact]
    public void PathFromArray()
    {
        var task1 = new TaskInstance("pick", new Dictionary<string, Frame>());
        var task2 = new TaskInstance("place", new Dictionary<string, Frame>());
        var path = new TaskPath(task1, task2);

        Assert.Equal(2, path.Tasks.Count);
        Assert.Equal(task1, path.Tasks[0]);
        Assert.Equal(task2, path.Tasks[1]);
    }

    [Fact]
    public void PathFromEnumerable()
    {
        var tasks = new List<TaskInstance>
        {
            new TaskInstance("pick", new Dictionary<string, Frame>()),
            new TaskInstance("place", new Dictionary<string, Frame>()),
            new TaskInstance("pick", new Dictionary<string, Frame>())
        };
        var path = new TaskPath(tasks);

        Assert.Equal(3, path.Tasks.Count);
        Assert.Equal("pick", path.Tasks[0].Identity);
        Assert.Equal("place", path.Tasks[1].Identity);
        Assert.Equal("pick", path.Tasks[2].Identity);
    }

    [Fact]
    public void PathIsReadOnly()
    {
        var task = new TaskInstance("test", new Dictionary<string, Frame>());
        var path = new TaskPath(task);

        Assert.Throws<NotSupportedException>(() => 
            ((IList<TaskInstance>)path.Tasks).Add(task));
    }
}
