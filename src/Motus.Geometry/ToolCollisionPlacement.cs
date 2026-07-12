using Motus.Core;

namespace Motus.Geometry;

public static class ToolCollisionPlacement
{
    private const string FlangeLocalToolName = "robotiq_2f85";

    public static bool UsesFlangePlacement(CollisionObject? geometry, bool explicitFlag = false) =>
        explicitFlag || string.Equals(geometry?.Name, FlangeLocalToolName, StringComparison.Ordinal);

    private static Frame? ResolveAttachOffset(CollisionObject? toolGeometry, Frame? geometryAttachOffset)
    {
        if (geometryAttachOffset is not null)
            return geometryAttachOffset;
        if (!string.Equals(toolGeometry?.Name, FlangeLocalToolName, StringComparison.Ordinal))
            return null;
        return Ur10eWrist3ToTool0();
    }

    private static Frame Ur10eWrist3ToTool0() =>
        Transforms.ToFrame(Transforms.Multiply(
            Transforms.FromRpy(0, 0, 0, 0, -Math.PI / 2, -Math.PI / 2),
            Transforms.FromRpy(0, 0, 0, Math.PI / 2, 0, Math.PI / 2)));

    public static double[] WorldMatrix(
        IFkSolver fk,
        IReadOnlyList<double> joints,
        BaseFrame baseFrame,
        ToolFrame toolFrame,
        CollisionObject? toolGeometry,
        bool geometryInFlangeFrame = false,
        Frame? geometryAttachOffset = null) =>
        WorldMatrix(
            fk,
            joints,
            baseFrame,
            toolFrame,
            UsesFlangePlacement(toolGeometry, geometryInFlangeFrame),
            ResolveAttachOffset(toolGeometry, geometryAttachOffset));

    public static double[] WorldMatrix(
        IFkSolver fk,
        IReadOnlyList<double> joints,
        BaseFrame baseFrame,
        ToolFrame toolFrame,
        bool geometryInFlangeFrame,
        Frame? geometryAttachOffset = null)
    {
        if (geometryInFlangeFrame)
        {
            var linkMats = fk.ComputeLinkTransforms(joints);
            var baseM = Transforms.FromFrame(baseFrame.Frame);
            if (linkMats.Count == 0)
                return baseM;

            var last = linkMats[^1];
            if (geometryAttachOffset is { } attach)
                last = Transforms.Multiply(last, Transforms.FromFrame(attach));
            return Transforms.Multiply(baseM, last);
        }

        return fk.ComputeTcpTransform(joints, baseFrame.Frame, toolFrame.Frame);
    }
}
