using Motus.Native;
using Motus.OMPL.Native;

namespace Motus.Core.Tests;

public class NativeRuntimeTests
{
    private static bool ExpectFullNative =>
        string.Equals(Environment.GetEnvironmentVariable("MOTUS_NATIVE_FULL"), "1", StringComparison.Ordinal);

    [Fact]
    public void NativeLibrary_LoadsFromPackageLayout()
    {
        if (!NativeBindings.LibraryLoaded && OperatingSystem.IsMacOS())
            return;

        Assert.True(NativeBindings.LibraryLoaded, NativeBindings.LastError());
    }

    [Fact]
    public void NativeAvailability_MatchesBuildProfile()
    {
        var ompl = NativeBindings.OmplIsAvailable();
        var fcl = NativeBindings.FclIsAvailable();
        if (ExpectFullNative)
        {
            Assert.True(ompl, $"OMPL unavailable: {NativeBindings.LastError()}");
            Assert.True(fcl, $"FCL unavailable: {NativeBindings.LastError()}");
            Assert.True(NativeOmpl.IsAvailable);
        }
        else
        {
            Assert.False(ompl);
            Assert.False(fcl);
        }
    }
}
