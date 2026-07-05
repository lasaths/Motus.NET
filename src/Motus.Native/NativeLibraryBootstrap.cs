using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Motus.Native;

/// <summary>Loads motus_native from NuGet runtimes/{rid}/native/ (Rhino Win/Mac, CI Linux).</summary>
internal static class NativeLibraryBootstrap
{
    private static int _initialized;
    private static IntPtr _loaded;

    internal static void EnsureResolver()
    {
        if (Interlocked.Exchange(ref _initialized, 1) == 1) return;
        NativeLibrary.SetDllImportResolver(typeof(NativeBindings).Assembly, Resolve);
        TryPreload();
    }

    internal static bool IsLoaded => _loaded != IntPtr.Zero;

    private static void TryPreload()
    {
        if (_loaded != IntPtr.Zero) return;
        var path = FindLibraryPath();
        if (path is not null && NativeLibrary.TryLoad(path, typeof(NativeBindings).Assembly, null, out var handle))
            _loaded = handle;
    }

    private static IntPtr Resolve(string libraryName, Assembly assembly, DllImportSearchPath? searchPath)
    {
        if (!string.Equals(libraryName, "motus_native", StringComparison.Ordinal))
            return IntPtr.Zero;

        if (_loaded != IntPtr.Zero) return _loaded;

        var path = FindLibraryPath();
        if (path is not null && NativeLibrary.TryLoad(path, assembly, searchPath, out var handle))
        {
            _loaded = handle;
            return handle;
        }

        if (NativeLibrary.TryLoad(libraryName, assembly, searchPath, out handle))
        {
            _loaded = handle;
            return handle;
        }

        return IntPtr.Zero;
    }

    private static string? FindLibraryPath()
    {
        var fileName = GetLibraryFileName();
        var rid = GetRuntimeIdentifier();
        if (rid is null) return null;

        foreach (var root in GetSearchRoots())
        {
            var candidate = Path.Combine(root, "runtimes", rid, "native", fileName);
            if (File.Exists(candidate)) return candidate;

            var flat = Path.Combine(root, fileName);
            if (File.Exists(flat)) return flat;
        }

        return null;
    }

    private static IEnumerable<string> GetSearchRoots()
    {
        if (!string.IsNullOrEmpty(AppContext.BaseDirectory))
            yield return AppContext.BaseDirectory;

        var asmDir = Path.GetDirectoryName(typeof(NativeBindings).Assembly.Location);
        if (!string.IsNullOrEmpty(asmDir))
            yield return asmDir;
    }

    internal static string? GetRuntimeIdentifier()
    {
        if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows) && RuntimeInformation.OSArchitecture == Architecture.X64)
            return "win-x64";
        if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
        {
            return RuntimeInformation.OSArchitecture switch
            {
                Architecture.Arm64 => "osx-arm64",
                Architecture.X64 => "osx-x64",
                _ => null
            };
        }
        if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux) && RuntimeInformation.OSArchitecture == Architecture.X64)
            return "linux-x64";
        return null;
    }

    private static string GetLibraryFileName()
    {
        if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows)) return "motus_native.dll";
        if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX)) return "libmotus_native.dylib";
        return "libmotus_native.so";
    }
}
