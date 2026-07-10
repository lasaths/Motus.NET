using System.Text.RegularExpressions;
using System.Xml.Linq;

namespace Motus.Presets;

public sealed class XacroOptions
{
    public IReadOnlyList<string> SearchPaths { get; init; } = Array.Empty<string>();
    public IReadOnlyDictionary<string, string> Args { get; init; } =
        new Dictionary<string, string>(StringComparer.Ordinal);
}

/// <summary>Minimal xacro expander: includes, properties, simple macros, ${arg} substitution. No $(find).</summary>
public static class XacroPreprocessor
{
    private static readonly XNamespace Xacro = "http://www.ros.org/wiki/xacro";
    private static readonly Regex ArgPattern = new(@"\$\{([^}]+)\}", RegexOptions.Compiled);

    public static string Expand(string path, XacroOptions? options = null)
    {
        options ??= new XacroOptions();
        var baseDir = Path.GetDirectoryName(Path.GetFullPath(path)) ?? ".";
        var searchPaths = BuildSearchPaths(baseDir, options);
        var doc = LoadAndExpand(path, searchPaths, new Dictionary<string, string>(options.Args, StringComparer.Ordinal));
        return doc.ToString(SaveOptions.DisableFormatting);
    }

    public static XDocument ExpandDocument(string path, XacroOptions? options = null)
    {
        options ??= new XacroOptions();
        var baseDir = Path.GetDirectoryName(Path.GetFullPath(path)) ?? ".";
        var searchPaths = BuildSearchPaths(baseDir, options);
        return LoadAndExpand(path, searchPaths, new Dictionary<string, string>(options.Args, StringComparer.Ordinal));
    }

    private static List<string> BuildSearchPaths(string baseDir, XacroOptions options)
    {
        var paths = new List<string> { baseDir };
        foreach (var p in options.SearchPaths)
            paths.Add(Path.IsPathRooted(p) ? p : Path.GetFullPath(Path.Combine(baseDir, p)));
        return paths;
    }

    private static XDocument LoadAndExpand(string path, IReadOnlyList<string> searchPaths, Dictionary<string, string> scope)
    {
        var doc = XDocument.Load(path, LoadOptions.SetLineInfo);
        var root = doc.Root ?? throw new InvalidOperationException("Xacro file has no root element.");
        var macros = new Dictionary<string, XElement>(StringComparer.Ordinal);
        ExpandElement(root, Path.GetDirectoryName(Path.GetFullPath(path)) ?? ".", searchPaths, scope, macros, expanding: new HashSet<string>(StringComparer.OrdinalIgnoreCase));
        StripXacroElements(root);
        return doc;
    }

    private static void ExpandElement(
        XElement element,
        string currentDir,
        IReadOnlyList<string> searchPaths,
        Dictionary<string, string> scope,
        Dictionary<string, XElement> macros,
        HashSet<string> expanding)
    {
        foreach (var child in element.Elements().ToList())
        {
            if (child.Name.Namespace == Xacro)
            {
                var local = child.Name.LocalName;
                if (local == "include")
                {
                    var file = Substitute(child.Attribute("filename")?.Value ?? "", scope);
                    if (file.Contains("$(find", StringComparison.Ordinal))
                        throw new NotSupportedException("$(find pkg) is not supported; expand xacro offline or add SearchPaths.");
                    var incPath = ResolveInclude(file, currentDir, searchPaths);
                    if (!expanding.Add(incPath))
                        throw new InvalidOperationException($"Circular xacro include: {incPath}");
                    var incDoc = LoadAndExpand(incPath, searchPaths, new Dictionary<string, string>(scope, StringComparer.Ordinal));
                    expanding.Remove(incPath);
                    var incRoot = incDoc.Root ?? throw new InvalidOperationException($"Included xacro has no root: {incPath}");
                    var inserted = new List<XElement>();
                    foreach (var incChild in incRoot.Elements().ToList())
                    {
                        var clone = new XElement(incChild);
                        inserted.Add(clone);
                        child.AddBeforeSelf(clone);
                    }
                    child.Remove();
                    foreach (var clone in inserted)
                        ExpandElement(clone, Path.GetDirectoryName(incPath) ?? currentDir, searchPaths, scope, macros, expanding);
                    continue;
                }

                if (local == "property")
                {
                    var name = child.Attribute("name")?.Value ?? "";
                    var value = Substitute(child.Attribute("value")?.Value ?? child.Value, scope);
                    scope[name] = value;
                    child.Remove();
                    continue;
                }

                if (local == "macro")
                {
                    var name = child.Attribute("name")?.Value ?? "";
                    macros[name] = new XElement(child);
                    child.Remove();
                    continue;
                }

                if (macros.TryGetValue(local, out var macro))
                {
                    var expanded = InstantiateMacro(macro, child, scope).ToList();
                    child.ReplaceWith(expanded);
                    foreach (var el in expanded)
                    {
                        ExpandElement(el, currentDir, searchPaths, scope, macros, expanding);
                        SubstituteAttributes(el, scope);
                        SubstituteText(el, scope);
                    }
                    continue;
                }

                child.Remove();
                continue;
            }

            ExpandElement(child, currentDir, searchPaths, scope, macros, expanding);
            SubstituteAttributes(child, scope);
            SubstituteText(child, scope);
        }

        SubstituteAttributes(element, scope);
        SubstituteText(element, scope);
    }

    private static IEnumerable<XElement> InstantiateMacro(XElement macro, XElement call, Dictionary<string, string> parentScope)
    {
        var localScope = new Dictionary<string, string>(parentScope, StringComparer.Ordinal);
        var paramAttr = macro.Attribute("params")?.Value;
        if (!string.IsNullOrWhiteSpace(paramAttr))
        {
            foreach (var param in paramAttr.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries))
            {
                var key = param.Trim();
                var val = call.Attribute(key)?.Value ?? call.Attribute($"*{key}")?.Value ?? "";
                localScope[key] = Substitute(val, localScope);
            }
        }

        var body = macro.Elements().Select(e => new XElement(e)).ToList();
        foreach (var el in body)
            ExpandElement(el, ".", Array.Empty<string>(), localScope, new Dictionary<string, XElement>(), new HashSet<string>());
        return body;
    }

    private static void StripXacroElements(XElement root)
    {
        foreach (var x in root.Descendants().Where(e => e.Name.Namespace == Xacro).ToList())
            x.Remove();
    }

    private static string ResolveInclude(string file, string currentDir, IReadOnlyList<string> searchPaths)
    {
        if (Path.IsPathRooted(file) && File.Exists(file)) return Path.GetFullPath(file);
        var rel = Path.Combine(currentDir, file);
        if (File.Exists(rel)) return Path.GetFullPath(rel);
        foreach (var dir in searchPaths)
        {
            var candidate = Path.Combine(dir, file);
            if (File.Exists(candidate)) return Path.GetFullPath(candidate);
        }
        throw new FileNotFoundException($"Xacro include not found: {file}");
    }

    private static void SubstituteAttributes(XElement element, Dictionary<string, string> scope)
    {
        foreach (var attr in element.Attributes().ToList())
            attr.Value = Substitute(attr.Value, scope);
    }

    private static void SubstituteText(XElement element, Dictionary<string, string> scope)
    {
        if (!element.HasElements && element.Value.Contains("${", StringComparison.Ordinal))
            element.Value = Substitute(element.Value, scope);
    }

    private static string Substitute(string text, Dictionary<string, string> scope) =>
        ArgPattern.Replace(text, m => scope.TryGetValue(m.Groups[1].Value.Trim(), out var v) ? v : m.Value);
}
