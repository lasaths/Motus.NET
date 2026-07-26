using System.Text.Json;
using Motus.Core;
using Motus.Geometry;

// ponytail: dump WalkHex stick frames → self-contained preview.html (no Rhino).
// Run: dotnet run --project tools/legged-viewer
// Open preview.html in a browser (AI can screenshot via browser MCP).

var deg = Math.PI / 180.0;
var layout = LeggedLayout.HexMithi(0.06, 0.035, 0.08, 0.10, 0.07);
var limits = Enumerable.Range(0, 18).Select(_ => new JointLimit(-Math.PI, Math.PI, Math.PI, Math.PI * 2)).ToList();
var model = new RobotModel(layout.ToPreset("hex", 18, limits));

var path = new List<Vec3>();
for (var i = 0; i < 9; i++)
{
    var a = Math.PI - i / 8.0 * Math.PI;
    path.Add(new Vec3(0.22 + 0.18 * Math.Cos(a), 0.18 * Math.Sin(a), 0));
}

// Uneven ground: gentle ramp + ripples (stay in 3R workspace).
LeggedGait.TerrainHeight terrain = static (x, y) =>
    0.02 + 0.10 * x + 0.018 * Math.Sin(x * 16.0) * Math.Cos(y * 12.0);

if (!LeggedGait.TryBuild(
        layout, path, 0.06, 0.04, 0.025,
        7.5 * deg, 30 * deg, -30 * deg,
        model, out var result, out var err, terrain))
{
    Console.Error.WriteLine(err);
    return 1;
}

var frames = new List<object>();
var stride = Math.Max(1, result!.Trajectory.Points.Count / 48);
for (var i = 0; i < result.Trajectory.Points.Count; i += stride)
{
    var q = result.Trajectory.Points[i].JointState.Positions;
    var bf = result.BasePath[i];
    var yaw = 2.0 * Math.Atan2(bf.Qz, bf.Qw);
    var c = Math.Cos(yaw);
    var s = Math.Sin(yaw);

    var legs = new List<object>();
    for (var leg = 0; leg < layout.LegCount; leg++)
    {
        var hy = layout.HipYawsRad[leg];
        var hipB = new Vec3(layout.BodyR * Math.Cos(hy), layout.BodyR * Math.Sin(hy), layout.BodyZ);
        var q0 = q[leg * 3];
        var q1 = q[leg * 3 + 1];
        var q2 = q[leg * 3 + 2];
        // Same FK as Motus.NET (coxa tip → femur tip → foot).
        var coxaTip = new Vec3(
            hipB.X + Math.Cos(q0) * layout.Coxa,
            hipB.Y + Math.Sin(q0) * layout.Coxa,
            hipB.Z);
        var ankleB = LegIk3R.KneePosition(hipB, layout.Coxa, layout.Femur, q0, q1);
        var footB = LegIk3R.FootPosition(hipB, layout.Coxa, layout.Femur, layout.Tibia, q0, q1, q2);

        legs.Add(new
        {
            hip = W(hipB),
            knee = W(coxaTip),
            ankle = W(ankleB),
            foot = W(footB),
        });

        double[] W(Vec3 b) =>
        [
            bf.X + c * b.X - s * b.Y,
            bf.Y + s * b.X + c * b.Y,
            bf.Z + b.Z,
        ];
    }

    frames.Add(new
    {
        t = result.Trajectory.Points[i].TimeSeconds,
        basePose = new[] { bf.X, bf.Y, bf.Z, yaw },
        legs,
    });
}

// Terrain patch for viz (XY grid samples of the heightfield).
var terrainPts = new List<double[]>();
const int nx = 28, ny = 22;
for (var ix = 0; ix <= nx; ix++)
for (var iy = 0; iy <= ny; iy++)
{
    var x = -0.05 + 0.55 * ix / nx;
    var y = -0.30 + 0.60 * iy / ny;
    terrainPts.Add([x, y, terrain(x, y)]);
}

var payload = new
{
    generatedUtc = DateTime.UtcNow.ToString("o"),
    note = "WalkHex arc on uneven terrain (ramp + ripples). No Rhino.",
    layout = new { layout.BodyR, layout.Coxa, layout.Femur, layout.Tibia, layout.BodyZ },
    terrain = new { kind = "rampRipple", formula = "0.02+0.10*x+0.018*sin(16x)*cos(12y)", points = terrainPts, nx, ny },
    frames,
    warning = result.Warning,
    minSsm = result.MinStaticStabilityMarginMeters,
};

var outDir = AppContext.BaseDirectory;
// Prefer source tools/legged-viewer when running from repo.
var srcDir = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", ".."));
if (!File.Exists(Path.Combine(srcDir, "ExportGait.csproj")))
    srcDir = Directory.GetCurrentDirectory();

var json = JsonSerializer.Serialize(payload, new JsonSerializerOptions { WriteIndented = false });
var htmlPath = Path.Combine(srcDir, "preview.html");
var templatePath = Path.Combine(srcDir, "preview.template.html");
var template = File.ReadAllText(templatePath);
var html = template.Replace("/*__GAIT_JSON__*/null", json);
File.WriteAllText(htmlPath, html);
File.WriteAllText(Path.Combine(srcDir, "gait.json"), JsonSerializer.Serialize(payload, new JsonSerializerOptions { WriteIndented = true }));
Console.WriteLine($"Wrote {htmlPath}");
Console.WriteLine($"Frames: {frames.Count}  minSSM: {result.MinStaticStabilityMarginMeters:F4} m");
return 0;
