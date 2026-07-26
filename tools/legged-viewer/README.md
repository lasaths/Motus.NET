# Legged gait preview (no Rhino)

Stick-figure WalkHex viewer for Motus.NET `LeggedGait` — AI- and human-friendly.

```bash
cd Motus.NET
dotnet run --project tools/legged-viewer
# open tools/legged-viewer/preview.html
```

Regenerate after gait/terrain changes. Edit `Program.cs` (path, ramp, layout) then re-run.
`preview.html` is self-contained (JSON embedded); `gait.json` is the raw dump.
