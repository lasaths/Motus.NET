# Dark studio renderer for the Warren truss

This folder holds the Dark studio renderer for the shipped Warren truss plan. The adapter reads `src/Motus.Construction/Bamboo/BambooTrussDefaultPlan.json` and writes a scene file. The renderer draws that scene and writes a video or a still. The storyboard tool composes stills into one image.

## Look

- Background: Dark `#111214`.
- Locked jaws: matte Signal Vermilion `#E64A19`.
- Open jaws: matte graphite.
- Canes, posts and chassis: low-sheen matte bamboo.

The world frame stays as given in the scene JSON (`scene`, `worldUp` = +Z).

## Outputs

- Clean: one 16:9 hero view.
- Annotated: a 3-pane layout (hero + PLAN VIEW + ELEVATION) with stage labels.
- Storyboard: one PNG of key stages. A copy is in `docs/media/warren_truss_dark_storyboard.png`.

## Prerequisites

- Python 3. The adapter uses the standard library only. The storyboard tool also needs Pillow.
- Node.js 18 or later.
- Google Chrome or Chromium, or set `CHROME_PATH`.
- `ffmpeg`, only when you write a video.

## Usage

Run all commands from the repo root. Paths are relative to that root.

Write the scene file to `tools/studio/out/warren_truss_scene.json`:

```
python3 tools/studio/adapt_warren_plan.py
```

Write the Clean video, the Annotated 3-pane video, and the storyboard:

```
node tools/studio/render.mjs --scene tools/studio/out/warren_truss_scene.json --theme dark --mode clean --layout single --video tools/studio/out/warren_truss_dark_clean.mp4
node tools/studio/render.mjs --scene tools/studio/out/warren_truss_scene.json --theme dark --mode annotated --layout multiview --video tools/studio/out/warren_truss_dark_annotated.mp4
python3 tools/studio/make_storyboard.py --scene tools/studio/out/warren_truss_scene.json --theme dark --layout single --out tools/studio/out/warren_truss_dark_storyboard.png
```

Write one still:

```
node tools/studio/render.mjs --scene tools/studio/out/warren_truss_scene.json --theme dark --mode clean --layout single --still start --out tools/studio/out/warren_truss_start.png
```

The adapter accepts `--plan` and `--out` for other paths. Do not commit files under `tools/studio/out/`.
