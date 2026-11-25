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
- Storyboard: one PNG of key stages, with a mini timeline under each panel. A copy is in `docs/media/warren_truss_dark_storyboard.png`.

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

To select the task timeline, add `--timeline simple`, `--timeline detailed` or `--timeline off` to a `render.mjs` command (see [Task timeline](#task-timeline)):

```
node tools/studio/render.mjs --scene tools/studio/out/warren_truss_scene.json --theme dark --mode clean --layout single --timeline detailed --video tools/studio/out/warren_truss_dark_clean_detailed.mp4
```

The adapter accepts `--plan` and `--out` for other paths. Do not commit files under `tools/studio/out/`.

## Task timeline

The renderer draws a task timeline at the bottom of each video. Use `--timeline` to select the mode:

- `simple`: a 2 px line under the caption. The elapsed part is grey. The active task is vermilion and is at least 24 px long. A 2x10 px vermilion tick shows the current time. A short white tick shows each member that is complete. The line has no text.
- `detailed`: one 24 px bar under the hero view. Each segment is one task, with a 2 px gap between tasks. Past segments are `#5C6068`, future segments are `#2A2C30`, and the active segment is vermilion. The task name is under the active segment only. The caption moves up above the bar.
- `off`: no timeline. The output is the same as before.

Each mode puts a `#111214` backing (85% opacity) behind the timeline, so the floor grid does not show through it. The jaw chip (`annotated`) ends at the right end of the timeline, on the same baseline as the caption. Captions use ` · ` as the separator.

The default is `simple` for `clean` and `detailed` for `annotated`. The `bare` mode has no timeline. In the 3-pane layout, the timeline is under the hero view only.

The timeline uses the video frame as its clock. The playhead moves at a constant speed and does not stop. The timeline does not scroll or zoom.

`timeline.js` gets the tasks from the phase and stage labels in the scene file:

- Task types are `pick`, `climb`, `flip`, `bridge`, `place`, `walk` and `relay`.
- Consecutive frames of the same type are one task. A task shorter than 6 frames joins the task before it.
- Start, park and finished frames are idle. Idle frames have no segment.
- A member is complete at the end of its release (`release` in the stage `place <member>`, or `<member>_open`).

The storyboard shows the panels in time order. Each panel has its video time (mm:ss) and a 4 px mini timeline. A vermilion tick shows the time of the panel. Use `make_storyboard.py --timeline off` to remove it.

`timeline.js` is the same file in Motus.Bambino and Motus.NET (`tools/studio/`). If you change it, change it in the two repos.
