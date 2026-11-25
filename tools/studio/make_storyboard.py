#!/usr/bin/env python3
"""Compose a 2x3 or 2x4 editorial storyboard from studio stills."""
from __future__ import annotations

import argparse
import bisect
import json
import os
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

STUDIO_DIR = Path(__file__).resolve().parent

THEMES = {
    "light": {"bg": (244, 244, 242), "pane": (238, 238, 235), "div": (212, 212, 208),
              "ink": (34, 37, 42), "muted": (118, 122, 129)},
    "dark":  {"bg": (17, 18, 20), "pane": (21, 23, 26), "div": (42, 45, 50),
              "ink": (236, 238, 241), "muted": (146, 152, 162)},
    "black": {"bg": (0, 0, 0), "pane": (7, 8, 10), "div": (34, 37, 43),
              "ink": (236, 238, 241), "muted": (140, 146, 156)},
}


def find_font(candidates, size):
    for p in candidates:
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    try:
        out = subprocess.check_output(
            ["fc-match", "-f", "%{file}", "Barlow:style=Medium"], text=True
        ).strip()
        if os.path.exists(out):
            return ImageFont.truetype(out, size)
    except Exception:
        pass
    return ImageFont.load_default()


BARLOW = "/usr/share/fonts/truetype/sand-box/google/Barlow"
FONT_CAND = {
    "reg": [f"{BARLOW}/Barlow-Regular.ttf", "/usr/share/fonts/truetype/barlow/Barlow-Regular.ttf"],
    "med": [f"{BARLOW}/Barlow-Medium.ttf", "/usr/share/fonts/truetype/barlow/Barlow-Medium.ttf"],
    "semi": [f"{BARLOW}/Barlow-SemiBold.ttf", "/usr/share/fonts/truetype/barlow/Barlow-SemiBold.ttf"],
}


def cells_from_scene(scene: dict):
    keys = scene.get("keyIndices") or {}
    names = keys.get("storyboard") if isinstance(keys.get("storyboard"), list) else None
    unique = scene.get("uniqueFrames") or []
    frames_by_phase = {}
    for fr in unique:
        frames_by_phase[fr["phase"]] = fr
    cells = []
    if names:
        for i, item in enumerate(names):
            if isinstance(item, dict):
                still = item.get("still", item.get("phase", i))
                if isinstance(still, int) and 0 <= still < len(unique):
                    fr = unique[still]
                else:
                    fr = frames_by_phase.get(still)
                label = item.get("label") or (fr or {}).get("phaseLabel") or str(still)
                idx = item.get("idx") or (fr or {}).get("phaseIdx") or f"{i + 1:02d}"
                cells.append((still, str(idx), label))
                continue
            fr = frames_by_phase.get(item)
            label = (fr or {}).get("phaseLabel") or str(item).replace("_", " ").title()
            idx = (fr or {}).get("phaseIdx") or f"{i + 1:02d}"
            cells.append((item, str(idx), label))
    if len(cells) < 6 and unique:
        n = len(unique)
        have = {c[0] for c in cells}
        for j in range(6):
            still = round(j * (n - 1) / 5) if n > 1 else 0
            if still in have:
                continue
            fr = unique[still]
            if j == 0:
                label = "Start"
            elif j == 5:
                label = "End"
            else:
                label = fr.get("phaseLabel") or fr.get("phase", "").replace("_", " ").title()
            cells.append((still, f"{len(cells) + 1:02d}", label))
            have.add(still)
            if len(cells) >= 6:
                break
        def _order_key(cell):
            still = cell[0]
            if isinstance(still, int):
                return still
            fr = frames_by_phase.get(still)
            return unique.index(fr) if fr in unique else 0
        cells = [(s, f"{i + 1:02d}", lab) for i, (s, _, lab) in enumerate(sorted(cells, key=_order_key)[:6])]
    return cells


def still_fraction(scene: dict, still) -> float:
    """Where a storyboard still sits on the video timeline, 0..1 (first frame that shows it)."""
    unique = scene.get("uniqueFrames") or []
    tl = scene.get("timeline") or []
    if not tl:
        return 0.0
    if not isinstance(still, int):
        keys = scene.get("keyIndices") or {}
        if isinstance(keys.get(still), int):
            still = keys[still]
        else:
            hits = [i for i, fr in enumerate(unique) if fr.get("phase") == still]
            still = hits[-1] if hits else 0
    j = tl.index(still) if still in tl else min(bisect.bisect_left(tl, still), len(tl) - 1)
    return (j + 1) / len(tl)


def draw_mini_timeline(draw, x0, x1, y, frac):
    """Mini option A: 4 px track, elapsed grey, vermilion tick at the panel's time."""
    xt = x0 + (x1 - x0) * frac
    draw.rectangle([x0, y, x1, y + 3], fill=(42, 44, 48))
    draw.rectangle([x0, y, xt, y + 3], fill=(138, 141, 147))
    draw.rectangle([xt - 1, y - 4, xt + 1, y + 7], fill=(230, 74, 25))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True)
    ap.add_argument("--theme", default="dark", choices=["light", "dark", "black"])
    ap.add_argument("--layout", default="single", choices=["single", "multiview"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeline", default="simple", choices=["simple", "off"],
                    help="simple: a 4 px mini timeline under each panel (default); off: none")
    args = ap.parse_args()

    scene = json.loads(Path(args.scene).read_text())
    cells = cells_from_scene(scene)
    T = THEMES[args.theme]
    # 2x3 for up to 6 hero tiles; 2x4 when multiview adds ortho panes
    use_ortho = args.layout == "multiview" and bool(scene.get("node")) and len(cells) >= 4
    # Always a full 2x3 hero grid (2x4 when the ortho column is added).
    cols = 4 if use_ortho else 3
    rows = 2
    W, H = 2560, 1440
    cw, ch = W // cols, H // rows
    sheet = Image.new("RGB", (W, H), T["bg"])
    draw = ImageDraw.Draw(sheet)

    f_idx = find_font(FONT_CAND["reg"], 20)
    f_lbl = find_font(FONT_CAND["med"], 20)
    f_tag = find_font(FONT_CAND["semi"], 16)

    pane = scene.get("paneLabels") or {}
    top_lbl = pane.get("top", "PLAN VIEW")
    bot_lbl = pane.get("bottom", "ELEVATION")
    marks = scene.get("angleMarks") or {}
    if marks.get("plan"):
        top_lbl = f"{top_lbl} · {marks['plan']}"
    if marks.get("elevation"):
        bot_lbl = f"{bot_lbl} · {marks['elevation']}"

    render_js = str(STUDIO_DIR / "render.mjs")
    with tempfile.TemporaryDirectory() as td:
        seat_img = None
        hero_cols = 3 if use_ortho else cols
        for idx, (phase, num, label) in enumerate(cells[: hero_cols * rows]):
            p = os.path.join(td, f"{idx}_{phase}.png")
            subprocess.check_call([
                "node", render_js,
                "--scene", args.scene,
                "--theme", args.theme,
                "--mode", "bare",
                "--layout", args.layout,
                "--still", str(phase),
                "--out", p,
            ])
            im = Image.open(p).convert("RGB")
            if phase in ("in_stroke", "seated_b", "seated_a") or idx == min(4, len(cells) - 1):
                seat_img = im.copy()
            if args.layout == "multiview":
                hero_src = im.crop((170, 20, 1136, 1060))
            else:
                # Trim a little margin from the 16:9 hero
                hero_src = im.crop((80, 40, 1840, 1020))
            scale = min(cw / hero_src.width, (ch - 52) / hero_src.height)
            nw, nh = int(hero_src.width * scale), int(hero_src.height * scale)
            hero = hero_src.resize((nw, nh), Image.Resampling.LANCZOS)
            col = idx % hero_cols
            row = idx // hero_cols
            x0, y0 = col * cw, row * ch
            sheet.paste(hero, (x0 + (cw - nw) // 2, y0 + (ch - 56 - nh) // 2 + 8))
            draw.text((x0 + 24, y0 + ch - 44), num, fill=T["muted"], font=f_idx)
            draw.text((x0 + 56, y0 + ch - 44), label, fill=T["ink"], font=f_lbl)
            if args.timeline == "simple":
                draw_mini_timeline(draw, x0 + 24, x0 + cw - 24, y0 + ch - 14, still_fraction(scene, phase))

        if use_ortho and seat_img is not None:
            end_crop = seat_img.crop((1307, 0, 1920, 540))
            plan_crop = seat_img.crop((1307, 541, 1920, 1080))
            for r_idx, (crop_im, tag) in enumerate([(end_crop, top_lbl), (plan_crop, bot_lbl)]):
                x0, y0 = 3 * cw, r_idx * ch
                draw.rectangle([x0, y0, x0 + cw, y0 + ch], fill=T["pane"])
                scale = min((cw - 24) / crop_im.width, (ch - 80) / crop_im.height)
                nw, nh = int(crop_im.width * scale), int(crop_im.height * scale)
                resized = crop_im.resize((nw, nh), Image.Resampling.LANCZOS)
                sheet.paste(resized, (x0 + (cw - nw) // 2, y0 + (ch - nh) // 2 + 10))
                draw.text((x0 + 24, y0 + 22), tag, fill=T["muted"], font=f_tag)

    for c in range(1, cols):
        draw.line([(c * cw, 0), (c * cw, H)], fill=T["div"], width=1)
    draw.line([(0, ch), (W, ch)], fill=T["div"], width=1)

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    sheet.save(args.out, optimize=True)
    print(f"wrote storyboard {args.out} ({W}x{H})")


if __name__ == "__main__":
    main()
