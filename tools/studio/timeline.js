// Task timeline overlay for the Dark studio.
// Shared file: keep it identical in every repo that copies tools/studio.
// Clock = video frame index on the scene timeline (an even subsample of the plan plus short holds),
// so the playhead and the fill move linearly and never stop.
(function () {
  const VERM = "#E64A19";
  const BG = "#111214";
  const MIN_W = 6, GAP = 2;  // detailed bar: every task at least 6 px wide, 2 px between tasks

  // Task type of one frame, from the plan's own phase / stage labels.
  // null = keep the previous task; "" = idle (start park, finished structure): an unlabelled segment that is never active.
  function taskOf(fr) {
    const p = String(fr.phase || "").toLowerCase();
    const s = String(fr.stage || "").toLowerCase();
    if (["start", "park", "frame"].includes(p) || s === "done") return "";
    if (p.startsWith("walk")) return "walk";                                   // Warren: walk_*
    if (p.includes(">")) return /_(via|grasp|close|lift)$/.test(p) ? "pick" : "relay"; // Warren relay leg
    if (s.includes("->")) return "bridge";                                     // carry/return stage "Pa->Pb"
    if (p === "flip") return "flip";
    if (p.startsWith("bridge")) return "bridge";
    if (/_(via|grasp|close|lift|pull)$/.test(p)) return "pick";                // Warren member pick
    if (/_(carry|place|open|park)$/.test(p) && !/^(h1|h2|mc|approach|place|flip|rehome)_/.test(p)) return "place";
    const verb = s.split(/\s+/)[1] || "";                                      // stage label such as "2 pick CB"
    if (verb === "pick") return "pick";
    if (verb === "place") return "place";
    if (verb === "down" || verb === "up" || verb === "carry") return "climb";
    return null;
  }

  // Member name when this frame releases a placed member, else null.
  function releaseOf(fr) {
    const p = String(fr.phase || "");
    if (p === "release") {                                                     // stage label such as "3 place R2 from P2"
      const m = /\bplace\s+(\S+)/i.exec(String(fr.stage || ""));
      return m ? m[1] : null;
    }
    const w = /^([^_>]+)_open$/.exec(p);                                       // Warren: <member>_open after <member>_place
    return w ? w[1] : null;
  }

  function build(data) {
    const tl = data.timeline || [];
    const u = data.uniqueFrames || [];
    const n = tl.length;
    const segs = [];
    const rel = {};                                                            // member -> last frame of its first release run
    const placePhases = new Set(u.map((f) => f.phase).filter((ph) => /_place$/.test(ph)));
    let prev = null;
    for (let i = 0; i < n; i++) {
      const fr = u[tl[i]];
      const own = taskOf(fr), t = own === null ? prev : own;
      prev = t;
      const last = segs[segs.length - 1];
      if (t !== null && last && last.task === t && last.b === i) last.b = i + 1;
      else if (t !== null) segs.push({ task: t, a: i, b: i + 1 });      // task "" = idle: drawn, never active
      const m = releaseOf(fr);
      if (m && (fr.phase === "release" || placePhases.has(`${m}_place`)) && (!(m in rel) || rel[m] === i - 1)) rel[m] = i;
    }
    const members = Object.entries(rel).map(([name, i]) => ({ name, i })).sort((x, y) => x.i - y.i);
    return { n, fps: data.fps || 30, segs, members };
  }

  const mix = (a, b, k) => "#" + [1, 3, 5].map((o) => {
    const v = Math.round(parseInt(a.substr(o, 2), 16) * (1 - k) + parseInt(b.substr(o, 2), 16) * k);
    return v.toString(16).padStart(2, "0");
  }).join("");
  const activeSeg = (m, i) => m.segs.find((s) => s.task && s.a <= i && i < s.b) || null;
  // One caption separator everywhere: " · ".
  const sep = (label) => String(label).replace(/\s+[-\u2013\u2014\u00b7]\s+/g, " \u00b7 ");
  // Bottom scrim over the hero: 0 % to 85 % BG over 96 px above the strip, then 85 % to the bottom.
  const scrim = (cfg, top) => {
    const w = cfg.layout === "single" ? 1920 : 1306, y0 = top - 96, k = (96 / (1080 - y0)).toFixed(3);
    return `<defs><linearGradient id="tl-scrim" x1="0" y1="0" x2="0" y2="1">` +
      `<stop offset="0" stop-color="${BG}" stop-opacity="0"/><stop offset="${k}" stop-color="${BG}" stop-opacity="0.85"/>` +
      `<stop offset="1" stop-color="${BG}" stop-opacity="0.85"/></linearGradient></defs>` +
      `<rect x="0" y="${y0}" width="${w}" height="${1080 - y0}" fill="url(#tl-scrim)"/>`;
  };
  const span = (cfg) => (cfg.layout === "single" ? [48, 1872] : [48, 1258]);   // hero pane only in multiview

  // Detailed bar layout: segment widths follow duration, but no task is narrower than MIN_W.
  function layoutBar(m, x0, x1) {
    const n = m.segs.length, room = x1 - x0 - GAP * (n - 1);
    let fixed = new Set(), w = [];
    for (let pass = 0; pass < n; pass++) {
      const free = m.segs.filter((_, k) => !fixed.has(k)).reduce((t, s) => t + s.b - s.a, 0);
      const scale = (room - MIN_W * fixed.size) / Math.max(1, free);
      w = m.segs.map((s, k) => (fixed.has(k) ? MIN_W : (s.b - s.a) * scale));
      const more = w.map((v, k) => (v < MIN_W && !fixed.has(k) ? k : -1)).filter((k) => k >= 0);
      if (!more.length) break;
      more.forEach((k) => fixed.add(k));
    }
    let x = x0;
    return m.segs.map((s, k) => { const r = { s, x, w: w[k] }; x += w[k] + GAP; return r; });
  }
  // Playhead x on the bar: linear in time inside each segment.
  function barX(bar, p) {
    for (const r of bar) if (p <= r.s.b) return r.x + r.w * Math.max(0, p - r.s.a) / (r.s.b - r.s.a);
    const last = bar[bar.length - 1];
    return last ? last.x + last.w : 0;
  }

  // Each mode: where the caption and the jaw chip sit, and an SVG draw function.
  // chipRight: the jaw chip ends at this x, on the caption baseline (null = old position).
  // A new mode (for example "gantt") is one more entry here; `reserve` is the bottom strip it needs.
  const MODES = {
    off: { captionY: 1038, captionSize: 17, chipY: 1030, chipRight: () => null, reserve: 0, draw: () => "" },

    // Option A: 2 px hairline, elapsed grey, active task vermilion (min 24 px), member ticks,
    // 2x10 px vermilion playhead tick. No text.
    simple: {
      captionY: 1038, captionSize: 22, chipY: 1034, chipRight: (cfg) => span(cfg)[1], reserve: 0,
      draw(m, i, cfg) {
        const [x0, x1] = span(cfg), y = 1062, p = Math.min(i + 1, m.n);
        const X = (v) => (x0 + (x1 - x0) * v / m.n).toFixed(1);
        const out = [
          scrim(cfg, y - 18),
          `<line x1="${x0}" x2="${x1}" y1="${y}" y2="${y}" stroke="#2A2C30" stroke-width="2"/>`,
          `<line x1="${x0}" x2="${X(p)}" y1="${y}" y2="${y}" stroke="#8A8D93" stroke-width="2"/>`,
        ];
        const s = activeSeg(m, i);
        if (s) {
          const xa = Math.max(x0, Math.min(+X(s.a), +X(p) - 24));
          out.push(`<line x1="${xa.toFixed(1)}" x2="${X(p)}" y1="${y}" y2="${y}" stroke="${VERM}" stroke-width="2"/>`);
        }
        for (const e of m.members) {
          out.push(`<line x1="${X(e.i + 1)}" x2="${X(e.i + 1)}" y1="${y - 6}" y2="${y + 6}" stroke="${e.i <= i ? "#E8E8E6" : "#4A4D53"}" stroke-width="2"/>`);
        }
        out.push(`<line x1="${X(p)}" x2="${X(p)}" y1="${y - 5}" y2="${y + 5}" stroke="${VERM}" stroke-width="2"/>`);
        return out.join("");
      },
    },

    // Option C: one 24 px segmented bar, one segment per task, white playhead,
    // task name under the active segment only.
    detailed: {
      captionY: 1012, captionSize: 22, chipY: 1008, chipRight: (cfg) => span(cfg)[1], reserve: 0,
      draw(m, i, cfg) {
        const [x0, x1] = span(cfg), y = 1032, p = Math.min(i + 1, m.n);
        const bar = layoutBar(m, x0, x1), xp = barX(bar, p);
        const out = [scrim(cfg, y - 12)];
        for (const { s, x, w } of bar) {
          const on = s.task && s.a <= i && i < s.b, past = s.b <= i;
          const fill = on ? VERM : past ? "#5C6068" : "#2A2C30";
          out.push(`<rect x="${x.toFixed(1)}" y="${y}" width="${w.toFixed(1)}" height="24" rx="2" fill="${fill}"/>`);
        }
        out.push(`<line x1="${xp.toFixed(1)}" x2="${xp.toFixed(1)}" y1="${y - 8}" y2="${y + 28}" stroke="#E8E8E6" stroke-width="2"/>`);
        const s = activeSeg(m, i);
        if (s) {
          const k = Math.min(1, (i - s.a) / Math.max(1, 0.12 * m.fps));       // 120 ms label fade
          const txt = s.task.toUpperCase(), half = (txt.length * 9.4) / 2;    // Barlow Medium 13 px + 0.08em
          const tx = Math.min(Math.max(xp, x0 + half), x1 - half);           // centred under the playhead, inside the bar
          out.push(`<text x="${tx.toFixed(1)}" y="${y + 41}" text-anchor="middle" font-size="13" font-weight="500" letter-spacing="1.04" fill="${mix("#6E7178", "#E8E8E6", k)}">${txt}</text>`);
        }
        return out.join("");
      },
    },
  };

  window.StudioTimeline = { build, MODES, taskOf, releaseOf, sep };
})();
