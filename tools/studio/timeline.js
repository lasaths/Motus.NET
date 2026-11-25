// Task timeline overlay for the Dark studio.
// Shared file: keep it identical in Motus.Bambino and Motus.NET tools/studio.
// Clock = video frame index on the scene timeline (an even subsample of the plan plus short holds),
// so the playhead and the fill move linearly and never stop.
(function () {
  const VERM = "#E64A19";
  const MIN_FRAMES = 6;  // a task shorter than this (0.2 s at 30 fps) joins the task before it
  // Past-segment grey per task type (detailed mode); task types get no hues.
  const GREY = { pick: "#4A4D53", climb: "#5C6068", walk: "#5C6068", flip: "#6E7178",
                 bridge: "#80838A", relay: "#80838A", place: "#92959B" };

  // Task type of one frame, from the plan's own phase / stage labels.
  // null = keep the previous task; "" = idle (start park, finished structure): no segment.
  function taskOf(fr) {
    const p = String(fr.phase || "").toLowerCase();
    const s = String(fr.stage || "").toLowerCase();
    if (["start", "park", "frame"].includes(p) || s === "done") return "";
    if (p.startsWith("walk")) return "walk";                                   // Warren: walk_*
    if (p.includes(">")) return /_(via|grasp|close|lift)$/.test(p) ? "pick" : "relay"; // Warren relay leg
    if (s.includes("->")) return "bridge";                                     // Bambino: carry/return Pa->Pb
    if (p === "flip") return "flip";
    if (p.startsWith("bridge")) return "bridge";
    if (/_(via|grasp|close|lift|pull)$/.test(p)) return "pick";                // Warren member pick
    if (/_(carry|place|open|park)$/.test(p) && !/^(h1|h2|mc|approach|place|flip|rehome)_/.test(p)) return "place";
    const verb = s.split(/\s+/)[1] || "";                                      // Bambino stage "2 pick CB"
    if (verb === "pick") return "pick";
    if (verb === "place") return "place";
    if (verb === "down" || verb === "up" || verb === "carry") return "climb";
    return null;
  }

  // Member name when this frame releases a placed member, else null.
  function releaseOf(fr) {
    const p = String(fr.phase || "");
    if (p === "release") {                                                     // Bambino: stage "3 place R2 from P2"
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
      if (t && last && last.task === t && last.b === i) last.b = i + 1;
      else if (t) segs.push({ task: t, a: i, b: i + 1 });
      const m = releaseOf(fr);
      if (m && (fr.phase === "release" || placePhases.has(`${m}_place`)) && (!(m in rel) || rel[m] === i - 1)) rel[m] = i;
    }
    const merged = [];
    for (const s of segs) {
      const last = merged[merged.length - 1];
      if (last && last.b === s.a && (s.b - s.a < MIN_FRAMES || last.task === s.task)) last.b = s.b;
      else merged.push({ ...s });
    }
    const members = Object.entries(rel).map(([name, i]) => ({ name, i })).sort((x, y) => x.i - y.i);
    return { n, fps: data.fps || 30, segs: merged, members };
  }

  const mix = (a, b, k) => "#" + [1, 3, 5].map((o) => {
    const v = Math.round(parseInt(a.substr(o, 2), 16) * (1 - k) + parseInt(b.substr(o, 2), 16) * k);
    return v.toString(16).padStart(2, "0");
  }).join("");
  const activeSeg = (m, i) => m.segs.find((s) => s.a <= i && i < s.b) || null;
  const span = (cfg) => (cfg.layout === "single" ? [48, 1872] : [48, 1258]);   // hero pane only in multiview

  // Each mode: where the caption and the jaw chip sit, and an SVG draw function.
  // A new mode (for example "gantt") is one more entry here; `reserve` is the bottom strip it needs.
  const MODES = {
    off: { captionY: 1038, captionSize: 17, chipY: 1030, reserve: 0, draw: () => "" },

    // Option A: 2 px hairline, elapsed grey, active task vermilion, member ticks. No text.
    simple: {
      captionY: 1038, captionSize: 17, chipY: 1030, reserve: 0,
      draw(m, i, cfg) {
        const [x0, x1] = span(cfg), y = 1062, p = Math.min(i + 1, m.n);
        const X = (v) => (x0 + (x1 - x0) * v / m.n).toFixed(1);
        const out = [
          `<line x1="${x0}" x2="${x1}" y1="${y}" y2="${y}" stroke="#2A2C30" stroke-width="2"/>`,
          `<line x1="${x0}" x2="${X(p)}" y1="${y}" y2="${y}" stroke="#8A8D93" stroke-width="2"/>`,
        ];
        const s = activeSeg(m, i);
        if (s) out.push(`<line x1="${X(s.a)}" x2="${X(p)}" y1="${y}" y2="${y}" stroke="${VERM}" stroke-width="2"/>`);
        for (const e of m.members) {
          out.push(`<line x1="${X(e.i + 1)}" x2="${X(e.i + 1)}" y1="${y - 6}" y2="${y + 6}" stroke="${e.i <= i ? "#E8E8E6" : "#4A4D53"}" stroke-width="2"/>`);
        }
        return out.join("");
      },
    },

    // Option C: one 24 px segmented bar, one segment per task, white playhead, task tag.
    detailed: {
      captionY: 1012, captionSize: 22, chipY: 1004, reserve: 0,
      draw(m, i, cfg) {
        const [x0, x1] = span(cfg), y = 1032, p = Math.min(i + 1, m.n);
        const X = (v) => x0 + (x1 - x0) * v / m.n;
        const out = [];
        for (const s of m.segs) {
          const on = s.a <= i && i < s.b, past = s.b <= i;
          const fill = on ? VERM : past ? (GREY[s.task] || "#5C6068") : "#1F2124";
          out.push(`<rect x="${(X(s.a) + 1).toFixed(1)}" y="${y}" width="${Math.max(1, X(s.b) - X(s.a) - 2).toFixed(1)}" height="24" rx="2" fill="${fill}"/>`);
        }
        out.push(`<line x1="${X(p).toFixed(1)}" x2="${X(p).toFixed(1)}" y1="${y - 8}" y2="${y + 32}" stroke="#E8E8E6" stroke-width="2"/>`);
        const s = activeSeg(m, i);
        if (s) {
          const k = Math.min(1, (i - s.a) / Math.max(1, 0.12 * m.fps));       // 120 ms label fade
          const tx = Math.min(Math.max(X(s.a) + 1, x0), x1 - 64);             // label rides under its segment
          out.push(`<text x="${tx.toFixed(1)}" y="${y + 41}" font-size="13" font-weight="600" letter-spacing="1.04" fill="${mix("#6E7178", "#E8E8E6", k)}">${s.task.toUpperCase()}</text>`);
        }
        return out.join("");
      },
    },
  };

  window.StudioTimeline = { build, MODES, taskOf, releaseOf };
})();
