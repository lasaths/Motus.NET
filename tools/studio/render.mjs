#!/usr/bin/env node
import http from "node:http";
import fs from "node:fs";
import path from "node:path";
import { spawn } from "node:child_process";
import { fileURLToPath } from "node:url";

const args = process.argv.slice(2);
const getArg = (k, def) => {
  const i = args.indexOf(k);
  return i >= 0 && i + 1 < args.length ? args[i + 1] : def;
};

const STUDIO_DIR = path.dirname(fileURLToPath(import.meta.url));
const SCENE_PATH = getArg("--scene", path.join(STUDIO_DIR, "scene.json"));
const THEME = getArg("--theme", "dark");       // dark | light | black
const MODE = getArg("--mode", "clean");        // clean | bare | annotated
const LAYOUT = getArg("--layout", "single");   // single | multiview
const STILL_PHASE = getArg("--still", null);   // phase name or frame index
const STILL_OUT = getArg("--out", path.join(STUDIO_DIR, `preview_${THEME}.png`));
const VIDEO_OUT = getArg("--video", null);

function findThree() {
  const candidates = [
    path.join(STUDIO_DIR, "vendor", "three.r128.min.js"),
    path.join(STUDIO_DIR, "three.r128.min.js"),
  ];
  for (const p of candidates) {
    if (fs.existsSync(p)) return p;
  }
  throw new Error("three.r128.min.js not found in tools/studio/vendor/");
}

function findChrome() {
  if (process.env.CHROME_PATH && fs.existsSync(process.env.CHROME_PATH)) {
    return process.env.CHROME_PATH;
  }
  const candidates = [
    "/usr/bin/google-chrome",
    "/usr/bin/google-chrome-stable",
    "/usr/bin/chromium",
    "/usr/bin/chromium-browser",
    "/snap/bin/chromium",
  ];
  for (const p of candidates) {
    if (fs.existsSync(p)) return p;
  }
  throw new Error("Chrome or Chromium not found. Set CHROME_PATH.");
}

function startServer(threePath) {
  return new Promise((resolve) => {
    const srv = http.createServer((req, res) => {
      const url = req.url.split("?")[0];
      if (url === "/" || url.startsWith("/viewer.html")) {
        res.writeHead(200, { "Content-Type": "text/html; charset=utf-8" });
        fs.createReadStream(path.join(STUDIO_DIR, "viewer.html")).pipe(res);
      } else if (url === "/three.min.js") {
        res.writeHead(200, { "Content-Type": "application/javascript" });
        fs.createReadStream(threePath).pipe(res);
      } else if (url === "/scene.json") {
        res.writeHead(200, { "Content-Type": "application/json" });
        fs.createReadStream(SCENE_PATH).pipe(res);
      } else {
        res.writeHead(404); res.end();
      }
    });
    srv.listen(0, "127.0.0.1", () => resolve({ srv, port: srv.address().port }));
  });
}

async function launchChrome(port, chromePath) {
  const dbgPort = 19222 + Math.floor(Math.random() * 10000);
  const proc = spawn(chromePath, [
    "--headless=new",
    "--no-sandbox",
    "--disable-dev-shm-usage",
    "--use-gl=angle",
    "--use-angle=swiftshader",
    "--enable-unsafe-swiftshader",
    "--enable-webgl",
    "--ignore-gpu-blocklist",
    "--window-size=1920,1080",
    "--force-device-scale-factor=1",
    "--hide-scrollbars",
    `--remote-debugging-port=${dbgPort}`,
    `http://127.0.0.1:${port}/viewer.html`,
  ], { stdio: ["ignore", "pipe", "pipe"] });

  let wsUrl = null;
  const t0 = Date.now();
  while (Date.now() - t0 < 20000) {
    try {
      const r = await fetch(`http://127.0.0.1:${dbgPort}/json/list`);
      const list = await r.json();
      const page = list.find((p) => p.type === "page" && p.webSocketDebuggerUrl);
      if (page) { wsUrl = page.webSocketDebuggerUrl; break; }
    } catch {}
    await new Promise((r) => setTimeout(r, 120));
  }
  if (!wsUrl) throw new Error("Chrome CDP did not start");

  const ws = new WebSocket(wsUrl);
  await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
  let nextId = 1;
  const pending = new Map();
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.id && pending.has(msg.id)) {
      const { res, rej } = pending.get(msg.id);
      pending.delete(msg.id);
      if (msg.error) rej(new Error(msg.error.message));
      else res(msg.result);
    }
  };
  const send = (method, params = {}) => new Promise((res, rej) => {
    const id = nextId++;
    pending.set(id, { res, rej });
    ws.send(JSON.stringify({ id, method, params }));
  });
  const evalJs = async (expression) => {
    const r = await send("Runtime.evaluate", { expression, awaitPromise: true, returnByValue: true });
    if (r.exceptionDetails) throw new Error(JSON.stringify(r.exceptionDetails));
    return r.result.value;
  };
  return { proc, ws, send, evalJs };
}

function resolveStillIndex(still, meta) {
  if (still === null || still === undefined) return null;
  if (still !== "" && Number.isInteger(Number(still)) && String(Number(still)) === String(still).trim()) {
    return Number(still);
  }
  const keys = meta.keyIndices || {};
  if (still in keys && typeof keys[still] === "number") return keys[still];
  if (still === "mid") return keys.mid ?? Math.floor((meta.uniqueCount - 1) / 2);
  if (still === "qa") return keys.qa ?? keys.mid ?? Math.floor((meta.uniqueCount - 1) / 2);
  if (still === "end") return keys.end ?? (meta.uniqueCount - 1);
  if (still === "start") return keys.start ?? 0;
  if (meta.phases && still in meta.phases) return meta.phases[still];
  return meta.uniqueCount - 1;
}

async function main() {
  const threePath = findThree();
  const chromePath = findChrome();
  const { srv, port } = await startServer(threePath);
  const chrome = await launchChrome(port, chromePath);
  try {
    await chrome.send("Emulation.setDeviceMetricsOverride", {
      width: 1920, height: 1080, deviceScaleFactor: 1, mobile: false,
    });
    for (let i = 0; i < 60; i++) {
      const ok = await chrome.evalJs("Boolean(window.STUDIO && document.fonts && document.fonts.status === 'loaded')");
      if (ok) break;
      await new Promise((r) => setTimeout(r, 100));
    }
    const meta = await chrome.evalJs(
      `window.STUDIO.init('/scene.json', ${JSON.stringify({ theme: THEME, mode: MODE, layout: LAYOUT })})`
    );

    if (STILL_PHASE !== null) {
      const idx = resolveStillIndex(STILL_PHASE, meta);
      await chrome.evalJs(`window.STUDIO.renderFrame(${idx})`);
      await chrome.evalJs(`new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))`);
      const { data } = await chrome.send("Page.captureScreenshot", { format: "png" });
      fs.mkdirSync(path.dirname(STILL_OUT), { recursive: true });
      fs.writeFileSync(STILL_OUT, Buffer.from(data, "base64"));
      console.log(`wrote still ${STILL_OUT} (frame ${idx})`);
    }

    if (VIDEO_OUT) {
      fs.mkdirSync(path.dirname(VIDEO_OUT), { recursive: true });
      const ff = spawn("ffmpeg", [
        "-y", "-loglevel", "error",
        "-f", "image2pipe", "-vcodec", "png", "-r", String(meta.fps), "-i", "-",
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", "-preset", "medium",
        "-movflags", "+faststart", VIDEO_OUT,
      ], { stdio: ["pipe", "inherit", "inherit"] });

      const cache = new Map();
      const tl = meta.timeline;
      const tStart = Date.now();
      for (let i = 0; i < tl.length; i++) {
        const uid = tl[i];
        let buf = cache.get(uid);
        if (!buf) {
          await chrome.evalJs(`window.STUDIO.renderFrame(${uid})`);
          const { data } = await chrome.send("Page.captureScreenshot", { format: "png" });
          buf = Buffer.from(data, "base64");
          cache.set(uid, buf);
        }
        if (!ff.stdin.write(buf)) {
          await new Promise((r) => ff.stdin.once("drain", r));
        }
        if (i % 90 === 0 || i === tl.length - 1) {
          console.log(`frame ${i + 1}/${tl.length} (unique ${cache.size}) ${((Date.now() - tStart) / 1000).toFixed(1)}s`);
        }
      }
      ff.stdin.end();
      await new Promise((res, rej) => { ff.on("exit", (c) => c === 0 ? res() : rej(new Error(`ffmpeg exit ${c}`))); });
      console.log(`wrote video ${VIDEO_OUT} (${tl.length} frames, ${(tl.length / meta.fps).toFixed(1)}s)`);
    }
  } finally {
    try { chrome.ws.close(); } catch {}
    chrome.proc.kill("SIGKILL");
    srv.close();
  }
}

main().catch((e) => { console.error(e); process.exit(1); });
