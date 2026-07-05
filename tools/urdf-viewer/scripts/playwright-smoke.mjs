/**
 * Playwright CLI smoke checks for the URDF viewer.
 * Usage (dev server on :5173):
 *   npm run dev
 *   npm run test:playwright
 */
import { spawnSync } from 'node:child_process';
import { mkdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(__dirname, '..', '..', '..');
const outDir = path.join(root, 'output', 'playwright');
mkdirSync(outDir, { recursive: true });

const session = 'viewer-test';

function quote(arg) {
  if (/[\s"'()&|<>^]/.test(arg)) {
    return `"${arg.replace(/"/g, '\\"')}"`;
  }
  return arg;
}

function run(args, { raw = false } = {}) {
  const parts = [
    'npx',
    '--yes',
    '--package',
    '@playwright/cli',
    'playwright-cli',
    `-s=${session}`,
  ];
  if (raw) parts.push('--raw');
  for (const arg of args) parts.push(quote(arg));
  const cmd = parts.join(' ');
  const res = spawnSync(cmd, { cwd: root, encoding: 'utf8', shell: true });
  const text = `${res.stdout ?? ''}${res.stderr ?? ''}`.trim();
  if (res.status !== 0) {
    throw new Error(`playwright-cli ${args.join(' ')} failed (${res.status}):\n${text}`);
  }
  return (res.stdout ?? '').trim();
}

function evalJs(expr) {
  const out = run(['eval', expr], { raw: true });
  try {
    return JSON.parse(out);
  } catch {
    return out;
  }
}

const failures = [];

function assert(name, ok, detail = '') {
  if (!ok) failures.push({ name, detail });
  console.log(`${ok ? '✓' : '✗'} ${name}${detail ? ` — ${detail}` : ''}`);
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

console.log('Playwright viewer smoke\n');

run(['open', 'http://localhost:5173/']);
await sleep(5000);

const title = evalJs('() => document.title');
assert('page title', title.includes('Motus URDF Viewer'), title);

const dof = evalJs("() => document.getElementById('dof-count')?.textContent");
assert('robot loaded', dof === '6', `dof=${dof}`);

const summary = evalJs("() => document.getElementById('test-summary')?.textContent || ''");
assert('FK report loaded', /\\d+\\/\\d+ FK/.test(summary), summary);
assert('multi-point FK suite', parseInt(summary.match(/(\\d+) poses/)?.[1] ?? '0', 10) >= 12, summary);
assert('planning scenarios', summary.includes('2/2 plans'), summary);

const options = evalJs(
  "() => Array.from(document.getElementById('test-case')?.options || []).map(o => o.textContent).join('|')",
);
assert('planning scenario option', options.includes('RRT-Connect around sphere'), options);
assert('FK case options', options.split('|').filter((o) => o.includes('ur10e_')).length >= 6, options);

const dropHint = evalJs("() => document.getElementById('drop-hint')?.textContent || ''");
assert('planned trajectory loaded', dropHint.toLowerCase().includes('waypoint'), dropHint);

const playEnabled = evalJs('() => !document.getElementById("play-btn")?.disabled');
assert('play enabled', playEnabled === true, String(playEnabled));

const jointBefore = evalJs(
  "() => document.querySelector('#joints .joint-val')?.textContent || ''",
);
run(['click', '#play-btn']);
await sleep(1200);
const jointAfter = evalJs(
  "() => document.querySelector('#joints .joint-val')?.textContent || ''",
);
assert('playback moves joints', jointBefore !== jointAfter, `${jointBefore} -> ${jointAfter}`);
run(['click', '#play-btn']);
await sleep(200);

const themeBtn = evalJs("() => document.getElementById('theme-btn')?.textContent || ''");
if (themeBtn === 'Dark') {
  run(['click', '#theme-btn']);
}
let theme = evalJs("() => document.documentElement.dataset.theme || 'dark'");
assert('dark mode', theme === 'dark', theme);

run(['screenshot', path.join(outDir, 'viewer-dark.png')]);
console.log(`  screenshot: ${path.join(outDir, 'viewer-dark.png')}`);

run(['click', '#theme-btn']);
theme = evalJs("() => document.documentElement.dataset.theme || 'dark'");
const themeBtnLight = evalJs("() => document.getElementById('theme-btn')?.textContent || ''");
assert('light mode active', theme === 'light', theme);
assert('theme button shows Dark', themeBtnLight === 'Dark', themeBtnLight);

run(['screenshot', path.join(outDir, 'viewer-light.png')]);
console.log(`  screenshot: ${path.join(outDir, 'viewer-light.png')}`);

run(['localstorage-set', 'motus-viewer-theme', 'light']);
run(['reload']);
await sleep(4000);
theme = evalJs("() => document.documentElement.dataset.theme || 'dark'");
assert('theme restored from localStorage', theme === 'light', theme);

run(['select', '#fixture', 'kr210_r3100_ultra']);
await sleep(8000);
const hero = evalJs("() => document.getElementById('fixture-hero')?.textContent || ''");
const dofKr = evalJs("() => document.getElementById('dof-count')?.textContent || ''");
assert('KR210 fixture loads', hero.toLowerCase().includes('kr'), hero);
assert('KR210 has 6 joints', dofKr === '6', dofKr);

const krSummary = evalJs("() => document.getElementById('test-summary')?.textContent || ''");
assert('KR210 report', krSummary.includes('2/2 plans') || krSummary.includes('FK'), krSummary);

run(['screenshot', path.join(outDir, 'viewer-kr210.png')]);
console.log(`  screenshot: ${path.join(outDir, 'viewer-kr210.png')}`);

run(['close']);

console.log('');
if (failures.length) {
  console.error(`FAILED ${failures.length} check(s):`);
  for (const f of failures) console.error(`  - ${f.name}: ${f.detail}`);
  process.exit(1);
}
console.log('All smoke checks passed.');
