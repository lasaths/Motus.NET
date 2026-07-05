import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';

const FIXTURES = [
  { id: 'ur10e', label: 'UR10e', file: 'ur10e.urdf', tip: 'tool0', dir: 'ur10e' },
  { id: 'kr210_r3100_ultra', label: 'KR 210 R3100 ultra', file: 'kr210_r3100_ultra.urdf', tip: 'tool0', dir: 'kr210_r3100_ultra' },
];

const FIXTURE_BASE = '/tests/fixtures/';

const viewport = document.getElementById('viewport');
const fixtureSelect = document.getElementById('fixture');
const jointsPanel = document.getElementById('joints');
const dropZone = document.getElementById('drop-zone');
const dropHint = document.getElementById('drop-hint');
const playBtn = document.getElementById('play-btn');
const resetBtn = document.getElementById('reset-btn');
const recenterBtn = document.getElementById('recenter-btn');
const statusLine = document.getElementById('status-line');
const fixtureHero = document.getElementById('fixture-hero');
const dofCount = document.getElementById('dof-count');
const testSummary = document.getElementById('test-summary');
const testCaseSelect = document.getElementById('test-case');
const themeBtn = document.getElementById('theme-btn');

const THEME_KEY = 'motus-viewer-theme';
const SCENE_THEMES = {
  dark: {
    bg: 0x000000,
    gridCenter: 0x333333,
    gridLine: 0x1a1a1a,
    path: 0x5b9bf6,
    robot: 0xffffff,
    ambient: 0.45,
    key: 0.85,
    fill: 0.25,
  },
  light: {
    bg: 0xf0f0f2,
    gridCenter: 0xc4c4ca,
    gridLine: 0xdedee4,
    path: 0x1d5bb8,
    robot: 0xc8c8ce,
    ambient: 0.62,
    key: 0.72,
    fill: 0.38,
  },
};

let currentTheme = 'dark';

const scene = new THREE.Scene();
scene.background = new THREE.Color(SCENE_THEMES.dark.bg);

const camera = new THREE.PerspectiveCamera(48, 1, 0.01, 50);
camera.position.set(1.2, 0.9, 1.2);

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
viewport.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0.25, 0);
controls.enableDamping = true;
controls.dampingFactor = 0.08;

const ambient = new THREE.AmbientLight(0xffffff, SCENE_THEMES.dark.ambient);
scene.add(ambient);
const key = new THREE.DirectionalLight(0xffffff, SCENE_THEMES.dark.key);
key.position.set(2.5, 4, 2);
scene.add(key);
const fill = new THREE.DirectionalLight(0xe8e8e8, SCENE_THEMES.dark.fill);
fill.position.set(-2, 1, -1);
scene.add(fill);

let grid = new THREE.GridHelper(2.5, 25, SCENE_THEMES.dark.gridCenter, SCENE_THEMES.dark.gridLine);
scene.add(grid);

// URDF is Z-up; THREE.js is Y-up (same as urdf-viewer-element default up="+Z").
const world = new THREE.Group();
world.rotation.x = -Math.PI / 2;
scene.add(world);

const obstacleGroup = new THREE.Group();
world.add(obstacleGroup);

const loader = new URDFLoader();
loader.parseCollision = true;
loader.parseVisual = true;
loader.workingPath = FIXTURE_BASE;

let robot = null;
let trajectory = null;
let trajectoryTimes = null;
let playTimer = null;
let isPlaying = false;
let actuatedJoints = [];
let viewerPresets = null;
let viewerReport = null;
let viewerReportPromise = null;
let loadFixtureSeq = 0;
let defaultPose = {};
let pathLine = null;
let currentFixtureId = null;
let currentScenarioId = null;
let fkCaseTrajectory = null;

const PRESETS_URL = `${FIXTURE_BASE}viewer_presets.json`;
const REPORT_URL = `${FIXTURE_BASE}viewer_report.json`;

function setStatus(text, state = '') {
  statusLine.textContent = text;
  statusLine.dataset.state = state;
}

function setGridTheme(centerHex, lineHex) {
  scene.remove(grid);
  grid.geometry.dispose();
  grid.material.dispose();
  grid = new THREE.GridHelper(2.5, 25, centerHex, lineHex);
  scene.add(grid);
}

function applyTheme(theme) {
  currentTheme = theme === 'light' ? 'light' : 'dark';
  const t = SCENE_THEMES[currentTheme];
  document.documentElement.dataset.theme = currentTheme === 'light' ? 'light' : '';
  scene.background.setHex(t.bg);
  setGridTheme(t.gridCenter, t.gridLine);
  ambient.intensity = t.ambient;
  key.intensity = t.key;
  fill.intensity = t.fill;
  if (pathLine) pathLine.material.color.setHex(t.path);
  styleRobotMaterials();
  themeBtn.textContent = currentTheme === 'light' ? 'Dark' : 'Light';
  themeBtn.title = currentTheme === 'light' ? 'Switch to dark mode' : 'Switch to light mode';
  try {
    localStorage.setItem(THEME_KEY, currentTheme);
  } catch {
    /* private browsing */
  }
}

function initTheme() {
  let theme = 'dark';
  try {
    const stored = localStorage.getItem(THEME_KEY);
    if (stored === 'light' || stored === 'dark') theme = stored;
    else if (window.matchMedia('(prefers-color-scheme: light)').matches) theme = 'light';
  } catch {
    /* ignore */
  }
  applyTheme(theme);
}

function setDropState(hint, state = '') {
  dropHint.textContent = hint;
  dropZone.dataset.state = state;
}

for (const f of FIXTURES) {
  const opt = document.createElement('option');
  opt.value = f.id;
  opt.textContent = f.label ?? f.id.replace(/_/g, ' ');
  fixtureSelect.appendChild(opt);
}

function resize() {
  const w = viewport.clientWidth;
  const h = viewport.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

function disposePathLine() {
  if (!pathLine) return;
  pathLine.parent?.remove(pathLine);
  pathLine.geometry.dispose();
  pathLine.material.dispose();
  pathLine = null;
}

function disposeRobot() {
  if (!robot) return;
  disposePathLine();
  clearObstacles();
  world.remove(robot);
  robot.traverse((c) => {
    if (c.geometry) c.geometry.dispose();
    if (c.material) {
      if (Array.isArray(c.material)) c.material.forEach((m) => m.dispose());
      else c.material.dispose();
    }
  });
  robot = null;
}

function styleRobotMaterials() {
  if (!robot) return;
  const robotColor = SCENE_THEMES[currentTheme].robot;
  robot.traverse((c) => {
    if (!c.isMesh) return;
    const mats = Array.isArray(c.material) ? c.material : [c.material];
    mats.forEach((m) => {
      if (!m) return;
      m.color?.setHex(robotColor);
      m.metalness = currentTheme === 'light' ? 0.15 : 0.1;
      m.roughness = currentTheme === 'light' ? 0.48 : 0.55;
    });
  });
}

function fitCamera() {
  if (!robot) return;
  const box = new THREE.Box3().setFromObject(robot);
  if (box.isEmpty()) {
    controls.target.set(0, 1.2, 0);
    camera.position.set(2.8, 2.2, 2.8);
    controls.update();
    return;
  }
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3()).length() || 1;
  controls.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(size * 0.75, size * 0.55, size * 0.75));
  controls.update();
}

function stopPlayback() {
  isPlaying = false;
  if (playTimer) {
    clearTimeout(playTimer);
    playTimer = null;
  }
  playBtn.textContent = 'Play test cases';
}

function buildJointSliders() {
  jointsPanel.innerHTML = '';
  actuatedJoints = [];
  if (!robot) {
    dofCount.textContent = '0';
    return;
  }

  const names = Object.keys(robot.joints).filter((n) => robot.joints[n].jointType !== 'fixed');
  for (const name of names) {
    const joint = robot.joints[name];
    actuatedJoints.push(name);

    const row = document.createElement('div');
    row.className = 'joint-row';

    const nameEl = document.createElement('span');
    nameEl.className = 'joint-name';
    nameEl.textContent = name;
    row.dataset.joint = name;

    const val = document.createElement('span');
    val.className = 'joint-val';
    val.textContent = '0.00';

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = joint.limit?.lower ?? -3.14;
    slider.max = joint.limit?.upper ?? 3.14;
    slider.step = '0.01';
    slider.value = '0';
    slider.addEventListener('input', () => {
      if (isPlaying) stopPlayback();
      const q = parseFloat(slider.value);
      robot.setJointValue(name, q);
      val.textContent = q.toFixed(2);
    });

    row.appendChild(nameEl);
    row.appendChild(val);
    row.appendChild(slider);
    jointsPanel.appendChild(row);
  }

  dofCount.textContent = String(actuatedJoints.length);
}

let exportJointNames = null;

function mapTrajectoryPoint(p) {
  if (p.joints && typeof p.joints === 'object' && !Array.isArray(p.joints)) return p.joints;
  const raw = p.jointsRadians ?? p.joints ?? [];
  if (Array.isArray(raw) && exportJointNames?.length) {
    const state = {};
    exportJointNames.forEach((name, i) => {
      if (i < raw.length) state[name] = raw[i];
    });
    return state;
  }
  return raw;
}

async function ensureViewerReport() {
  if (!viewerReportPromise) {
    viewerReportPromise = (async () => {
      try {
        const res = await fetch(REPORT_URL);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        viewerReport = await res.json();
        updateReportSummary();
      } catch {
        viewerReport = null;
        testSummary.textContent = 'No report — run dotnet test (UrdfFkCrossCheckTests)';
        testSummary.dataset.state = 'warn';
      }
      return viewerReport;
    })();
  }
  return viewerReportPromise;
}

function updateReportSummary() {
  if (!viewerReport?.summary) {
    testSummary.textContent = 'No report — run dotnet test (UrdfFkCrossCheckTests)';
    testSummary.dataset.state = 'warn';
    return;
  }
  const { passed, failed, total, viewerCases, planningScenarios, planningPassed, planningFailed } = viewerReport.summary;
  const stamp = viewerReport.generatedUtc?.slice(0, 19).replace('T', ' ') ?? '';
  const planNote = planningScenarios
    ? ` · ${planningPassed}/${planningScenarios} plans`
    : '';
  if (failed > 0 || planningFailed > 0) {
    testSummary.textContent = `${passed}/${total} FK · ${viewerCases} poses${planNote} · ${stamp}`;
    testSummary.dataset.state = 'err';
  } else {
    testSummary.textContent = `${passed}/${total} FK · ${viewerCases} poses${planNote} · ${stamp}`;
    testSummary.dataset.state = 'ok';
  }
}

function clearObstacles() {
  while (obstacleGroup.children.length) {
    const c = obstacleGroup.children[0];
    obstacleGroup.remove(c);
    c.geometry?.dispose();
    if (c.material) {
      if (Array.isArray(c.material)) c.material.forEach((m) => m.dispose());
      else c.material.dispose();
    }
  }
}

function renderObstacles(obstacles) {
  clearObstacles();
  const mat = new THREE.MeshStandardMaterial({
    color: 0xff4444,
    transparent: true,
    opacity: 0.38,
    roughness: 0.6,
  });
  for (const o of obstacles ?? []) {
    let mesh;
    if (o.shape === 'sphere') {
      mesh = new THREE.Mesh(new THREE.SphereGeometry(o.radius ?? 0.05, 20, 20), mat);
      mesh.position.set(o.x, o.y, o.z);
    } else if (o.shape === 'box') {
      mesh = new THREE.Mesh(
        new THREE.BoxGeometry((o.halfX ?? 0.05) * 2, (o.halfY ?? 0.05) * 2, (o.halfZ ?? 0.05) * 2),
        mat,
      );
      mesh.position.set(o.x, o.y, o.z);
    } else {
      continue;
    }
    mesh.name = o.name ?? 'obstacle';
    obstacleGroup.add(mesh);
  }
}

function populateTestCases(fixtureId) {
  testCaseSelect.innerHTML = '';
  const fixture = viewerReport?.fixtures?.[fixtureId];
  const cases = fixture?.cases ?? [];
  const scenarios = fixture?.scenarios ?? [];

  if (!cases.length && !scenarios.length) {
    const opt = document.createElement('option');
    opt.value = '';
    opt.textContent = 'No cases in report';
    testCaseSelect.appendChild(opt);
    testCaseSelect.disabled = true;
    return;
  }

  if (scenarios.length) {
    const group = document.createElement('optgroup');
    group.label = 'Planning scenarios';
    for (const s of scenarios) {
      const opt = document.createElement('option');
      opt.value = `scenario:${s.id}`;
      opt.textContent = `${s.passed ? '✓' : '✗'} ${s.label}`;
      group.appendChild(opt);
    }
    testCaseSelect.appendChild(group);
  }

  if (cases.length) {
    const group = document.createElement('optgroup');
    group.label = 'FK cross-check';
    for (const c of cases) {
      const opt = document.createElement('option');
      opt.value = c.id;
      opt.textContent = `${c.passed ? '✓' : '✗'} ${c.id}`;
      group.appendChild(opt);
    }
    testCaseSelect.appendChild(group);
  }

  testCaseSelect.disabled = false;
}

function applyTestCase(caseId) {
  currentScenarioId = null;
  const fixture = viewerReport?.fixtures?.[currentFixtureId];
  const c = fixture?.cases?.find((x) => x.id === caseId);
  if (!c) return;
  clearObstacles();
  applyJointState(c.joints);
  trajectory = fkCaseTrajectory;
  trajectoryTimes = null;
  playBtn.disabled = !trajectory?.length;
  rebuildTcpPath();
  const err = c.positionErrorM?.toFixed?.(4) ?? '?';
  const suiteNote = trajectory?.length ? ` · ${trajectory.length} FK poses` : '';
  setDropState(`${c.id} · Δ${err}m${suiteNote}`, c.passed ? 'ok' : 'err');
  setStatus(`[${c.passed ? 'PASS' : 'FAIL'} · ${c.id} · Δ${err}m]`, c.passed ? 'ok' : 'err');
}

function applyScenario(scenarioId) {
  currentScenarioId = scenarioId;
  const fixture = viewerReport?.fixtures?.[currentFixtureId];
  const s = fixture?.scenarios?.find((x) => x.id === scenarioId);
  if (!s) return;
  renderObstacles(s.obstacles);
  trajectory = (s.points ?? []).map((p) => p.joints);
  trajectoryTimes = (s.points ?? []).map((p) => p.timeSeconds ?? null);
  playBtn.disabled = !trajectory.length;
  if (trajectory.length) {
    applyJointState(trajectory[0]);
    rebuildTcpPath();
    setDropState(`${trajectory.length} planned waypoints · ${s.planner}`, s.passed ? 'ok' : 'err');
  }
  setStatus(`[${s.passed ? 'PASS' : 'FAIL'} · ${s.label}]`, s.passed ? 'ok' : 'err');
}

function onTestCaseChange() {
  const value = testCaseSelect.value;
  if (!value) return;
  stopPlayback();
  if (value.startsWith('scenario:')) applyScenario(value.slice('scenario:'.length));
  else applyTestCase(value);
}

function loadReportTrajectory(fixtureId) {
  const fixture = viewerReport?.fixtures?.[fixtureId];
  const scenarios = fixture?.scenarios ?? [];
  const cases = fixture?.cases ?? [];

  populateTestCases(fixtureId);
  fkCaseTrajectory = cases.length ? cases.map((c) => c.joints) : null;

  const defaultScenario = scenarios[0];
  if (defaultScenario) {
    testCaseSelect.value = `scenario:${defaultScenario.id}`;
    applyScenario(defaultScenario.id);
    return;
  }

  if (!cases.length) {
    fkCaseTrajectory = null;
    loadBundledPath({ id: fixtureId });
    return;
  }

  trajectory = fkCaseTrajectory;
  trajectoryTimes = null;
  playBtn.disabled = false;
  setDropState(`${cases.length} FK cross-check poses · Motus report`, viewerReport.summary?.failed ? 'err' : 'ok');

  const home = cases.find((c) => c.id.endsWith('_home')) ?? cases[0];
  defaultPose = home.joints;
  applyDefaultPose();
  clearObstacles();
  if (testCaseSelect.options.length) testCaseSelect.value = home.id;
  rebuildTcpPath();
}

async function ensureViewerPresets() {
  if (viewerPresets) return viewerPresets;
  const res = await fetch(PRESETS_URL);
  if (!res.ok) throw new Error(`Failed to load viewer presets (${res.status})`);
  viewerPresets = await res.json();
  return viewerPresets;
}

function getCurrentJointState() {
  const state = {};
  if (!robot) return state;
  for (const name of actuatedJoints) {
    state[name] = robot.joints[name]?.angle ?? 0;
  }
  return state;
}

function syncSliders() {
  if (!robot) return;
  jointsPanel.querySelectorAll('.joint-row').forEach((row) => {
    const name = row.dataset.joint;
    const joint = robot.joints[name];
    if (!joint) return;
    const q = joint.angle ?? 0;
    const slider = row.querySelector('input[type="range"]');
    const val = row.querySelector('.joint-val');
    if (slider) slider.value = q;
    if (val) val.textContent = q.toFixed(2);
  });
}

function resolveJointName(name) {
  if (!robot?.joints[name]) {
    const withSuffix = `${name}_joint`;
    if (robot.joints[withSuffix]) return withSuffix;
    if (name.endsWith('_joint')) {
      const short = name.slice(0, -'_joint'.length);
      if (robot.joints[short]) return short;
    }
    return null;
  }
  return name;
}

function applyJointState(state) {
  if (!robot || state == null) return;
  if (Array.isArray(state)) {
    actuatedJoints.forEach((name, i) => {
      if (i < state.length) robot.setJointValue(name, state[i]);
    });
  } else {
    for (const [name, q] of Object.entries(state)) {
      const resolved = resolveJointName(name);
      if (resolved) robot.setJointValue(resolved, q);
    }
  }
  robot.updateMatrixWorld(true);
  syncSliders();
}

function applyDefaultPose() {
  applyJointState(defaultPose);
}

function rebuildTcpPath() {
  disposePathLine();
  if (!robot || !trajectory?.length) return;

  const spec = FIXTURES.find((f) => f.id === currentFixtureId);
  const tip = spec ? robot.links[spec.tip] : null;
  if (!tip) return;

  const saved = getCurrentJointState();
  const coords = [];

  for (const point of trajectory) {
    applyJointState(point);
    robot.updateMatrixWorld(true);
    const p = new THREE.Vector3();
    tip.getWorldPosition(p);
    coords.push(p.x, p.y, p.z);
  }

  applyJointState(saved);

  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.Float32BufferAttribute(coords, 3));
  pathLine = new THREE.Line(
    geom,
    new THREE.LineBasicMaterial({
      color: SCENE_THEMES[currentTheme].path,
      transparent: true,
      opacity: 0.85,
    }),
  );
  // Tip positions are scene-world coords; line must not be under the Z-up correction group.
  scene.add(pathLine);
}

function loadBundledPath(spec) {
  const preset = viewerPresets?.[spec.id];
  const points = preset?.demoPath?.points ?? [];
  if (!points.length) {
    trajectory = null;
    trajectoryTimes = null;
    playBtn.disabled = true;
    setDropState('Drop Motus trajectory JSON export');
    dropZone.dataset.state = '';
    return;
  }
  trajectory = points.map((p) => mapTrajectoryPoint(p));
  trajectoryTimes = points.map((p) => p.timeSeconds ?? null);
  playBtn.disabled = false;
  setDropState(`${points.length} points · bundled demo`, 'ok');
  rebuildTcpPath();
}

async function loadFixture(id) {
  const seq = ++loadFixtureSeq;
  const spec = FIXTURES.find((f) => f.id === id) ?? FIXTURES[0];
  currentFixtureId = spec.id;
  stopPlayback();
  trajectory = null;
  trajectoryTimes = null;
  fkCaseTrajectory = null;
  exportJointNames = null;
  playBtn.disabled = true;
  setDropState('Drop Motus trajectory JSON export');
  dropZone.dataset.state = '';

  fixtureHero.textContent = spec.label ?? spec.id.replace(/_/g, ' ');
  setStatus('[LOADING]', 'busy');

  try {
    await ensureViewerPresets();
    if (seq !== loadFixtureSeq) return;
    await ensureViewerReport();
    if (seq !== loadFixtureSeq) return;
    disposeRobot();
    const base = spec.dir ? `${FIXTURE_BASE}${spec.dir}/` : FIXTURE_BASE;
    const url = `${base}${spec.file}`;
    loader.workingPath = base;
    robot = await loader.loadAsync(url);
    if (seq !== loadFixtureSeq) return;
    world.add(robot);
    styleRobotMaterials();
    buildJointSliders();

    const preset = viewerPresets[spec.id];
    const fixtureReport = viewerReport?.fixtures?.[spec.id];
    if (!fixtureReport) {
      defaultPose = preset?.defaultPose ?? {};
      applyDefaultPose();
      loadBundledPath(spec);
      populateTestCases(spec.id);
    } else {
      loadReportTrajectory(spec.id);
    }
    fitCamera();
    const reportNote = fixtureReport ? ' · test report' : '';
    setStatus(`[READY · ${spec.file}${reportNote}]`, 'ok');
  } catch (err) {
    if (seq !== loadFixtureSeq) return;
    setStatus(`[ERROR · ${err.message}]`, 'err');
    console.error(err);
  }
}

function applyJointVector(positions) {
  applyJointState(positions);
}

function resetJoints() {
  stopPlayback();
  if (!robot) return;
  applyDefaultPose();
  setStatus('[HOME POSE]', 'ok');
}

function loadTrajectoryJson(text) {
  const data = JSON.parse(text);
  exportJointNames = data.jointNames ?? null;
  const points = data.points ?? [];
  if (!points.length) throw new Error('Trajectory has no points');
  trajectory = points.map((p) => mapTrajectoryPoint(p));
  trajectoryTimes = points.map((p) => p.timeSeconds ?? null);
  playBtn.disabled = false;
  setDropState(`${points.length} points loaded`, 'ok');
  rebuildTcpPath();
}

function playbackDelayMs(index) {
  if (!trajectoryTimes || index <= 0) return 900;
  const prev = trajectoryTimes[index - 1];
  const cur = trajectoryTimes[index];
  if (prev == null || cur == null) return 900;
  return Math.max(40, (cur - prev) * 1000);
}

function playTrajectory() {
  if (!trajectory?.length) return;
  if (isPlaying) {
    stopPlayback();
    setStatus('[PAUSED]', 'ok');
    return;
  }
  isPlaying = true;
  playBtn.textContent = 'Stop playback';
  let i = 0;
  const tick = () => {
    if (!isPlaying) return;
    const fixture = viewerReport?.fixtures?.[currentFixtureId];
    if (currentScenarioId) {
      applyJointState(trajectory[i]);
      setStatus(`[PLAYING · ${i + 1}/${trajectory.length} · plan]`, 'busy');
    } else {
      const caseEntry = fixture?.cases?.[i];
      if (caseEntry) {
        testCaseSelect.value = caseEntry.id;
        applyJointState(caseEntry.joints);
        const err = caseEntry.positionErrorM?.toFixed?.(4) ?? '?';
        setStatus(`[PLAYING · ${i + 1}/${trajectory.length} · ${caseEntry.id} · Δ${err}m]`, 'busy');
      } else {
        applyJointVector(trajectory[i]);
        setStatus(`[PLAYING · ${i + 1}/${trajectory.length}]`, 'busy');
      }
    }
    const next = (i + 1) % trajectory.length;
    const delay = playbackDelayMs(next === 0 ? 1 : next);
    i = next;
    playTimer = setTimeout(tick, delay);
  };
  tick();
}

function handleTrajectoryFile(file) {
  file.text().then((text) => {
    loadTrajectoryJson(text);
    setStatus(`[TRAJECTORY · ${file.name}]`, 'ok');
  }).catch((err) => {
    setDropState(err.message, 'err');
    setStatus('[IMPORT FAILED]', 'err');
  });
}

fixtureSelect.addEventListener('change', () => loadFixture(fixtureSelect.value));
testCaseSelect.addEventListener('change', onTestCaseChange);
resetBtn.addEventListener('click', resetJoints);
recenterBtn.addEventListener('click', () => {
  fitCamera();
  setStatus('[RECENTERED]', 'ok');
});
playBtn.addEventListener('click', playTrajectory);
themeBtn.addEventListener('click', () => {
  applyTheme(currentTheme === 'light' ? 'dark' : 'light');
});

dropZone.addEventListener('dragover', (e) => {
  e.preventDefault();
  dropZone.classList.add('drag');
});
dropZone.addEventListener('dragleave', () => dropZone.classList.remove('drag'));
dropZone.addEventListener('drop', (e) => {
  e.preventDefault();
  dropZone.classList.remove('drag');
  const file = e.dataTransfer?.files?.[0];
  if (file) handleTrajectoryFile(file);
});
dropZone.addEventListener('click', () => {
  const input = document.createElement('input');
  input.type = 'file';
  input.accept = '.json,application/json';
  input.addEventListener('change', () => {
    if (input.files?.[0]) handleTrajectoryFile(input.files[0]);
  });
  input.click();
});

window.addEventListener('resize', resize);
initTheme();
resize();

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

fixtureSelect.value = 'ur10e';
ensureViewerReport().then(() => loadFixture('ur10e'));
