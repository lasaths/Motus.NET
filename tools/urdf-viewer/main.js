import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';

const FIXTURES = [
  { id: 'two_link', file: 'two_link.urdf', tip: 'tip_link', dir: '' },
  { id: 'prismatic_lift', file: 'prismatic_lift.urdf', tip: 'tip_link', dir: '' },
  { id: 'ur5e_collision', file: 'ur5e_collision.urdf', tip: 'tool0', dir: '' },
  { id: 'ur10e', file: 'ur10e.urdf', tip: 'tool0', dir: 'ur10e' },
  { id: 'ur10e_collision', file: 'ur10e_collision.urdf', tip: 'tool0', dir: '' },
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

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);

const camera = new THREE.PerspectiveCamera(48, 1, 0.01, 50);
camera.position.set(1.2, 0.9, 1.2);

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
viewport.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0.25, 0);
controls.enableDamping = true;
controls.dampingFactor = 0.08;

scene.add(new THREE.AmbientLight(0xffffff, 0.45));
const key = new THREE.DirectionalLight(0xffffff, 0.85);
key.position.set(2.5, 4, 2);
scene.add(key);
const fill = new THREE.DirectionalLight(0xe8e8e8, 0.25);
fill.position.set(-2, 1, -1);
scene.add(fill);

const grid = new THREE.GridHelper(2.5, 25, 0x333333, 0x1a1a1a);
scene.add(grid);

// URDF is Z-up; THREE.js is Y-up (same as urdf-viewer-element default up="+Z").
const world = new THREE.Group();
world.rotation.x = -Math.PI / 2;
scene.add(world);

const loader = new URDFLoader();
loader.parseCollision = true;
loader.parseVisual = true;
loader.workingPath = FIXTURE_BASE;

let robot = null;
let trajectory = null;
let playTimer = null;
let isPlaying = false;
let actuatedJoints = [];
let viewerPresets = null;
let defaultPose = {};
let pathLine = null;
let currentFixtureId = null;

const PRESETS_URL = `${FIXTURE_BASE}viewer_presets.json`;

function setStatus(text, state = '') {
  statusLine.textContent = text;
  statusLine.dataset.state = state;
}

function setDropState(hint, state = '') {
  dropHint.textContent = hint;
  dropZone.dataset.state = state;
}

for (const f of FIXTURES) {
  const opt = document.createElement('option');
  opt.value = f.id;
  opt.textContent = f.id.replace(/_/g, ' ');
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
  robot.traverse((c) => {
    if (!c.isMesh) return;
    const mats = Array.isArray(c.material) ? c.material : [c.material];
    mats.forEach((m) => {
      if (!m || m.map) return;
      m.color?.setHex(0xc8c8c8);
      m.metalness = 0.15;
      m.roughness = 0.65;
    });
  });
}

function fitCamera() {
  if (!robot) return;
  const box = new THREE.Box3().setFromObject(robot);
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
  playBtn.textContent = 'Play trajectory';
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

function applyJointState(state) {
  if (!robot || state == null) return;
  if (Array.isArray(state)) {
    actuatedJoints.forEach((name, i) => {
      if (i < state.length) robot.setJointValue(name, state[i]);
    });
  } else {
    for (const [name, q] of Object.entries(state)) {
      if (robot.joints[name]) robot.setJointValue(name, q);
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
    new THREE.LineBasicMaterial({ color: 0x5b9bf6, transparent: true, opacity: 0.85 }),
  );
  // Tip positions are scene-world coords; line must not be under the Z-up correction group.
  scene.add(pathLine);
}

function loadBundledPath(spec) {
  const preset = viewerPresets?.[spec.id];
  const points = preset?.demoPath?.points ?? [];
  if (!points.length) {
    trajectory = null;
    playBtn.disabled = true;
    setDropState('Drop Motus trajectory JSON export');
    dropZone.dataset.state = '';
    return;
  }
  trajectory = points.map((p) => mapTrajectoryPoint(p));
  playBtn.disabled = false;
  setDropState(`${points.length} points · bundled demo`, 'ok');
  rebuildTcpPath();
}

async function loadFixture(id) {
  const spec = FIXTURES.find((f) => f.id === id) ?? FIXTURES[0];
  currentFixtureId = spec.id;
  stopPlayback();
  trajectory = null;
  exportJointNames = null;
  playBtn.disabled = true;
  setDropState('Drop Motus trajectory JSON export');
  dropZone.dataset.state = '';

  fixtureHero.textContent = spec.id.replace(/_/g, ' ');
  setStatus('[LOADING]', 'busy');

  try {
    await ensureViewerPresets();
    disposeRobot();
    const base = spec.dir ? `${FIXTURE_BASE}${spec.dir}/` : FIXTURE_BASE;
    const url = `${base}${spec.file}`;
    loader.workingPath = base;
    robot = await loader.loadAsync(url);
    world.add(robot);
    styleRobotMaterials();
    buildJointSliders();

    const preset = viewerPresets[spec.id];
    defaultPose = preset?.defaultPose ?? {};
    applyDefaultPose();
    loadBundledPath(spec);
    fitCamera();
    setStatus(`[READY · ${spec.file}]`, 'ok');
  } catch (err) {
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
  playBtn.disabled = false;
  setDropState(`${points.length} points loaded`, 'ok');
  rebuildTcpPath();
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
    applyJointVector(trajectory[i]);
    setStatus(`[PLAYING · ${i + 1}/${trajectory.length}]`, 'busy');
    i = (i + 1) % trajectory.length;
    playTimer = setTimeout(tick, 80);
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
resetBtn.addEventListener('click', resetJoints);
recenterBtn.addEventListener('click', () => {
  fitCamera();
  setStatus('[RECENTERED]', 'ok');
});
playBtn.addEventListener('click', playTrajectory);

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
resize();

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

fixtureSelect.value = 'ur10e';
loadFixture('ur10e');
