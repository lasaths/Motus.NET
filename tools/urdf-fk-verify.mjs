#!/usr/bin/env node
/**
 * Headless FK oracle using urdf-loader. Reads JSON from stdin, writes tip-link pose to stdout.
 * Run from repo root after: npm ci --prefix tools/urdf-viewer
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { JSDOM } from './urdf-viewer/node_modules/jsdom/lib/api.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const viewerNodeModules = path.join(__dirname, 'urdf-viewer', 'node_modules');
const { default: URDFLoader } = await import(
  pathToFileURL(path.join(viewerNodeModules, 'urdf-loader', 'src', 'URDFLoader.js')).href
);
const THREE = await import(
  pathToFileURL(path.join(viewerNodeModules, 'three', 'build', 'three.module.js')).href
);

const dom = new JSDOM('<!DOCTYPE html><html><body></body></html>');
global.DOMParser = dom.window.DOMParser;
global.Document = dom.window.Document;
global.Element = dom.window.Element;

function readStdin() {
  return new Promise((resolve, reject) => {
    const chunks = [];
    process.stdin.setEncoding('utf8');
    process.stdin.on('data', (c) => chunks.push(c));
    process.stdin.on('end', () => resolve(chunks.join('')));
    process.stdin.on('error', reject);
  });
}

function tipPose(robot, tipLink) {
  robot.updateMatrixWorld(true);
  const link = robot.links[tipLink];
  if (!link) {
    throw new Error(`Tip link '${tipLink}' not found. Available: ${Object.keys(robot.links).join(', ')}`);
  }
  const position = new THREE.Vector3();
  const quaternion = new THREE.Quaternion();
  link.getWorldPosition(position);
  link.getWorldQuaternion(quaternion);
  const matrix = Array.from(link.matrixWorld.elements);
  return {
    position: { x: position.x, y: position.y, z: position.z },
    quaternion: { w: quaternion.w, x: quaternion.x, y: quaternion.y, z: quaternion.z },
    matrix,
  };
}

function resolveUrdfPath(repoRoot, urdfPath) {
  const full = path.isAbsolute(urdfPath) ? urdfPath : path.join(repoRoot, urdfPath);
  return path.normalize(full);
}

async function main() {
  const input = JSON.parse(await readStdin());
  const repoRoot = input.repoRoot
    ? path.resolve(input.repoRoot)
    : path.resolve(__dirname, '..');
  const urdfPath = resolveUrdfPath(repoRoot, input.urdfPath);
  const xml = fs.readFileSync(urdfPath, 'utf8');
  const loader = new URDFLoader();
  loader.workingPath = path.dirname(urdfPath) + path.sep;
  loader.parseCollision = false;
  loader.parseVisual = false;
  const robot = loader.parse(xml);

  const results = (input.cases ?? []).map((c) => {
    robot.setJointValues(c.joints ?? {});
    return { id: c.id ?? null, ...tipPose(robot, input.tipLink) };
  });

  process.stdout.write(JSON.stringify({ tipLink: input.tipLink, results }));
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
