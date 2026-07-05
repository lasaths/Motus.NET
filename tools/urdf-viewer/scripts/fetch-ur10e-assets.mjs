#!/usr/bin/env node
/**
 * Download UR10e visual meshes from Universal_Robots_ROS2_Description (BSD-3-Clause).
 * Run: node tools/urdf-viewer/scripts/fetch-ur10e-assets.mjs
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const outDir = path.resolve(__dirname, '../../../tests/fixtures/ur10e');
const meshDir = path.join(outDir, 'meshes', 'visual');
const baseUrl =
  'https://raw.githubusercontent.com/UniversalRobots/Universal_Robots_ROS2_Description/rolling/meshes/ur10e/visual';

const meshes = [
  'base.dae',
  'shoulder.dae',
  'upperarm.dae',
  'forearm.dae',
  'wrist1.dae',
  'wrist2.dae',
  'wrist3.dae',
];

fs.mkdirSync(meshDir, { recursive: true });

for (const name of meshes) {
  const dest = path.join(meshDir, name);
  if (fs.existsSync(dest) && fs.statSync(dest).size > 0) {
    console.log(`skip ${name}`);
    continue;
  }
  console.log(`fetch ${name}`);
  const res = await fetch(`${baseUrl}/${name}`);
  if (!res.ok) throw new Error(`Failed ${name}: ${res.status}`);
  const buf = Buffer.from(await res.arrayBuffer());
  fs.writeFileSync(dest, buf);
}

console.log(`Meshes in ${meshDir}`);
