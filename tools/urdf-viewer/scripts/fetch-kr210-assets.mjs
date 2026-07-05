#!/usr/bin/env node
/**
 * Download KR210 R3100 ultra visual meshes from kuka_robot_descriptions (Apache-2.0).
 * Run: node tools/urdf-viewer/scripts/fetch-kr210-assets.mjs
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const outDir = path.resolve(__dirname, '../../../tests/fixtures/kr210_r3100_ultra');
const meshDir = path.join(outDir, 'meshes', 'visual');
const baseUrl =
  'https://raw.githubusercontent.com/kroshu/kuka_robot_descriptions/master/kuka_quantec_support/meshes/kr210_r3100_ultra/visual';

const meshes = [
  'base_link.dae',
  'link_1.dae',
  'link_2.dae',
  'link_3.dae',
  'link_4.dae',
  'link_5.dae',
  'link_6.dae',
];

function whitenKukaMeshes(meshDir) {
  const brandColors = [
    '1 0.09989873 0 1',
    '1 1 0 1',
  ];
  for (const name of fs.readdirSync(meshDir)) {
    if (!name.endsWith('.dae')) continue;
    const file = path.join(meshDir, name);
    let text = fs.readFileSync(file, 'utf8');
    for (const color of brandColors) {
      text = text.replaceAll(`<color sid="diffuse">${color}</color>`, '<color sid="diffuse">1 1 1 1</color>');
    }
    text = text.replace(
      /<color sid="diffuse">([\d.]+) ([\d.]+) ([\d.]+) 1<\/color>/g,
      (match, r, g, b) => {
        const rf = Number(r);
        const gf = Number(g);
        const bf = Number(b);
        if (rf > 0.85 && gf < 0.25 && bf < 0.15) return '<color sid="diffuse">1 1 1 1</color>';
        if (rf > 0.95 && gf > 0.95 && bf < 0.1) return '<color sid="diffuse">1 1 1 1</color>';
        return match;
      },
    );
    fs.writeFileSync(file, text);
  }
}

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

whitenKukaMeshes(meshDir);

console.log(`Meshes in ${meshDir}`);
