import { defineConfig } from 'vite';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const fixturesDir = path.resolve(__dirname, '../../tests/fixtures');

const MIME = {
  '.urdf': 'application/xml',
  '.xml': 'application/xml',
  '.dae': 'model/vnd.collada+xml',
  '.stl': 'model/stl',
};

export default defineConfig({
  server: {
    fs: {
      allow: [path.resolve(__dirname, '../..')],
    },
  },
  plugins: [
    {
      name: 'motus-fixtures',
      configureServer(server) {
        server.middlewares.use('/tests/fixtures', (req, res, next) => {
          const rel = decodeURIComponent((req.url ?? '/').split('?')[0]);
          const filePath = path.normalize(path.join(fixturesDir, rel));
          if (!filePath.startsWith(fixturesDir) || !fs.existsSync(filePath) || fs.statSync(filePath).isDirectory()) {
            return next();
          }
          const ext = path.extname(filePath).toLowerCase();
          if (MIME[ext]) res.setHeader('Content-Type', MIME[ext]);
          fs.createReadStream(filePath).pipe(res);
        });
      },
    },
  ],
});
