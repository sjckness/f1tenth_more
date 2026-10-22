import { defineConfig } from 'vite';
import preact from '@preact/preset-vite';

// The gateway serves dist/ (www_dir) on the same origin as /api and /ws.
// `npm run dev` proxies both to a gateway on 127.0.0.1:8770 (--mock on linus).
export default defineConfig({
  plugins: [preact()],
  build: { outDir: 'dist', assetsDir: 'assets', sourcemap: false, target: 'safari15' },
  server: {
    proxy: {
      '/api': 'http://127.0.0.1:8770',
      '/ws': { target: 'ws://127.0.0.1:8770', ws: true },
    },
  },
});
