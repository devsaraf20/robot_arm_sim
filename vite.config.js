import { defineConfig } from 'vite';

// Three.js is ~600 KB minified; that's expected, so raise the warning threshold.
export default defineConfig({
  build: { chunkSizeWarningLimit: 800 },
});
