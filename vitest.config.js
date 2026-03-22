import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    testTimeout: 120000,
    hookTimeout: 60000,
    include: ['tests/test.*.js'],
    exclude: ['tests/test.init.js'],
    // Run test files sequentially to avoid shared state / WASM memory issues
    pool: 'threads',
    includeTaskLocation: true,
    poolOptions: {
      threads: { singleThread: true }
    }
  }
});
