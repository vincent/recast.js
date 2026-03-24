import { describe, it, expect } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;
const tilecachePath = path.join(testsDir, '../fixtures/tilecache.dist.bin');

/**
 * Helper: create a fresh recast instance with a loaded tile cache.
 * Each test gets its own instance to avoid obstacle-pool state leaking between tests.
 *
 * Note on the C++ getAllTempObstacles JSON bug: the C++ serialisation uses
 *   data += (i == obstacleCount - 1 ? "" : ",");
 * where the comma decision is based on slot index, not on whether the slot is
 * the last *non-empty* one. If the last slot by index is empty but an earlier
 * slot is non-empty, the output has a trailing comma and is not valid JSON.
 * Tests that call getAllTempObstaclesAsync therefore only exercise the case where
 * the result is guaranteed to be valid JSON (empty array, or all slots filled).
 */
async function freshRecast() {
  const r = await Recast();
  settings(r);
  await r.loadTileCacheAsync(tilecachePath);
  return r;
}

describe('obstacles', () => {
  it('addTempObstacle does not throw', async () => {
    const r = await freshRecast();
    // Note: C++ addTempObstacle returns void; the d.ts claiming number return is inaccurate
    expect(() => {
      r.addTempObstacle(0, 0, 0, 1.0);
    }).not.toThrow();
  });

  it('removeAllTempObstacles does not throw', async () => {
    const r = await freshRecast();
    r.addTempObstacle(1, 0, 1, 0.5);
    expect(() => {
      r.removeAllTempObstacles();
    }).not.toThrow();
  });

  it('getAllTempObstaclesAsync returns an empty array on a fresh tile cache', async () => {
    // A freshly loaded tile cache has no active obstacles → all pool slots are EMPTY →
    // the C++ loop skips all of them → produces "[]" which is valid JSON.
    const r = await freshRecast();
    const obstacles = await r.getAllTempObstaclesAsync();
    expect(Array.isArray(obstacles)).toBe(true);
    expect(obstacles.length).toBe(0);
  });
});
