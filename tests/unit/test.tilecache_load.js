import { describe, it, expect } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;

describe('tilecache load', () => {
  it('loads a tiled cache and queries it', async () => {
    const recast = await Recast();
    settings(recast);

    await recast.loadTileCacheAsync(path.join(testsDir, '../fixtures/tilecache.dist.bin'));

    const pt1 = await recast.getRandomPointAsync();
    expect(typeof pt1.x).toBe('number');
    expect(typeof pt1.y).toBe('number');
    expect(typeof pt1.z).toBe('number');

    const ext = { x: 10, y: 10, z: 10 };
    const pt2 = await recast.findNearestPointAsync(pt1, ext);
    expect(typeof pt2.x).toBe('number');
    expect(typeof pt2.y).toBe('number');
    expect(typeof pt2.z).toBe('number');
  });
});
