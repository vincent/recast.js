import { describe, it, expect } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from './settings.js';

const require = createRequire(import.meta.url);
const Recast = require('../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;

describe('tilecache load', () => {
  it('loads a tiled cache and queries it', async () => {
    const recast = await Recast();
    settings(recast);

    await recast.loadTileCacheAsync(path.join(testsDir, 'tilecache.dist.bin'));

    const [pt1x, pt1y, pt1z] = await recast.getRandomPointAsync();
    expect(typeof pt1x).toBe('number');
    expect(typeof pt1y).toBe('number');
    expect(typeof pt1z).toBe('number');

    const [pt2x, pt2y, pt2z] = await recast.findNearestPointAsync(pt1x, pt1y, pt1z, 10, 10, 10);
    expect(typeof pt2x).toBe('number');
    expect(typeof pt2y).toBe('number');
    expect(typeof pt2z).toBe('number');
  });
});
