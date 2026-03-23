import { describe, it, expect, beforeAll } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;

describe('pathfinding', () => {
  /** @type {import('../../lib/recast.js').RecastModule} */
  let recast;

  beforeAll(async () => {
    recast = await Recast();
    settings(recast);
    await new Promise((resolve) => {
      recast.OBJLoader(path.join(testsDir, '../fixtures/nav_test.obj'), function() {
        recast.buildTiled();
        resolve();
      });
    });
  });

  it('finds a random point', async () => {
    const { x, y, z } = await recast.getRandomPointAsync();
    expect(typeof x).toBe('number');
    expect(typeof y).toBe('number');
    expect(typeof z).toBe('number');
  });

  it('finds the nearest point', async () => {
    const extend = 3;
    const ext = { x: extend, y: extend, z: extend };
    const { x, y, z } = await recast.findNearestPointAsync({ x: 0, y: 0, z: 0 }, ext);
    expect(typeof x).toBe('number');
    expect(typeof y).toBe('number');
    expect(typeof z).toBe('number');
  });

  it('finds the nearest poly', async () => {
    const extend = 3;
    const ext = { x: extend, y: extend, z: extend };
    const polygon = await recast.findNearestPolyAsync({ x: 0, y: 0, z: 0 }, ext);
    expect(polygon.vertices).toBeTruthy();
    expect(typeof polygon.vertices.length).not.toBe('undefined');
  });

  it('finds a path between two points', async () => {
    const extend = 3;
    const ext = { x: extend, y: extend, z: extend };
    const pt1 = await recast.getRandomPointAsync();
    const pt2 = await recast.findNearestPointAsync({ x: 0, y: 0, z: 0 }, ext);
    const path = await recast.findPathAsync(pt1, pt2, 1000);
    expect(path).toBeTruthy();
    expect(typeof path.length).not.toBe('undefined');
  });
});
