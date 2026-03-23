import { describe, it, expect, beforeAll } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;

describe('pathfinding', () => {
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
    const [x, y, z] = await recast.getRandomPointAsync();
    expect(typeof x).toBe('number');
    expect(typeof y).toBe('number');
    expect(typeof z).toBe('number');
  });

  it('finds the nearest point', async () => {
    const extend = 3;
    const [x, y, z] = await recast.findNearestPointAsync(0, 0, 0, extend, extend, extend);
    expect(typeof x).toBe('number');
    expect(typeof y).toBe('number');
    expect(typeof z).toBe('number');
  });

  it('finds the nearest poly', async () => {
    const extend = 3;
    const polygon = await recast.findNearestPolyAsync(0, 0, 0, extend, extend, extend);
    expect(polygon.vertices).toBeTruthy();
    expect(typeof polygon.vertices.length).not.toBe('undefined');
  });

  it('finds a path between two points', async () => {
    const extend = 3;
    const [pt1x, pt1y, pt1z] = await recast.getRandomPointAsync();
    const [pt2x, pt2y, pt2z] = await recast.findNearestPointAsync(0, 0, 0, extend, extend, extend);
    const path = await recast.findPathAsync(pt1x, pt1y, pt1z, pt2x, pt2y, pt2z, 1000);
    expect(path).toBeTruthy();
    expect(typeof path.length).not.toBe('undefined');
  });
});
