import { describe, it, expect, beforeAll } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;

describe('query', () => {
  /** @type {import('../../lib/recast.js').RecastModule} */
  let recast;

  beforeAll(async () => {
    recast = await Recast();
    settings(recast);
    await new Promise((resolve) => {
      recast.OBJLoader(path.join(testsDir, '../fixtures/nav_test.obj'), function () {
        recast.buildTiled();
        resolve();
      });
    });
  });

  it('queryPolygonsAsync returns an array of polygon objects', async () => {
    // queryPolygonsAsync returns Array<{ ref: number, vertices: Vec3[] }>
    // (the d.ts says number[] but the C++ emits JSON objects via recast_generic_callback_string)
    const polys = await recast.queryPolygonsAsync(0, 0, 0, 10, 10, 10, 100);
    expect(Array.isArray(polys)).toBe(true);
    expect(polys.length).toBeGreaterThan(0);
    expect(typeof polys[0].ref).toBe('number');
    expect(Array.isArray(polys[0].vertices)).toBe(true);
  });

  it('setPolyFlags sets flags on all polygons in a bounding box', async () => {
    // No getter exists, so we verify via no-throw and re-query if flags are exposed
    expect(() => {
      recast.setPolyFlags(0, 0, 0, 10, 10, 10, recast.FLAG_SWIM);
    }).not.toThrow();

    // Restore walk flag so subsequent tests are unaffected
    expect(() => {
      recast.setPolyFlags(0, 0, 0, 10, 10, 10, recast.FLAG_WALK);
    }).not.toThrow();
  });

  it('setPolyFlagsByRef sets flags on a specific polygon reference', async () => {
    const polys = await recast.queryPolygonsAsync(0, 0, 0, 10, 10, 10, 10);
    expect(polys.length).toBeGreaterThan(0);

    const ref = polys[0].ref;
    expect(() => {
      recast.setPolyFlagsByRef(ref, recast.FLAG_DOOR);
    }).not.toThrow();

    // Restore
    expect(() => {
      recast.setPolyFlagsByRef(ref, recast.FLAG_WALK);
    }).not.toThrow();
  });
});
