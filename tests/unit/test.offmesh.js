import { describe, it, expect, beforeAll } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;
const objPath = path.join(testsDir, '../fixtures/nav_test.obj');

describe('offmesh connections', () => {
  /** @type {import('../../lib/recast.js').RecastModule} */
  let recast;

  beforeAll(async () => {
    recast = await Recast();
    settings(recast);
    // addOffMeshConnection MUST be called before build — links are baked in at build time
    recast.addOffMeshConnection(-6.7, -2.3, 27.2, 41.7, 7.9, 19.2, 3.0, true);
    await new Promise((resolve) => {
      recast.OBJLoader(objPath, function () {
        recast.events.once('built', resolve);
        recast.buildSolo();
      });
    });
  });

  it('addOffMeshConnection does not prevent a successful build', () => {
    expect(recast.navmeshType).toBe('solo');
  });

  it('path can be found after adding an off-mesh connection', async () => {
    // Use two known-navigable points from the fixture geometry.
    // The off-mesh link provides a shortcut between them.
    const ext = { x: 5, y: 5, z: 5 };
    const pt1 = await recast.findNearestPointAsync({ x: -6.7, y: -2.3, z: 27.2 }, ext);
    const pt2 = await recast.findNearestPointAsync({ x: 41.7, y: 7.9, z: 19.2 }, ext);

    const pathResult = await recast.findPathAsync(pt1, pt2, 100);
    expect(Array.isArray(pathResult)).toBe(true);
    expect(pathResult.length).toBeGreaterThan(0);
  });

  it('addOffMeshConnection accepts bidir=false for one-way links', async () => {
    const r = await Recast();
    settings(r);
    // One-way link (bidir = false)
    expect(() => {
      r.addOffMeshConnection(0, 0, 0, 5, 0, 5, 1.0, false);
    }).not.toThrow();
    // Build to confirm it does not break the navmesh
    const type = await new Promise((resolve) => {
      r.OBJLoader(objPath, function () {
        r.events.once('built', resolve);
        r.buildSolo();
      });
    });
    expect(type).toBe('solo');
  });
});
