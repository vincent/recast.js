import { describe, it, expect, beforeAll } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;

describe('crowd', () => {
  /** @type {import('../../lib/recast.js').RecastModule} */
  let recast;

  beforeAll(async () => {
    recast = await Recast();
    settings(recast);
    await new Promise((resolve) => {
      recast.OBJLoader(path.join(testsDir, '../fixtures/nav_test.obj'), function() {
        recast.buildTiled();
        recast.initCrowd(1000, 1.0);
        resolve();
      });
    });
  });

  it('manages the crowd', async () => {
    const [pt1x, pt1y, pt1z] = await recast.getRandomPointAsync();

    expect(typeof pt1x).toBe('number');
    expect(typeof pt1y).toBe('number');
    expect(typeof pt1z).toBe('number');

    const id = recast.addAgent({
      position: { x: pt1x, y: pt1y, z: pt1z },
      radius: 0.5,
      height: 0.8,
      maxAcceleration: 1.0,
      maxSpeed: 2.0,
      updateFlags: 0,
      separationWeight: 10.0
    });

    expect(typeof id).toBe('number');

    await new Promise((resolve) => {
      recast.vent.once('update', function(agents) {
        expect(agents).toBeTruthy();
        expect(agents.length).toBe(1);
        resolve();
      });

      recast.crowdUpdate(1.0);
      recast.crowdGetActiveAgents();
    });
  });
});
