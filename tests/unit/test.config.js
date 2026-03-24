import { describe, it, expect, beforeAll } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;

describe('config', () => {
  /** @type {import('../../lib/recast.js').RecastModule} */
  let recast;

  beforeAll(async () => {
    recast = await Recast();
  });

  it('settings() applies all parameters as a batch', async () => {
    recast.settings({
      cellSize: 0.3,
      cellHeight: 0.2,
      agentHeight: 0.8,
      agentRadius: 0.2,
      agentMaxClimb: 4.0,
      agentMaxSlope: 30.0,
    });

    // Confirm build succeeds with batch-applied settings
    const type = await new Promise((resolve) => {
      recast.OBJLoader(path.join(testsDir, '../fixtures/nav_test.obj'), function () {
        recast.events.once('built', resolve);
        recast.buildTiled();
      });
    });

    expect(type).toBe('tiled');
  });

  it('FLAG_* constants have correct bitmask values', () => {
    expect(recast.FLAG_WALK).toBe(0x01);
    expect(recast.FLAG_SWIM).toBe(0x02);
    expect(recast.FLAG_DOOR).toBe(0x04);
    expect(recast.FLAG_JUMP).toBe(0x08);
    expect(recast.FLAG_DISABLED).toBe(0x10);
    expect(recast.FLAG_ALL).toBe(0xffff);
  });

  it('CROWD_* constants have correct bitmask values', () => {
    expect(recast.CROWD_ANTICIPATE_TURNS).toBe(1);
    expect(recast.CROWD_OBSTACLE_AVOIDANCE).toBe(2);
    expect(recast.CROWD_SEPARATION).toBe(4);
    expect(recast.CROWD_OPTIMIZE_VIS).toBe(8);
    expect(recast.CROWD_OPTIMIZE_TOPO).toBe(16);
  });

  it('navmeshType is set after build and built event fires', async () => {
    const recast2 = await Recast();
    settings(recast2);

    const type = await new Promise((resolve) => {
      recast2.OBJLoader(path.join(testsDir, '../fixtures/nav_test.obj'), function () {
        recast2.events.once('built', resolve);
        recast2.buildTiled();
      });
    });

    expect(type).toBe('tiled');
    expect(recast2.navmeshType).toBe('tiled');
  });
});
