import { describe, it, beforeAll } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;

describe('load terrain', () => {
  /** @type {import('../../lib/recast.js').RecastModule} */
  let recast;

  beforeAll(async () => {
    recast = await Recast();
    settings(recast);
  });

  it('loads an .obj file and builds tiled navmesh', () => new Promise((resolve) => {
    recast.OBJLoader(path.join(testsDir, '../fixtures/nav_test.obj'), function() {
      recast.buildTiled();
      resolve();
    });
  }));
});
