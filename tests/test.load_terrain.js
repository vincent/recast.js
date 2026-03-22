import { describe, it, beforeAll } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from './settings.js';

const require = createRequire(import.meta.url);
const Recast = require('../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;

describe('load terrain', () => {
  let recast;

  beforeAll(async () => {
    recast = await Recast();
    settings(recast);
  });

  it('loads an .obj file and builds tiled navmesh', () => new Promise((resolve) => {
    recast.OBJLoader(path.join(testsDir, 'nav_test.obj'), function() {
      recast.buildTiled();
      resolve();
    });
  }));
});
