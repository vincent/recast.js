import { describe, it } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import fs from 'fs';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;

describe('tilecache save', () => {
  it('saves a tiled cache', async () => {
    const recast = await Recast();
    settings(recast);

    await new Promise((resolve) => {
      recast.OBJLoader(path.join(testsDir, '../fixtures/nav_test.obj'), function() {
        recast.buildTiled();
        resolve();
      });
    });

    const [error, serialized] = await recast.saveTileCacheAsync('./tilecache.bin');

    const buffer = Buffer.from(serialized);
    await fs.promises.writeFile(path.join(testsDir, '../fixtures/tilecache.bin'), buffer);
  });
});
