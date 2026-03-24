import { describe, it, expect } from 'vitest';
import { createRequire } from 'module';
import { readFileSync } from 'fs';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;
const objPath = path.join(testsDir, '../fixtures/nav_test.obj');

describe('build', () => {
  it('OBJLoaderAsync loads geometry from a file path', async () => {
    const r = await Recast();
    settings(r);
    await r.OBJLoaderAsync(objPath);
    const type = await r.buildTiledAsync();
    expect(type).toBe('tiled');
  });

  it('OBJDataLoader loads geometry from string content via callback', async () => {
    const r = await Recast();
    settings(r);
    const data = readFileSync(objPath);
    await new Promise((resolve) => {
      r.OBJDataLoader(data, resolve);
    });
    const type = await r.buildSoloAsync();
    expect(type).toBe('solo');
  });

  it('OBJDataLoaderAsync loads geometry from string content', async () => {
    const r = await Recast();
    settings(r);
    const data = readFileSync(objPath);
    await r.OBJDataLoaderAsync(data);
    const type = await r.buildSoloAsync();
    expect(type).toBe('solo');
  });

  it('buildSolo emits built event with type solo', async () => {
    const r = await Recast();
    settings(r);
    await r.OBJLoaderAsync(objPath);
    const type = await new Promise((resolve) => {
      r.events.once('built', resolve);
      r.buildSolo();
    });
    expect(type).toBe('solo');
    expect(r.navmeshType).toBe('solo');
  });

  it('buildSoloAsync resolves with navmesh type solo', async () => {
    const r = await Recast();
    settings(r);
    await r.OBJLoaderAsync(objPath);
    const type = await r.buildSoloAsync();
    expect(type).toBe('solo');
    expect(r.navmeshType).toBe('solo');
  });

  it('buildTiledAsync resolves with navmesh type tiled', async () => {
    const r = await Recast();
    settings(r);
    await r.OBJLoaderAsync(objPath);
    const type = await r.buildTiledAsync();
    expect(type).toBe('tiled');
    expect(r.navmeshType).toBe('tiled');
  });
});
