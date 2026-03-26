import { describe, it, expect } from 'vitest';
import { createRequire } from 'module';
import { readFileSync } from 'fs';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {(opts?: import('../../lib/recast.js').RecastModuleOptions) => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;
const objPath = path.join(testsDir, '../fixtures/nav_test.obj');

describe('logger', () => {
  it('silent mode: no output when no logger is set', async () => {
    const r = await Recast();
    settings(r);
    await r.OBJLoaderAsync(objPath);
    await r.buildSoloAsync();
    // No assertions needed — just verifying no errors are thrown
  });

  it('logger receives info messages during init, load, and build', async () => {
    const messages = [];
    const logger = {
      debug: (m) => messages.push(['debug', m]),
      info:  (m) => messages.push(['info',  m]),
      error: (m) => messages.push(['error', m]),
    };

    const r = await Recast({ logger });
    settings(r);

    const infoMessages = messages.filter(([l]) => l === 'info').map(([, m]) => m);
    expect(infoMessages).toContain('Recast WASM module initialized');

    await r.OBJLoaderAsync(objPath);

    const afterLoad = messages.filter(([l]) => l === 'info').map(([, m]) => m);
    expect(afterLoad.some(m => m.startsWith('obj loader: loading'))).toBe(true);
    expect(afterLoad).toContain('loading obj geometry');
    expect(afterLoad).toContain('obj geometry loaded');

    await r.buildSoloAsync();

    const afterBuild = messages.filter(([l]) => l === 'info').map(([, m]) => m);
    expect(afterBuild).toContain('buildSolo: starting');
    expect(afterBuild).toContain('buildSolo: complete');
  });

  it('logger receives messages during build (info + any debug from BuildContext)', async () => {
    const messages = [];
    const logger = {
      debug: (m) => messages.push(['debug', m]),
      info:  (m) => messages.push(['info',  m]),
      error: (m) => messages.push(['error', m]),
    };

    const r = await Recast({ logger });
    const countBefore = messages.length;
    settings(r);
    await r.OBJLoaderAsync(objPath);
    await r.buildSoloAsync();

    // At minimum, buildSolo: starting and buildSolo: complete should appear
    const afterBuild = messages.slice(countBefore);
    expect(afterBuild.length).toBeGreaterThan(0);
    const infoOrDebug = afterBuild.filter(([l]) => l === 'info' || l === 'debug');
    expect(infoOrDebug.length).toBeGreaterThan(0);
  });

  it('logger receives error when operations are called on unready state', async () => {
    const messages = [];
    const logger = {
      debug: (m) => messages.push(['debug', m]),
      info:  (m) => messages.push(['info',  m]),
      error: (m) => messages.push(['error', m]),
    };

    const r = await Recast({ logger });
    // removeTempObstacle before building logs "TileCache is not ready" synchronously
    r.removeTempObstacle(0, 0, 0, 1, 0, 1);

    const errors = messages.filter(([l]) => l === 'error').map(([, m]) => m);
    expect(errors.some(m => m.includes('not ready'))).toBe(true);
  });

  it('logger can be set after initialization', async () => {
    const r = await Recast();
    const messages = [];
    r.logger = {
      info: (m) => messages.push(m),
    };
    settings(r);
    await r.OBJLoaderAsync(objPath);

    expect(messages.some(m => m.startsWith('obj loader: loading'))).toBe(true);
  });

  it('logger can be removed after initialization', async () => {
    const messages = [];
    const r = await Recast({ logger: { info: (m) => messages.push(m) } });
    r.logger = null;
    settings(r);
    await r.OBJLoaderAsync(objPath);

    // After removing logger, no new messages should be added
    expect(messages.length).toBe(1); // only the init message captured before removal
  });

  it('partial logger (only info) does not throw when debug/error are absent', async () => {
    const r = await Recast({ logger: { info: () => {} } });
    settings(r);
    await r.OBJLoaderAsync(objPath);
    await r.buildSoloAsync();
    // No errors thrown despite missing debug/error methods
  });
});
