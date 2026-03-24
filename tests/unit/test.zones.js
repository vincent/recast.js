import { describe, it, expect, beforeAll } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;

describe('zones', () => {
  /** @type {import('../../lib/recast.js').RecastModule} */
  let recast;
  let polyRef;
  // zone reference — tests in this block are intentionally stateful (sequential flag mutations)
  let zone;

  beforeAll(async () => {
    recast = await Recast();
    settings(recast);
    await new Promise((resolve) => {
      recast.OBJLoader(path.join(testsDir, '../fixtures/nav_test.obj'), function () {
        recast.buildTiled();
        resolve();
      });
    });

    // Get a polygon ref to use for the zone
    const polys = await recast.queryPolygonsAsync(0, 0, 0, 10, 10, 10, 10);
    expect(polys.length).toBeGreaterThan(0);
    polyRef = polys[0].ref;
  });

  it('setZones creates named zones in recast.zones', () => {
    recast.setZones({
      water: { refs: [polyRef], initialFlags: [recast.FLAG_SWIM] },
    });

    zone = recast.zones.water;
    expect(zone).toBeDefined();
    expect(zone.name).toBe('water');
    expect(zone.refs).toEqual([polyRef]);
    expect(zone.flags).toBe(recast.FLAG_SWIM);
  });

  it('Zone.isWalkable returns false when only FLAG_SWIM is set', () => {
    expect(zone.isWalkable()).toBe(false);
  });

  it('Zone.is returns true for a set flag and false for an unset flag', () => {
    expect(zone.is(recast.FLAG_SWIM)).toBe(true);
    expect(zone.is(recast.FLAG_WALK)).toBe(false);
  });

  it('Zone.setFlags adds a flag bit without removing others', () => {
    zone.setFlags(recast.FLAG_WALK);
    expect(zone.is(recast.FLAG_WALK)).toBe(true);
    expect(zone.is(recast.FLAG_SWIM)).toBe(true);
    expect(zone.flags & (recast.FLAG_WALK | recast.FLAG_SWIM)).toBe(recast.FLAG_WALK | recast.FLAG_SWIM);
  });

  it('Zone.clearFlags removes a specific flag bit', () => {
    zone.clearFlags(recast.FLAG_SWIM);
    expect(zone.is(recast.FLAG_SWIM)).toBe(false);
    expect(zone.is(recast.FLAG_WALK)).toBe(true);
  });

  it('Zone.toggleFlags flips a flag bit on and off', () => {
    zone.toggleFlags(recast.FLAG_DOOR);
    expect(zone.is(recast.FLAG_DOOR)).toBe(true);

    zone.toggleFlags(recast.FLAG_DOOR);
    expect(zone.is(recast.FLAG_DOOR)).toBe(false);
  });

  it('Zone.isWalkable returns true when FLAG_WALK is set', () => {
    // After the mutations above, FLAG_WALK should still be set
    expect(zone.isWalkable()).toBe(true);
  });

  it('Zone.syncFlags returns the zone instance for chaining', () => {
    const result = zone.syncFlags();
    expect(result).toBe(zone);
  });
});
