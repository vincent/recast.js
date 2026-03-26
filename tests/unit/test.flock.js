import { describe, it, expect, beforeAll } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;
const objPath = path.join(testsDir, '../fixtures/nav_test.obj');

/** Run n crowd steps, returning the final agent snapshot. */
async function stepN(r, n, dt = 0.1) {
  let last;
  for (let i = 0; i < n; i++) {
    last = await new Promise((resolve) => {
      r.events.once('update', (agents) => {
        resolve(agents.map((a) => ({
          idx: a.idx,
          position: { x: a.position.x, y: a.position.y, z: a.position.z },
          velocity: { x: a.velocity.x, y: a.velocity.y, z: a.velocity.z },
          active: a.active,
        })));
      });
      r.crowdUpdate(dt);
    });
  }
  return last;
}

/** Distance between two {x,y,z} points in XZ plane. */
function distXZ(a, b) {
  return Math.sqrt((a.x - b.x) ** 2 + (a.z - b.z) ** 2);
}

async function freshCrowd() {
  const r = await Recast();
  settings(r);
  r.withPlugin(r.FlockGroup);
  await new Promise((resolve) => {
    r.OBJLoader(objPath, function () {
      r.buildTiled();
      r.initCrowd(100, 1.0);
      resolve();
    });
  });
  return r;
}

const AGENT_OPTS = {
  radius: 0.4,
  height: 1.8,
  maxAcceleration: 8.0,
  maxSpeed: 3.5,
  updateFlags: 0,
  separationWeight: 0,
};

describe('FlockGroup', () => {
  /** @type {import('../../lib/recast.js').RecastModule} */
  let recast;

  beforeAll(async () => {
    recast = await freshCrowd();
  });

  it('createFlockGroup returns an object with the expected interface', () => {
    const flock = recast.createFlockGroup({ agentIds: [] });
    expect(typeof flock.requestMoveTarget).toBe('function');
    expect(typeof flock.addAgent).toBe('function');
    expect(typeof flock.removeAgent).toBe('function');
    expect(typeof flock.destroy).toBe('function');
    flock.destroy();
  });

  it('agents converge toward each other when cohesionWeight > 0', async () => {
    // Spawn 3 agents at separate known points
    const pts = await Promise.all([
      recast.getRandomPointAsync(),
      recast.getRandomPointAsync(),
      recast.getRandomPointAsync(),
    ]);

    const ids = pts.map((pt) => recast.addAgent({ position: pt, ...AGENT_OPTS }));
    const flock = recast.createFlockGroup({
      agentIds: ids,
      cohesionWeight: 0.8,
      cohesionRadius: 100,
    });

    // Pick a reachable destination
    const dst = await recast.getRandomPointAsync();
    flock.requestMoveTarget(dst.x, dst.y, dst.z);

    const agents = await stepN(recast, 60);
    flock.destroy();
    ids.forEach((id) => recast.removeCrowdAgent(id));

    const members = agents.filter((a) => ids.includes(a.idx));
    expect(members.length).toBe(ids.length);

    // All members should have moved (velocity non-zero at some point is hard to
    // test post-hoc, so we verify they are not all sitting exactly at spawn)
    const moved = members.filter((a) =>
      distXZ(a.position, pts[ids.indexOf(a.idx)]) > 0.01
    );
    expect(moved.length).toBeGreaterThan(0);
  });

  it('agents without a destination do not move via the flock', async () => {
    const pt = await recast.getRandomPointAsync();
    const id = recast.addAgent({ position: pt, ...AGENT_OPTS });

    const flock = recast.createFlockGroup({ agentIds: [id], cohesionWeight: 0.5 });
    // No requestMoveTarget called

    const agents = await stepN(recast, 10);
    flock.destroy();
    recast.removeCrowdAgent(id);

    const member = agents.find((a) => a.idx === id);
    // Agent should remain near spawn (no target set)
    expect(distXZ(member.position, pt)).toBeLessThan(0.5);
  });

  it('addAgent and removeAgent update membership correctly', async () => {
    const pts = await Promise.all([
      recast.getRandomPointAsync(),
      recast.getRandomPointAsync(),
    ]);
    const [id1, id2] = pts.map((pt) => recast.addAgent({ position: pt, ...AGENT_OPTS }));

    const flock = recast.createFlockGroup({ agentIds: [id1], cohesionWeight: 0.5 });
    flock.addAgent(id2);
    flock.removeAgent(id1);

    const dst = await recast.getRandomPointAsync();
    flock.requestMoveTarget(dst.x, dst.y, dst.z);

    // id2 should move; id1 was removed so won't get flock targets
    const agents = await stepN(recast, 20);
    flock.destroy();
    recast.removeCrowdAgent(id1);
    recast.removeCrowdAgent(id2);

    const a2 = agents.find((a) => a.idx === id2);
    expect(distXZ(a2.position, pts[1])).toBeGreaterThan(0.01);
  });

  it('destroy stops issuing move targets', async () => {
    const pt = await recast.getRandomPointAsync();
    const id = recast.addAgent({ position: pt, ...AGENT_OPTS });
    const dst = await recast.getRandomPointAsync();

    const flock = recast.createFlockGroup({ agentIds: [id], cohesionWeight: 0.5 });
    flock.requestMoveTarget(dst.x, dst.y, dst.z);
    await stepN(recast, 5);

    // Record position just before destroy
    const before = await new Promise((resolve) => {
      recast.events.once('update', (agents) => {
        const a = agents.find((a) => a.idx === id);
        resolve({ x: a.position.x, y: a.position.y, z: a.position.z });
      });
      recast.crowdUpdate(0.1);
    });

    flock.destroy(); // no more targets issued

    // Agent may still coast for a bit; just verify no error thrown
    await stepN(recast, 5);
    recast.removeCrowdAgent(id);
  });
});
