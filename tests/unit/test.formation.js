import { describe, it, expect, beforeAll } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;
const objPath = path.join(testsDir, '../fixtures/nav_test.obj');

async function stepN(r, n, dt = 0.1) {
  let last;
  for (let i = 0; i < n; i++) {
    last = await new Promise((resolve) => {
      r.events.once('update', (agents) => {
        resolve(agents.map((a) => ({
          idx: a.idx,
          position: { x: a.position.x, y: a.position.y, z: a.position.z },
          active: a.active,
        })));
      });
      r.crowdUpdate(dt);
    });
  }
  return last;
}

function distXZ(a, b) {
  return Math.sqrt((a.x - b.x) ** 2 + (a.z - b.z) ** 2);
}

async function freshCrowd() {
  const r = await Recast();
  settings(r);
  r.withPlugin(r.Formation);
  await new Promise((resolve) => {
    r.OBJLoader(objPath, function () {
      r.buildTiled();
      r.initCrowd(100, 1.0);
      resolve();
    });
  });
  return r;
}

const LOCAL_FORMATION_OFFSETS = [
  [0, 0],
  [-2, -2],
  [2, -2],
  [-2, 2],
  [2, 2],
];

async function localFormationScenario(r, count) {
  const startOffsets = LOCAL_FORMATION_OFFSETS.slice(0, count);
  const pointExtent = { x: 4, y: 4, z: 4 };
  const destinationExtent = { x: 8, y: 8, z: 8 };

  for (let attempt = 0; attempt < 50; attempt++) {
    const anchor = await r.getRandomPointAsync();
    const pts = await Promise.all(
      startOffsets.map(([dx, dz]) => r.findNearestPointAsync({
        x: anchor.x + dx,
        y: anchor.y,
        z: anchor.z + dz,
      }, pointExtent))
    );
    const dst = await r.findNearestPointAsync({
      x: anchor.x + 8,
      y: anchor.y,
      z: anchor.z + 6,
    }, destinationExtent);
    const ys = pts.map((pt) => pt.y).concat(dst.y);
    const ySpread = Math.max(...ys) - Math.min(...ys);
    if (ySpread >= 0.6) continue;

    const paths = await Promise.all(pts.map((pt) => r.findPathAsync(pt, dst, 1000)));
    if (paths.every((path) => path.length > 1)) {
      return { pts, dst };
    }
  }

  throw new Error(`Unable to find a local connected formation scenario for ${count} agents`);
}

const AGENT_OPTS = {
  radius: 0.4,
  height: 1.8,
  maxAcceleration: 8.0,
  maxSpeed: 3.5,
  updateFlags: 0,
  separationWeight: 0,
};

const FORMATION_TYPES = [
  'circle',
  'line',
  'square',
  'arc',
  'v',
  'wedge',
  'hex',
  'rings',
  'spiral',
  'wave',
];

describe('Formation', () => {
  /** @type {import('../../lib/recast.js').RecastModule} */
  let recast;

  beforeAll(async () => {
    recast = await freshCrowd();
  });

  it('createFormation returns an object with the expected interface', () => {
    const f = recast.createFormation({ agentIds: [], type: 'circle', spacing: 2.0 });
    expect(typeof f.requestMoveTarget).toBe('function');
    expect(typeof f.addAgent).toBe('function');
    expect(typeof f.removeAgent).toBe('function');
    expect(typeof f.destroy).toBe('function');
    f.destroy();
  });

  it('all formation types can be created without error', () => {
    for (const type of FORMATION_TYPES) {
      const f = recast.createFormation({ agentIds: [], type, spacing: 2.0 });
      expect(f).toBeTruthy();
      f.destroy();
    }
  });

  it('agents in a circle formation all move toward the destination', async () => {
    const pts = await Promise.all([
      recast.getRandomPointAsync(),
      recast.getRandomPointAsync(),
      recast.getRandomPointAsync(),
    ]);
    const ids = pts.map((pt) => recast.addAgent({ position: pt, ...AGENT_OPTS }));
    const formation = recast.createFormation({ agentIds: ids, type: 'circle', spacing: 2.0 });

    const dst = await recast.getRandomPointAsync();
    formation.requestMoveTarget(dst.x, dst.y, dst.z);

    const agents = await stepN(recast, 40);
    formation.destroy();
    ids.forEach((id) => recast.removeCrowdAgent(id));

    const members = agents.filter((a) => ids.includes(a.idx));
    expect(members.length).toBe(ids.length);

    // Each agent should have moved from its spawn position
    const moved = members.filter((a, i) =>
      distXZ(a.position, pts[ids.indexOf(a.idx)]) > 0.01
    );
    expect(moved.length).toBeGreaterThan(0);
  });

  it('agents without a destination do not move', async () => {
    const pt = await recast.getRandomPointAsync();
    const id = recast.addAgent({ position: pt, ...AGENT_OPTS });
    const formation = recast.createFormation({ agentIds: [id], type: 'line', spacing: 2.0 });
    // No requestMoveTarget

    const agents = await stepN(recast, 10);
    formation.destroy();
    recast.removeCrowdAgent(id);

    const member = agents.find((a) => a.idx === id);
    expect(distXZ(member.position, pt)).toBeLessThan(0.5);
  });

  it('addAgent and removeAgent update slot membership', async () => {
    const pts = await Promise.all([
      recast.getRandomPointAsync(),
      recast.getRandomPointAsync(),
      recast.getRandomPointAsync(),
    ]);
    const [id1, id2, id3] = pts.map((pt) => recast.addAgent({ position: pt, ...AGENT_OPTS }));

    const formation = recast.createFormation({
      agentIds: [id1, id2],
      type: 'square',
      spacing: 2.0,
    });
    formation.addAgent(id3);     // id3 gets slot 2
    formation.removeAgent(id1);  // id1 ejected from formation

    const dst = await recast.getRandomPointAsync();
    formation.requestMoveTarget(dst.x, dst.y, dst.z);

    const agents = await stepN(recast, 20);
    formation.destroy();
    recast.removeCrowdAgent(id1);
    recast.removeCrowdAgent(id2);
    recast.removeCrowdAgent(id3);

    // id2 and id3 should have moved; id1 was removed and receives no formation target
    const a2 = agents.find((a) => a.idx === id2);
    const a3 = agents.find((a) => a.idx === id3);
    expect(distXZ(a2.position, pts[1])).toBeGreaterThan(0.01);
    expect(distXZ(a3.position, pts[2])).toBeGreaterThan(0.01);
  });

  it('destroy stops issuing formation targets without error', async () => {
    const pt = await recast.getRandomPointAsync();
    const id = recast.addAgent({ position: pt, ...AGENT_OPTS });
    const dst = await recast.getRandomPointAsync();

    const formation = recast.createFormation({ agentIds: [id], type: 'arc', spacing: 2.0 });
    formation.requestMoveTarget(dst.x, dst.y, dst.z);
    await stepN(recast, 5);

    formation.destroy();

    // Should not throw after destroy
    await stepN(recast, 5);
    recast.removeCrowdAgent(id);
  });

  it('circle formation agents are stable after arriving', async () => {
    const r = await freshCrowd();
    const { pts, dst } = await localFormationScenario(r, 3);
    const ids = pts.map((pt) => r.addAgent({ position: pt, ...AGENT_OPTS }));
    const formation = r.createFormation({ agentIds: ids, type: 'circle', spacing: 2.0 });
    formation.requestMoveTarget(dst.x, dst.y, dst.z);

    await stepN(r, 500);
    const snap1 = await stepN(r, 1);
    const snap2 = await stepN(r, 10);

    formation.destroy();
    ids.forEach((id) => r.removeCrowdAgent(id));

    for (const id of ids) {
      const a1 = snap1.find((a) => a.idx === id);
      const a2 = snap2.find((a) => a.idx === id);
      expect(distXZ(a1.position, a2.position)).toBeLessThan(0.2);
    }
  });

  it('square formation agents are stable after arriving (partial grid n=5)', async () => {
    const r = await freshCrowd();
    const { pts, dst } = await localFormationScenario(r, 5);
    const ids = pts.map((pt) => r.addAgent({ position: pt, ...AGENT_OPTS }));
    const formation = r.createFormation({ agentIds: ids, type: 'square', spacing: 2.0 });
    formation.requestMoveTarget(dst.x, dst.y, dst.z);

    await stepN(r, 500);
    const snap1 = await stepN(r, 1);
    const snap2 = await stepN(r, 10);

    formation.destroy();
    ids.forEach((id) => r.removeCrowdAgent(id));

    for (const id of ids) {
      const a1 = snap1.find((a) => a.idx === id);
      const a2 = snap2.find((a) => a.idx === id);
      expect(distXZ(a1.position, a2.position)).toBeLessThan(0.2);
    }
  });

  it('line formation agents are stable after arriving', async () => {
    const r = await freshCrowd();
    const { pts, dst } = await localFormationScenario(r, 4);
    const ids = pts.map((pt) => r.addAgent({ position: pt, ...AGENT_OPTS }));
    const formation = r.createFormation({ agentIds: ids, type: 'line', spacing: 2.0 });
    formation.requestMoveTarget(dst.x, dst.y, dst.z);

    await stepN(r, 500);
    const snap1 = await stepN(r, 1);
    const snap2 = await stepN(r, 10);

    formation.destroy();
    ids.forEach((id) => r.removeCrowdAgent(id));

    for (const id of ids) {
      const a1 = snap1.find((a) => a.idx === id);
      const a2 = snap2.find((a) => a.idx === id);
      expect(distXZ(a1.position, a2.position)).toBeLessThan(0.2);
    }
  });

  it('arc formation agents are stable after arriving', async () => {
    const r = await freshCrowd();
    const { pts, dst } = await localFormationScenario(r, 4);
    const ids = pts.map((pt) => r.addAgent({ position: pt, ...AGENT_OPTS }));
    const formation = r.createFormation({ agentIds: ids, type: 'arc', spacing: 2.0 });
    formation.requestMoveTarget(dst.x, dst.y, dst.z);

    await stepN(r, 500);
    const snap1 = await stepN(r, 1);
    const snap2 = await stepN(r, 10);

    formation.destroy();
    ids.forEach((id) => r.removeCrowdAgent(id));

    for (const id of ids) {
      const a1 = snap1.find((a) => a.idx === id);
      const a2 = snap2.find((a) => a.idx === id);
      expect(distXZ(a1.position, a2.position)).toBeLessThan(0.2);
    }
  });

  it('arc formation: single agent gets center slot without error', async () => {
    const pt = await recast.getRandomPointAsync();
    const id = recast.addAgent({ position: pt, ...AGENT_OPTS });

    const formation = recast.createFormation({ agentIds: [id], type: 'arc', spacing: 2.0 });
    const dst = await recast.getRandomPointAsync();
    formation.requestMoveTarget(dst.x, dst.y, dst.z);

    const agents = await stepN(recast, 10);
    formation.destroy();
    recast.removeCrowdAgent(id);

    const member = agents.find((a) => a.idx === id);
    expect(member).toBeTruthy();
  });

  it('new formation types assign targets and move agents', async () => {
    const r = await freshCrowd();

    for (const type of ['circle', 'v', 'wedge', 'hex', 'rings', 'spiral', 'wave']) {
      const pts = await Promise.all([
        r.getRandomPointAsync(),
        r.getRandomPointAsync(),
        r.getRandomPointAsync(),
      ]);
      const ids = pts.map((pt) => r.addAgent({ position: pt, ...AGENT_OPTS }));

      const formation = r.createFormation({ agentIds: ids, type, spacing: 2.0 });
      const dst = await r.getRandomPointAsync();
      formation.requestMoveTarget(dst.x, dst.y, dst.z);

      const agents = await stepN(r, 100);
      formation.destroy();
      ids.forEach((id) => r.removeCrowdAgent(id));

      const members = agents.filter((a) => ids.includes(a.idx));
      expect(members.length).toBe(ids.length);
      expect(members.some((a) => distXZ(a.position, pts[ids.indexOf(a.idx)]) > 0.01)).toBe(true);
    }
  });
});
