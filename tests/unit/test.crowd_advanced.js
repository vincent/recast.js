import { describe, it, expect, beforeAll } from 'vitest';
import { createRequire } from 'module';
import path from 'path';
import settings from '../shared/settings.js';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');
const testsDir = new URL('.', import.meta.url).pathname;
const objPath = path.join(testsDir, '../fixtures/nav_test.obj');

/**
 * Run one crowd step and resolve with a snapshot of the active agents list.
 *
 * IMPORTANT: The 'update' event is emitted with agentPoolBuffer, which is the
 * live C++ pool array. The C++ clears it (agentPool_clear) synchronously after
 * the emit returns, before any microtask can run. We must therefore capture a
 * plain-object snapshot inside the event handler itself, not after the await.
 *
 * Note: The `active` field comes from C++ as an integer (1 or 0), not a boolean.
 */
function step(r) {
  return new Promise((resolve) => {
    r.events.once('update', (agents) => {
      resolve(agents.map((a) => ({
        idx: a.idx,
        position: { x: a.position.x, y: a.position.y, z: a.position.z },
        velocity: { x: a.velocity.x, y: a.velocity.y, z: a.velocity.z },
        active: a.active, // integer 1/0 from C++, not JS boolean
        state: a.state,
        desiredSpeed: a.desiredSpeed,
      })));
    });
    r.crowdUpdate(0.1);
    r.crowdGetActiveAgents();
  });
}

/** Build navmesh + init crowd on a fresh recast instance. */
async function freshCrowd() {
  const r = await Recast();
  settings(r);
  await new Promise((resolve) => {
    r.OBJLoader(objPath, function () {
      r.buildTiled();
      r.initCrowd(100, 1.0);
      resolve();
    });
  });
  return r;
}

describe('crowd advanced', () => {
  /** @type {import('../../lib/recast.js').RecastModule} */
  let recast;
  let agentId;

  beforeAll(async () => {
    recast = await freshCrowd();
    const pt = await recast.getRandomPointAsync();
    agentId = recast.addAgent({
      position: pt,
      radius: 0.5,
      height: 0.8,
      maxAcceleration: 8.0,
      maxSpeed: 3.5,
      updateFlags: recast.CROWD_OBSTACLE_AVOIDANCE | recast.CROWD_OPTIMIZE_VIS | recast.CROWD_OPTIMIZE_TOPO,
      separationWeight: 5.0,
    });
  });

  it('updateCrowdAgentParameters updates agent params without removing the agent', async () => {
    // IMPORTANT: updateCrowdAgentParameters defaults missing fields to 0.
    // Always pass all fields to avoid zeroing out radius/height/updateFlags.
    recast.updateCrowdAgentParameters(agentId, {
      radius: 0.5,
      height: 0.8,
      maxSpeed: 1.5,
      maxAcceleration: 2.0,
      updateFlags: recast.CROWD_OBSTACLE_AVOIDANCE | recast.CROWD_OPTIMIZE_VIS | recast.CROWD_OPTIMIZE_TOPO,
      separationWeight: 8.0,
    });

    const agents = await step(recast);
    const agent = agents.find((a) => a.idx === agentId);
    expect(agent).toBeDefined();
    // active is C++ integer 1/0, not JS boolean
    expect(agent.active).toBeTruthy();
  });

  it('crowdRequestMoveTarget causes the agent to move toward the target', async () => {
    // Use a fresh crowd instance so no prior updateCrowdAgentParameters side-effects
    const r = await freshCrowd();
    const startPt = await r.getRandomPointAsync();
    const id = r.addAgent({
      position: startPt,
      radius: 0.5,
      height: 0.8,
      maxAcceleration: 8.0,
      maxSpeed: 3.5,
      updateFlags: r.CROWD_OBSTACLE_AVOIDANCE | r.CROWD_OPTIMIZE_VIS | r.CROWD_OPTIMIZE_TOPO,
      separationWeight: 5.0,
    });

    // Pick a target far from the start
    const target = await r.getRandomPointAsync();
    r.crowdRequestMoveTarget(id, target.x, target.y, target.z);

    // Capture position after first step
    const agents0 = await step(r);
    const pos0 = agents0.find((a) => a.idx === id).position;

    // Advance 15 more steps (1.5 seconds of simulation at dt=0.1)
    for (let i = 0; i < 15; i++) {
      await step(r);
    }

    const agents1 = await step(r);
    const pos1 = agents1.find((a) => a.idx === id).position;

    const dx = pos1.x - pos0.x;
    const dy = pos1.y - pos0.y;
    const dz = pos1.z - pos0.z;
    const distMoved = Math.sqrt(dx * dx + dy * dy + dz * dz);
    expect(distMoved).toBeGreaterThan(0.01);
  });

  it('crowdRequestMoveVelocity steers the agent by direct velocity', async () => {
    recast.crowdRequestMoveVelocity(agentId, 2.0, 0.0, 0.0);

    const agents = await step(recast);
    const agent = agents.find((a) => a.idx === agentId);
    expect(agent).toBeDefined();
    expect(agent.active).toBeTruthy();
  });

  it('removeCrowdAgent removes the agent from the active agents list', async () => {
    const pt2 = await recast.getRandomPointAsync();
    const tempId = recast.addAgent({
      position: pt2,
      radius: 0.5,
      height: 0.8,
      maxAcceleration: 4.0,
      maxSpeed: 2.0,
      updateFlags: 0,
    });

    recast.removeCrowdAgent(tempId);

    const agents = await step(recast);
    // A removed agent slot may still appear in the pool but must not be active
    const activeFound = agents.find((a) => a.idx === tempId && a.active);
    expect(activeFound).toBeUndefined();
  });
});
