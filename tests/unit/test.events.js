import { describe, it, expect, beforeAll } from 'vitest';
import { createRequire } from 'module';

const require = createRequire(import.meta.url);
/** @type {() => Promise<import('../../lib/recast.js').RecastModule>} */
const Recast = require('../../lib/recast.js');

describe('events', () => {
  /** @type {import('../../lib/recast.js').RecastModule} */
  let recast;

  beforeAll(async () => {
    recast = await Recast();
    // No navmesh needed — event emitter is independent of WASM state
  });

  it('events.on registers a persistent listener', () => {
    let count = 0;
    const listener = () => { count++; };

    recast.events.on('test-on', listener);
    recast.events.emit('test-on');
    recast.events.emit('test-on');

    expect(count).toBe(2);
    recast.events.off('test-on', listener);
  });

  it('events.off removes a specific listener', () => {
    let count = 0;
    const listener = () => { count++; };

    recast.events.on('test-off', listener);
    recast.events.off('test-off', listener);
    recast.events.emit('test-off');

    expect(count).toBe(0);
  });

  it('events.once fires exactly once', () => {
    let count = 0;

    recast.events.once('test-once', () => { count++; });
    recast.events.emit('test-once');
    recast.events.emit('test-once');

    expect(count).toBe(1);
  });

  it('events.removeAllListeners(type) clears only that type', () => {
    let countA = 0;
    let countB = 0;

    recast.events.on('type-a', () => { countA++; });
    recast.events.on('type-a', () => { countA++; });
    recast.events.on('type-b', () => { countB++; });

    recast.events.removeAllListeners('type-a');
    recast.events.emit('type-a');
    recast.events.emit('type-b');

    expect(countA).toBe(0);
    expect(countB).toBe(1);
    recast.events.removeAllListeners('type-b');
  });

  it('events.removeAllListeners() with no arg clears all types', () => {
    let countA = 0;
    let countB = 0;

    recast.events.on('all-a', () => { countA++; });
    recast.events.on('all-b', () => { countB++; });

    recast.events.removeAllListeners();
    recast.events.emit('all-a');
    recast.events.emit('all-b');

    expect(countA).toBe(0);
    expect(countB).toBe(0);
  });

  it('events.eventNames returns all registered event type names', () => {
    recast.events.removeAllListeners();

    const noop = () => {};
    recast.events.on('x', noop);
    recast.events.on('y', noop);

    const names = recast.events.eventNames();
    expect(names).toContain('x');
    expect(names).toContain('y');
    expect(names.length).toBe(2);

    recast.events.removeAllListeners();
  });

  it('events.deferEmit fires the listener on the next microtask tick', async () => {
    let fired = false;
    const listener = () => { fired = true; };

    recast.events.on('deferred', listener);
    recast.events.deferEmit('deferred');

    // Not yet called — still in synchronous execution
    expect(fired).toBe(false);

    // Yield to the microtask queue
    await Promise.resolve();

    expect(fired).toBe(true);
    recast.events.off('deferred', listener);
  });

  it('recast.on / recast.off / recast.emit are shorthands for events', () => {
    let count = 0;
    const listener = () => { count++; };

    recast.on('shorthand', listener);
    recast.emit('shorthand');
    expect(count).toBe(1);

    recast.off('shorthand', listener);
    recast.emit('shorthand');
    expect(count).toBe(1); // listener was removed
  });

  it('emit passes arguments to listener', () => {
    let received;
    recast.events.on('with-args', (a, b) => { received = { a, b }; });
    recast.events.emit('with-args', 42, 'hello');
    expect(received).toEqual({ a: 42, b: 'hello' });
    recast.events.removeAllListeners('with-args');
  });
});
