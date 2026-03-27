# Installation

```
npm install recastjs
```

# Usage

The compiled output is an async factory, call it to get the module instance:

```js
// CommonJS
const Recast = require('recastjs');
const recast = await Recast();

// ES module
import Recast from 'recastjs/recast.mjs';
const recast = await Recast();
```

From there, load your geometry and build the navmesh:

```js
recast.settings({
  cellSize: 0.3,
  cellHeight: 0.2,
  agentHeight: 2.0,
  agentRadius: 0.4,
  agentMaxClimb: 0.9,
  agentMaxSlope: 30,
});

await recast.OBJLoaderAsync('scene.obj');
await recast.buildTiledAsync();
```

Most methods have a `Promise`-based async variant.

```js
const start = await recast.getRandomPointAsync();
const end   = await recast.findNearestPointAsync({ x: 0, y: 0, z: 0 }, { x: 3, y: 3, z: 3 });
const path  = await recast.findPathAsync(start, end);
console.log(path); // [{ x, y, z }, ...]
```

Events are emitted on `recast.events` (a standard EventEmitter):

```js
// listen for crowd updates
recast.events.on('update', (agents) => { /* CrowdAgent[] */ });

// or use the shorthand on the module
recast.on('update', (agents) => { /* CrowdAgent[] */ });
recast.off('update', handler);
```
