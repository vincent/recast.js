# What is this all about

In a 3D environment context, Recast.js is meant to be used with a file describing a scene geometry. Using the [recastnavigation](https://github.com/recastnavigation/recastnavigation) C++ library compiled to WebAssembly, it will deduce a [navigation mesh](https://en.wikipedia.org/wiki/Navigation_mesh): a set of polygons on which your characters can move.

Once this navigation mesh is computed, it is possible to use built-in pathfinding operations like "find the shortest path between point A and point B", taking into account obstacles, slopes, and off-mesh connections.

If you decide to use it to animate your scene characters aka "agents", it also provides a complete crowd system capable of managing all your agents movements using per-agent settings (speed, radius, ...).

## What can you do with it

* load any mesh in `.obj` format
* compute its navigation mesh with configurable parameters (cell size, agent height, slope angle, ...)
* build a solo navmesh or a tiled navmesh (better for large scenes and dynamic obstacles)
* save and load the computed navmesh to skip recomputation
* find the shortest path between two points
* find the nearest navigable point or polygon to any world position
* get a random point guaranteed to be on the navmesh
* place and remove temporary obstacles with a radius
* define off-mesh connections (jump links, doors, portals)
* assign polygon flags and group them into zones to influence pathfinding
* add agents to the navmesh and drive them with a full crowd simulation
* update agents individually (speed, radius, separation weight, ...)

## Installation

```
npm install recastjs
```

The compiled output is an async factory — call it to get the module instance:

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

recast.OBJLoader('scene.obj', () => {
  recast.buildTiled();
});
```

All operations that return data have both a callback form and a `Promise`-based async variant:

```js
// callback
recast.findPath(sx, sy, sz, dx, dy, dz, maxPoints, recast.cb((path) => {
  console.log(path);
}));

// async/await
const path = await recast.findPathAsync(sx, sy, sz, dx, dy, dz, maxPoints);
```

## CLI

A `recast` binary is included for working with navmeshes from the command line:

```bash
# Build a navmesh from an OBJ file
recast build-navmesh scene.obj navmesh.bin

# Build a tile cache (supports dynamic obstacles)
recast build-tilecache scene.obj scene.tilecache

# Find a path from the closest walkable point to x=0,y=0,z=0 to the closest walkable point 10,0,10 and get JSON output
recast find-path navmesh.bin --from 0,0,0 --to 10,0,10

# Inspect a saved navmesh or tile cache
recast inspect navmesh.bin
```

Common options: `--cellSize`, `--cellHeight`, `--agentHeight`, `--agentRadius`, `--agentMaxClimb`, `--agentMaxSlope`, `--regionMinSize`, `--edgeMaxLen`, `--edgeMaxError`

## Is it for WebGL, Three.js or Babylon.js?

It is designed to work alongside any WebGL software but it is completely library agnostic. It only manages a mesh and its properties — it has no opinion on how you render things.

It runs in Node.js, in the browser, and in Web Workers. Included tests and demos use [Three.js](https://github.com/mrdoob/three.js), though.

## Demos

Browser demos are available in [tests/browser/](https://vincent.github.io/recast.js/tests/browser):

* **city** — crowd simulation with multiple agents and Three.js visualization
* **doors** — off-mesh connections for door traversal
* **offmeshs** — custom jump links
* **webgl** — direct WebGL debug rendering of the navmesh
* **worker** — running recast inside a Web Worker

