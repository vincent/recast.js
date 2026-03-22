# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

Recast.js is a JavaScript/WebAssembly navigation mesh library for games, compiled from the [recastnavigation](https://github.com/recastnavigation/recastnavigation) C++ library using Emscripten. It computes walkable areas from scene geometry and exposes pathfinding, crowd simulation, and obstacle management.

## Build requirements

- **CMake**
- **Emscripten** (`emcc` must be on PATH)
- **Node.js** + npm dependencies (`npm install`)

## Commands

```bash
# Initialize/update the C++ submodule and apply patches
make update-source

# Compile C++ → lib/recast.js
npm run build        # or: make build

# Run tests
npm test             # or: make test

# Run a single test file
./node_modules/nodeunit/bin/nodeunit tests/recast.tests.js

# Lint the compiled output
make lint

# Clean build artifacts
make clean
```

## Architecture

The project has three distinct layers:

### 1. C++ core (`recastnavigation/` submodule)
A patched version of the upstream recastnavigation library. Do not edit these files directly — patches are managed via `src/recastnavigation.patch` and applied by `make update-source`.

### 2. C++ JavaScript bindings (`src/JavascriptInterface/`)
- **`main.cpp`** — the primary Emscripten binding file. Exposes all navmesh operations to JS via Emscripten's `--bind` mechanism.
- **`JavascriptInterface.cpp/.h`** — helper abstractions used by main.cpp.
- Compiled together with the recastnavigation sources into a single `lib/recast.js` output file.

### 3. JavaScript wrapper (`src/pre.module.js`, `src/post.module.js`, `src/library_recast.js`)
- **`pre.module.js`** — prepended to the Emscripten output; sets up environment detection (Node/Web/Worker), EventEmitter, and buffer utilities.
- **`post.module.js`** — appended to the Emscripten output; handles CommonJS/AMD/global module export patterns.
- **`library_recast.js`** — Emscripten JS library (`--js-library`); implements callbacks (`flush_active_agents_callback`, `invoke_vector_callback`, `invoke_file_callback`) and GL object / agent pool management.

### Build output
Everything compiles into a single **`lib/recast.js`** file (~8MB). This file is what consumers `require()`/`import`. It is compiled with `WASM=0` (asm.js), closure compiler (`--closure 1`), and `ALLOW_MEMORY_GROWTH=1`.

## Tests

Tests live in `tests/` and use **nodeunit**. The `make test` target runs all files matching `tests/test.*.js`. Test geometry is provided as `.obj` files (`nav_test.obj`, `dungeon.obj`) and a pre-computed navmesh (`navmesh.bin`). Browser-based tests use the `tests/test.*.html` files and must be opened manually in a browser.

## Key API surface (from `main.cpp`)

Configuration setters: `set_cellSize`, `set_cellHeight`, `set_agentHeight`, `set_agentRadius`, `set_agentMaxClimb`, `set_agentMaxSlope`

Build: `buildSolo()`, `buildTiled()`

Navmesh queries: `findPath()`, `findNearestPoint()`, `findNearestPoly()`, `getRandomPoint()`

Crowd/agents: `initCrowd()`, `addAgent()`, `updateCrowdAgentParameters()`, `removeCrowdAgent()`, `crowdUpdate()`, `crowdRequestMoveTarget()`, `crowdGetActiveAgents()`

Obstacles: `addTempObstacle()`, `removeTempObstacle()`, `getAllTempObstacles()`

Persistence: `saveTileMesh()`, `loadTileMesh()`, `saveTileCache()`, `loadTileCache()`
