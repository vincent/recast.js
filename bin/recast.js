#!/usr/bin/env node
'use strict';

const fs     = require('fs');
const path   = require('path');
const Recast = require('../lib/recast');
const argv   = require('minimist')(process.argv.slice(2));

const SETTINGS = [
  'cellSize', 'cellHeight', 'agentHeight', 'agentRadius',
  'agentMaxClimb', 'agentMaxSlope', 'regionMinSize', 'regionMergeSize',
  'edgeMaxLen', 'edgeMaxError', 'vertsPerPoly', 'detailSampleDist', 'detailSampleMaxError',
];

const DEFAULTS = {
  cellSize: 0.3, cellHeight: 0.2, agentHeight: 2.0, agentRadius: 0.4,
  agentMaxClimb: 0.9, agentMaxSlope: 30, regionMinSize: 8, regionMergeSize: 20,
  edgeMaxLen: 12, edgeMaxError: 1.3,
};

function applySettings(recast, overrides) {
  const opts = Object.assign({}, DEFAULTS, overrides);
  for (const key of SETTINGS) {
    if (opts[key] !== undefined) recast[`set_${key}`](opts[key]);
  }
}

function loadObj(recast, objPath) {
  return new Promise((resolve, reject) => {
    recast.OBJLoader(path.resolve(objPath), function() { resolve(); });
  });
}

function detectType(filePath) {
  const ext = path.extname(filePath).toLowerCase();
  if (ext === '.tilecache') return 'tilecache';
  return 'navmesh';
}

function parseCoord(str) {
  const parts = str.split(',').map(Number);
  if (parts.length !== 3 || parts.some(isNaN)) {
    console.error(`Invalid coordinate "${str}" — expected x,y,z`);
    process.exit(1);
  }
  return parts;
}

async function cmdBuildNavmesh(argv) {
  const input  = argv.input  || argv.i || argv._[1];
  const output = argv.output || argv.o || argv._[2];

  if (!input || !output) {
    console.error('Usage: recast build-navmesh --input <level.obj> --output <level.bin> [options]');
    process.exit(1);
  }

  const recast = await Recast();
  applySettings(recast, argv);
  await loadObj(recast, input);
  recast.buildTiled();

  const [err, data] = await recast.saveTileMeshAsync('/tmp/recast_navmesh.bin');
  if (err) { console.error('Build failed:', err); process.exit(1); }

  await fs.promises.writeFile(output, Buffer.from(data));
  console.log(`Navmesh written to ${output} (${data.length} bytes)`);
}

async function cmdBuildTilecache(argv) {
  const input  = argv.input  || argv.i || argv._[1];
  const output = argv.output || argv.o || argv._[2];

  if (!input || !output) {
    console.error('Usage: recast build-tilecache --input <level.obj> --output <level.tilecache> [options]');
    process.exit(1);
  }

  const recast = await Recast();
  applySettings(recast, argv);
  await loadObj(recast, input);
  recast.buildTiled();

  const [err, data] = await recast.saveTileCacheAsync('/tmp/recast_tilecache.bin');
  if (err) { console.error('Build failed:', err); process.exit(1); }

  await fs.promises.writeFile(output, Buffer.from(data));
  console.log(`Tilecache written to ${output} (${data.length} bytes)`);
}

async function cmdFindPath(argv) {
  const navmesh = argv.navmesh || argv.n || argv._[1];
  const from    = argv.from    || argv.f || argv._[2];
  const to      = argv.to      || argv.t || argv._[3];

  if (!navmesh || !from || !to) {
    console.error('Usage: recast find-path --navmesh <file> --from x,y,z --to x,y,z');
    process.exit(1);
  }

  const [sx, sy, sz] = parseCoord(from);
  const [dx, dy, dz] = parseCoord(to);
  const extend = argv.extend || 5;

  const recast = await Recast();

  const type = detectType(navmesh);
  if (type === 'tilecache') {
    await recast.loadTileCacheAsync(path.resolve(navmesh));
  } else {
    await recast.loadTileMeshAsync(path.resolve(navmesh));
  }

  const [nx, ny, nz] = await recast.findNearestPointAsync(sx, sy, sz, extend, extend, extend);
  const [ex, ey, ez] = await recast.findNearestPointAsync(dx, dy, dz, extend, extend, extend);
  const pathResult   = await recast.findPathAsync(nx, ny, nz, ex, ey, ez, argv.maxPoints || 1000);

  if (!pathResult || pathResult.length === 0) {
    console.log(JSON.stringify({ found: false, points: [] }));
  } else {
    // pathResult is [{x, y, z}, ...] — filter out null padding entries
    const points = pathResult
      .filter(p => p !== null && p !== undefined)
      .map(p => [p.x, p.y, p.z]);
    console.log(JSON.stringify({ found: true, points }));
  }
}

async function cmdInspect(argv) {
  const file = argv.file || argv.f || argv._[1];

  if (!file) {
    console.error('Usage: recast inspect <navmesh.bin|level.tilecache>');
    process.exit(1);
  }

  const recast = await Recast();
  const type = detectType(file);

  if (type === 'tilecache') {
    await recast.loadTileCacheAsync(path.resolve(file));
  } else {
    await recast.loadTileMeshAsync(path.resolve(file));
  }

  const stat = fs.statSync(file);
  console.log(`File:  ${file}`);
  console.log(`Type:  ${type}`);
  console.log(`Size:  ${(stat.size / 1024).toFixed(1)} KB`);

  // Verify navmesh is functional by finding a random point
  try {
    const [x, y, z] = await recast.getRandomPointAsync();
    console.log(`Valid: yes (random point: ${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)})`);
  } catch (e) {
    console.log(`Valid: unable to verify (${e.message})`);
  }

  // Print tile info via getNavMeshTiles callback
  await new Promise((resolve) => {
    try {
      recast.getNavMeshTiles(recast.cb(function(tilesJson) {
        try {
          const tiles = JSON.parse(tilesJson);
          console.log(`Tiles: ${tiles.length}`);
        } catch (_) {
          // not all builds expose tile info after load
        }
        resolve();
      }));
    } catch (_) {
      resolve();
    }
  });
}

function printHelp() {
  console.log(`
recast - Navigation mesh CLI for recast.js

Commands:
  build-navmesh   Build a solo navmesh from an OBJ file
  build-tilecache Build a tiled navmesh (tilecache) from an OBJ file
  find-path       Find a path between two points in a navmesh
  inspect         Show info about a navmesh or tilecache file

Options (build commands):
  --cellSize N          Voxel cell size (default: 0.3)
  --cellHeight N        Voxel cell height (default: 0.2)
  --agentHeight N       Agent capsule height (default: 2.0)
  --agentRadius N       Agent capsule radius (default: 0.4)
  --agentMaxClimb N     Max step climb height (default: 0.9)
  --agentMaxSlope N     Max walkable slope in degrees (default: 30)
  --regionMinSize N     Min region size (default: 8)
  --regionMergeSize N   Max region merge size (default: 20)
  --edgeMaxLen N        Max edge length in voxels (default: 12)
  --edgeMaxError N      Edge simplification error (default: 1.3)

Examples:
  recast build-navmesh level.obj level.bin
  recast build-tilecache level.obj level.tilecache --agentHeight 1.8
  recast inspect level.bin
  recast find-path level.bin --from 0,0,0 --to 10,0,10
`.trim());
}

(async function main() {
  const cmd = argv._[0];

  try {
    switch (cmd) {
      case 'build-navmesh':   await cmdBuildNavmesh(argv);   break;
      case 'build-tilecache': await cmdBuildTilecache(argv); break;
      case 'find-path':       await cmdFindPath(argv);       break;
      case 'inspect':         await cmdInspect(argv);        break;
      default:
        printHelp();
        if (cmd) process.exit(1);
    }
  } catch (err) {
    console.error('Error:', err.message || err);
    process.exit(1);
  }
})();
