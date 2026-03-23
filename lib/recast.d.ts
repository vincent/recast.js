/**
 * recast.js — TypeScript declarations
 *
 * The library exports an async factory. Consumers must await it:
 * @example
 * // ESM
 * import Recast from 'recastjs';
 * const recast = await Recast();
 *
 * // CommonJS
 * const Recast = require('recastjs');
 * const recast = await Recast();
 */

// ─── Supporting types ────────────────────────────────────────────────────────

/** A 3D point or vector as a plain object. */
export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

/** A 3-element tuple `[x, y, z]` returned by point-query async helpers. */
export type Point3 = [number, number, number];

/** Navigation mesh configuration passed to {@link RecastModule.settings}. */
export interface RecastSettings {
  /** Voxel width and depth. Smaller = higher resolution. Default: 0.3 */
  cellSize: number;
  /** Voxel height. Smaller = more vertical precision. Default: 0.2 */
  cellHeight: number;
  /** Minimum floor-to-ceiling height for a walkable area. Default: 2.0 */
  agentHeight: number;
  /** Agent radius (determines clearance from walls). Default: 0.4 */
  agentRadius: number;
  /** Maximum step height an agent can climb. Default: 0.9 */
  agentMaxClimb: number;
  /** Maximum walkable slope in degrees. Default: 45 */
  agentMaxSlope: number;
}

/** Options for adding a crowd agent via {@link RecastModule.addAgent}. */
export interface AgentOptions {
  position: Vec3;
  radius: number;
  height: number;
  maxAcceleration: number;
  maxSpeed: number;
  /** Bitmask of `CROWD_*` flags. Defaults to all flags enabled. */
  updateFlags?: number;
  separationWeight?: number;
}

/** An active crowd agent returned by {@link RecastModule.crowdGetActiveAgents}. */
export interface CrowdAgent {
  /** Agent index in the crowd. */
  idx: number;
  position: Vec3;
  velocity: Vec3;
  radius: number;
  active: number;
  state: number;
  neighbors: number;
  partial: number;
  desiredSpeed: number;
}

/** A waypoint in a path returned by {@link RecastModule.findPathAsync}. */
export interface PathPoint {
  x: number;
  y: number;
  z: number;
}

/** Zone data passed to {@link RecastModule.setZones}. */
export interface ZoneData {
  /** Polygon references belonging to this zone. */
  refs: number[];
  /** Initial poly flags to apply (e.g. `FLAG_WALK`). */
  flags: number[];
}

// ─── Zone class ──────────────────────────────────────────────────────────────

/**
 * A named group of polygons sharing the same traversal flags.
 * Instances are created internally by {@link RecastModule.setZones}.
 */
export declare class Zone {
  constructor(name: string, data: ZoneData);

  name: string;
  refs: number[];
  flags: number;

  /** Returns `true` if `FLAG_WALK` is set. */
  isWalkable(): boolean;
  /** Returns `true` if all bits of `flags` are set. */
  is(flags: number): boolean;
  /** Sets one or more flag bits and syncs to the navmesh. */
  setFlags(flags: number): this;
  /** Clears one or more flag bits and syncs to the navmesh. */
  clearFlags(flags: number): this;
  /** Toggles one or more flag bits and syncs to the navmesh. */
  toggleFlags(flags: number): this;
  /** Pushes current flags to all referenced polygons. */
  syncFlags(): this;
}

// ─── EventEmitter ────────────────────────────────────────────────────────────

export interface RecastEventEmitter {
  on(type: string, listener: (...args: any[]) => void): this;
  once(type: string, listener: (...args: any[]) => void): this;
  remove(type?: string): this;
  emit(type: string, ...args: any[]): this;
  /** Emits on the next microtask tick. */
  deferEmit(type: string, ...args: any[]): this;
  listeners(): string[];
}

// ─── Main module interface ────────────────────────────────────────────────────

export interface RecastModule {

  // ── Configuration ──────────────────────────────────────────────────────────

  /**
   * Set all navmesh build parameters at once.
   * @example
   * recast.settings({ cellSize: 0.3, cellHeight: 0.2, agentHeight: 2, agentRadius: 0.4, agentMaxClimb: 0.9, agentMaxSlope: 45 });
   */
  settings(options: RecastSettings): void;

  /** Voxel width/depth of a cell. Smaller = higher resolution. */
  set_cellSize(value: number): void;
  /** Voxel height. */
  set_cellHeight(value: number): void;
  /** Minimum floor-to-ceiling clearance for a walkable area. */
  set_agentHeight(value: number): void;
  /** Agent clearance radius from walls. */
  set_agentRadius(value: number): void;
  /** Maximum step height an agent can climb. */
  set_agentMaxClimb(value: number): void;
  /** Maximum walkable slope in degrees. */
  set_agentMaxSlope(value: number): void;

  // ── Geometry loading ───────────────────────────────────────────────────────

  /**
   * Load geometry from an OBJ file at `path`.
   * In Node.js reads from disk; in the browser fetches via HTTP.
   */
  OBJLoader(path: string, callback: (module: RecastModule) => void): void;

  /**
   * Load geometry from an OBJ string or buffer already in memory.
   */
  OBJDataLoader(data: string | ArrayBuffer, callback: (module: RecastModule) => void): void;

  // ── Navmesh building ───────────────────────────────────────────────────────

  /**
   * Build a single-tile (solo) navmesh from the loaded geometry.
   * Emits `built` on {@link RecastModule.vent} when done.
   */
  buildSolo(): void;

  /**
   * Build a tiled navmesh from the loaded geometry.
   * Emits `built` on {@link RecastModule.vent} when done.
   */
  buildTiled(): void;

  // ── Pathfinding ────────────────────────────────────────────────────────────

  /**
   * Find a path between two points.
   * @param sx - Start X
   * @param sy - Start Y
   * @param sz - Start Z
   * @param dx - Destination X
   * @param dy - Destination Y
   * @param dz - Destination Z
   * @param max - Maximum number of waypoints
   * @param callback_id - Callback handle from {@link RecastModule.cb}
   */
  findPath(sx: number, sy: number, sz: number, dx: number, dy: number, dz: number, max: number, callback_id: number): void;

  /**
   * Async version of {@link RecastModule.findPath}.
   * Resolves with an array of waypoints along the path.
   */
  findPathAsync(sx: number, sy: number, sz: number, dx: number, dy: number, dz: number, max: number): Promise<PathPoint[]>;

  /**
   * Find the navmesh point nearest to the given position within an extent box.
   * @param callback_id - Callback handle from {@link RecastModule.cb}; called with `(x, y, z)`.
   */
  findNearestPoint(x: number, y: number, z: number, extX: number, extY: number, extZ: number, callback_id: number): void;

  /**
   * Async version of {@link RecastModule.findNearestPoint}.
   * Resolves with `[x, y, z]`.
   */
  findNearestPointAsync(x: number, y: number, z: number, extX: number, extY: number, extZ: number): Promise<Point3>;

  /**
   * Find the polygon reference nearest to the given position within an extent box.
   * @param callback_id - Callback handle from {@link RecastModule.cb}; called with the poly ref.
   */
  findNearestPoly(x: number, y: number, z: number, extX: number, extY: number, extZ: number, callback_id: number): void;

  /**
   * Async version of {@link RecastModule.findNearestPoly}.
   * Resolves with the polygon reference number.
   */
  findNearestPolyAsync(x: number, y: number, z: number, extX: number, extY: number, extZ: number): Promise<number>;

  /**
   * Get a random walkable point on the navmesh.
   * @param callback_id - Callback handle from {@link RecastModule.cb}; called with `(x, y, z)`.
   */
  getRandomPoint(callback_id: number): void;

  /**
   * Async version of {@link RecastModule.getRandomPoint}.
   * Resolves with `[x, y, z]`.
   */
  getRandomPointAsync(): Promise<Point3>;

  /**
   * Find all polygons within an AABB centred at the given position.
   * @param maxPolys - Maximum number of polygons to return (default: 1000).
   * @param callback_id - Callback handle from {@link RecastModule.cb}.
   */
  queryPolygons(posX: number, posY: number, posZ: number, extX: number, extY: number, extZ: number, maxPolys: number, callback_id: number): void;

  // ── Polygon flags ──────────────────────────────────────────────────────────

  /**
   * Set traversal flags on all polygons within an AABB.
   * Use the `FLAG_*` constants for the `flags` bitmask.
   */
  setPolyFlags(sx: number, sy: number, sz: number, dx: number, dy: number, dz: number, flags: number): void;

  /**
   * Set traversal flags on a specific polygon by its reference.
   */
  setPolyFlagsByRef(ref: number, flags: number): void;

  // ── Persistence ────────────────────────────────────────────────────────────

  /**
   * Save the current tiled navmesh to `path` on disk (Node.js / Emscripten FS).
   * @param callback_id - Callback handle from {@link RecastModule.cb}.
   */
  saveTileMesh(path: string, callback_id: number): void;

  /** Async version of {@link RecastModule.saveTileMesh}. */
  saveTileMeshAsync(path: string): Promise<void>;

  /**
   * Load a previously saved navmesh from `path`.
   * @param callback_id - Callback handle from {@link RecastModule.cb}.
   */
  loadTileMesh(path: string, callback_id: number): void;

  /** Async version of {@link RecastModule.loadTileMesh}. */
  loadTileMeshAsync(path: string): Promise<void>;

  /**
   * Save the tile cache (dynamic obstacles + navmesh) to `path`.
   * @param callback_id - Callback handle from {@link RecastModule.cb}.
   */
  saveTileCache(path: string, callback_id: number): void;

  /** Async version of {@link RecastModule.saveTileCache}. */
  saveTileCacheAsync(path: string): Promise<void>;

  /**
   * Load a previously saved tile cache from `path`.
   * @param callback_id - Callback handle from {@link RecastModule.cb}.
   */
  loadTileCache(path: string, callback_id: number): void;

  /** Async version of {@link RecastModule.loadTileCache}. */
  loadTileCacheAsync(path: string): Promise<void>;

  // ── Crowd simulation ───────────────────────────────────────────────────────

  /**
   * Initialise the crowd system.
   * Must be called once before adding agents.
   * @param maxAgents - Maximum number of simultaneous agents.
   * @param maxAgentRadius - Maximum agent radius (used to size internal buffers).
   */
  initCrowd(maxAgents: number, maxAgentRadius: number): void;

  /**
   * Add a crowd agent using an options object.
   * Returns the agent index used to identify it in subsequent calls.
   * @example
   * const id = recast.addAgent({ position: { x: 0, y: 0, z: 0 }, radius: 0.5, height: 1.8, maxAcceleration: 8, maxSpeed: 3.5 });
   */
  addAgent(options: AgentOptions): number;

  /**
   * Add a crowd agent using individual parameters.
   * Prefer {@link RecastModule.addAgent} for clarity.
   */
  addCrowdAgent(x: number, y: number, z: number, radius: number, height: number, maxAcceleration: number, maxSpeed: number, updateFlags: number, separationWeight: number): number;

  /**
   * Update parameters of an existing crowd agent.
   */
  updateCrowdAgentParameters(agentId: number, x: number, y: number, z: number, radius: number, height: number, maxAcceleration: number, maxSpeed: number, updateFlags: number, separationWeight: number): void;

  /**
   * Remove a crowd agent by its index.
   */
  removeCrowdAgent(agentId: number): void;

  /**
   * Request an agent to move toward a target position.
   */
  crowdRequestMoveTarget(agentId: number, x: number, y: number, z: number): void;

  /**
   * Request an agent to move with a specific velocity vector.
   */
  requestMoveVelocity(agentId: number, x: number, y: number, z: number): void;

  /**
   * Advance the crowd simulation by `dt` seconds.
   * Triggers an `update` event on {@link RecastModule.vent} with the active agent list.
   */
  crowdUpdate(dt: number): void;

  /**
   * Retrieve all currently active agents.
   * @param callback_id - Callback handle from {@link RecastModule.cb}; called with `CrowdAgent[]`.
   */
  crowdGetActiveAgents(callback_id?: number): void;

  // ── Temporary obstacles ────────────────────────────────────────────────────

  /**
   * Add a cylindrical obstacle to the tile cache.
   * Returns an obstacle reference ID.
   */
  addTempObstacle(x: number, y: number, z: number, radius: number): number;

  /**
   * Remove a temporary obstacle by its reference.
   */
  removeTempObstacle(obstacleRef: number): void;

  /**
   * Remove all temporary obstacles.
   */
  removeAllTempObstacles(): void;

  /**
   * Retrieve all active temporary obstacles.
   * @param callback_id - Callback handle from {@link RecastModule.cb}.
   */
  getAllTempObstacles(callback_id: number): void;

  // ── Off-mesh connections ───────────────────────────────────────────────────

  /**
   * Add a custom off-mesh connection (e.g. a jump or ladder link).
   * @param bidir - 1 for bidirectional, 0 for one-way.
   */
  addOffMeshConnection(startX: number, startY: number, startZ: number, endX: number, endY: number, endZ: number, radius: number, bidir: 0 | 1): void;

  // ── Zones ──────────────────────────────────────────────────────────────────

  /**
   * Named zones keyed by zone name.
   * Populated by {@link RecastModule.setZones}.
   */
  zones: Record<string, Zone>;

  /**
   * Create named zones with initial polygon references and flags.
   * @example
   * recast.setZones({ water: { refs: [polyRef1, polyRef2], flags: [recast.FLAG_SWIM] } });
   */
  setZones(zones: Record<string, ZoneData>): void;

  /** The Zone constructor — use to create zones manually. */
  Zone: typeof Zone;

  // ── Callback registration ──────────────────────────────────────────────────

  /**
   * Register a one-shot callback and return its integer ID.
   * Pass the returned ID as the `callback_id` argument of low-level methods.
   * @example
   * recast.getRandomPoint(recast.cb((x, y, z) => console.log(x, y, z)));
   */
  cb(fn: (...args: any[]) => void): number;

  // ── Events ────────────────────────────────────────────────────────────────

  /**
   * The underlying event emitter.
   * Emits:
   * - `built` (type: string) — after `buildSolo()` or `buildTiled()` completes.
   * - `update` (agents: CrowdAgent[]) — after each `crowdUpdate()` call.
   */
  vent: RecastEventEmitter;

  /** Shorthand for `recast.vent.on(type, listener)`. */
  on(type: string, listener: (...args: any[]) => void): void;
  /** Shorthand for `recast.vent.emit(type, ...args)`. */
  emit(type: string, ...args: any[]): void;

  // ── GL rendering (browser only) ────────────────────────────────────────────

  /**
   * Provide a WebGL rendering context for debug drawing.
   */
  setGLContext(glContext: WebGLRenderingContext): void;

  /** The current WebGL context set via {@link RecastModule.setGLContext}. */
  glContext?: WebGLRenderingContext;

  /**
   * Draw a named GL object using the current {@link RecastModule.glContext}.
   */
  drawObject(objectName: string): void;

  // ── State ─────────────────────────────────────────────────────────────────

  /**
   * The navmesh type last built: `solo` or `tiled`.
   * Set after the `built` event fires.
   */
  navmeshType?: 'solo' | 'tiled';

  // ── Crowd update flags ────────────────────────────────────────────────────

  /** Crowd update flag: enable anticipation of turns. */
  readonly CROWD_ANTICIPATE_TURNS: 1;
  /** Crowd update flag: enable obstacle avoidance. */
  readonly CROWD_OBSTACLE_AVOIDANCE: 2;
  /** Crowd update flag: enable separation between agents. */
  readonly CROWD_SEPARATION: 4;
  /** Crowd update flag: enable path visibility optimisation. */
  readonly CROWD_OPTIMIZE_VIS: 8;
  /** Crowd update flag: enable path topology optimisation. */
  readonly CROWD_OPTIMIZE_TOPO: 16;

  // ── Polygon traversal flags ───────────────────────────────────────────────

  /** Poly flag: walkable surface. */
  readonly FLAG_WALK: 0x01;
  /** Poly flag: swimmable surface. */
  readonly FLAG_SWIM: 0x02;
  /** Poly flag: door (can be opened/closed). */
  readonly FLAG_DOOR: 0x04;
  /** Poly flag: jumpable surface. */
  readonly FLAG_JUMP: 0x08;
  /** Poly flag: disabled (agents cannot traverse). */
  readonly FLAG_DISABLED: 0x10;
  /** Poly flag: all flags set. */
  readonly FLAG_ALL: 0xffff;
}

// ─── Factory function ─────────────────────────────────────────────────────────

/**
 * Async factory that initialises the WASM module and returns the recast API.
 *
 * @example
 * import Recast from 'recastjs';
 * const recast = await Recast();
 * recast.settings({ cellSize: 0.3, cellHeight: 0.2, agentHeight: 2, agentRadius: 0.4, agentMaxClimb: 0.9, agentMaxSlope: 45 });
 * await recast.OBJLoader('./scene.obj', () => {});
 * recast.buildSolo();
 */
declare function Recast(): Promise<RecastModule>;

export default Recast;
