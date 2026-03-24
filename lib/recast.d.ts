/**
 * recast.js — TypeScript declarations
 *
 * The library exports an async factory. Consumers must await it:
 * @example
 * // ESM
 * import Recast from 'recastjs';
 * const recast = await Recast();
 * await recast.OBJLoaderAsync('./scene.obj');
 * await recast.buildSoloAsync();
 * const path = await recast.findPathAsync({ x: 0, y: 0, z: 0 }, { x: 10, y: 0, z: 10 });
 *
 * // CommonJS
 * const Recast = require('recastjs');
 * const recast = await Recast();
 */


// ─── Factory function ─────────────────────────────────────────────────────────

/**
 * Async factory that initialises the WASM module and returns the recast API.
 *
 * @example
 * import Recast from 'recastjs';
 * const recast = await Recast();
 * recast.settings({
 *  cellSize: 0.3,
 *  cellHeight: 0.2,
 *  agentHeight: 2,
 *  agentRadius: 0.4,
 *  agentMaxClimb: 0.9,
 *  agentMaxSlope: 45
 * });
 * await recast.OBJLoaderAsync('./scene.obj');
 * await recast.buildSoloAsync();
 * const path = await recast.findPathAsync({ x: 0, y: 0, z: 0 }, { x: 10, y: 0, z: 10 });
 */
declare function Recast(): Promise<RecastModule>;

// ─── Supporting types ────────────────────────────────────────────────────────

/** A 3D point or vector as a plain object. */
export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

/** A navigation mesh polygon as returned by {@link RecastModule.findNearestPolyAsync} and {@link RecastModule.queryPolygonsAsync}. */
export interface NavPoly {
  /** Polygon reference ID. */
  ref: number;
  /** Tile X coordinate. */
  x: number;
  /** Tile Y coordinate. */
  y: number;
  /** Tile layer. */
  layer: number;
  /** Traversal flags bitmask. */
  flags: number;
  /** Area type. */
  area: number;
  /** Polygon vertices. */
  vertices: Vec3[];
}

/** A temporary cylindrical obstacle as returned by {@link RecastModule.getAllTempObstaclesAsync}. */
export interface TempObstacle {
  position: Vec3;
  radius: number;
  height: number;
  /** Internal obstacle state (1 = processing, 2 = active). */
  state: number;
}

/** Navigation mesh configuration passed to {@link RecastModule.settings}. */
export interface RecastSettings {
  /** 
   * Cell size in world units, should be >= 0.0001.
   * Smaller = higher resolution.
   * Default: 0.3
   */
  cellSize: number;
  /** 
   * Cell height in world units, should be >= 0.0001.
   * Smaller = more vertical precision.
   * Default: 0.2 
   */
  cellHeight: number;
  /** 
   * Agent height in world units, should be >= 0.0. 
   * Also the minimum floor-to-ceiling height for a walkable area.
   * Default: 2.0 
   */
  agentHeight: number;
  /**
   * Agent radius (determines clearance from walls).
   * Default: 0.4
   */
  agentRadius: number;
  /**
   * Maximum step height an agent can climb.
   * Default: 0.9
   */
  agentMaxClimb: number;
  /**
   * Maximum walkable slope in degrees.
   * Default: 45
   */
  agentMaxSlope: number;
}

/** Options for adding a crowd agent via {@link RecastModule.addAgent}. */
export interface AgentOptions {
  /** Agent position */
  position: Vec3;
  /** Agent radius */
  radius: number;
  /** Agent height */
  height: number;
  /** Maximum allowed acceleration */
  maxAcceleration: number;
  /** Maximum allowed speed. */
  maxSpeed: number;
  /** Bitmask of `CROWD_*` flags. Defaults to all flags enabled. */
  updateFlags?: number;
  /** 
   * How aggresive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0].
   * A higher value will result in agents trying to stay farther away from each other 
   * at the cost of more difficult steering in tight spaces.
   */
  separationWeight?: number;
}

/** An active crowd agent returned by {@link RecastModule.crowdGetActiveAgents}. */
export interface CrowdAgent {
  /** Agent index in the crowd. */
  idx: number;
  /** Agent position */
  position: Vec3;
  /** Agent velocity */
  velocity: Vec3;
  /** Agent radius */
  radius: number;
  /** 
   * True if the agent is active, false if the agent is in an unused slot in the agent pool.
   */
  active: boolean;
  /**
   * The type of mesh polygon the agent is traversing.
   */
  state: number;
  /**
   * Number of neighbouring agents.
   */
  neighbors: number;
  /**
   * `true` if the agent could only find a partial path to the target.
   */
  partial: boolean;
  /**
   * Agent desired speed. 
   */
  desiredSpeed: number;
}

/** Zone data passed to {@link RecastModule.setZones}. */
export interface ZoneData {
  /** Polygon references belonging to this zone. */
  refs: number[];
  /** Flag bitmasks to OR together as the zone's initial traversal flags (e.g. `[FLAG_WALK]`). */
  initialFlags: number[];
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
  /** Accumulated traversal flags bitmask. */
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
  off(type: string, listener: (...args: any[]) => void): this;
  removeAllListeners(type?: string): this;
  emit(type: string, ...args: any[]): this;
  /** Emits on the next microtask tick. */
  deferEmit(type: string, ...args: any[]): this;
  /** Returns the list of registered event type names. */
  eventNames(): string[];
}

// ─── Main module interface ────────────────────────────────────────────────────

export interface RecastModule {

  // ── Configuration ──────────────────────────────────────────────────────────

  /**
   * Set all navmesh build parameters at once.
   * @example
   * recast.settings({
   *  cellSize: 0.3,
   *  cellHeight: 0.2,
   *  agentHeight: 2,
   *  agentRadius: 0.4,
   *  agentMaxClimb: 0.9,
   *  agentMaxSlope: 45,
   * });
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

  /** Async version of {@link RecastModule.OBJLoader}. */
  OBJLoaderAsync(path: string): Promise<void>;

  /**
   * Load geometry from an OBJ string or buffer already in memory.
   */
  OBJDataLoader(data: string | ArrayBuffer, callback: (module: RecastModule) => void): void;

  /** Async version of {@link RecastModule.OBJDataLoader}. */
  OBJDataLoaderAsync(data: string | ArrayBuffer): Promise<void>;

  // ── Navmesh building ───────────────────────────────────────────────────────

  /**
   * Build a single-tile (solo) navmesh from the loaded geometry.
   * Emits `built` on {@link RecastModule.events} when done.
   */
  buildSolo(): void;

  /** Async version of {@link RecastModule.buildSolo}. Resolves with the navmesh type string. */
  buildSoloAsync(): Promise<string>;

  /**
   * Build a tiled navmesh from the loaded geometry.
   * Emits `built` on {@link RecastModule.events} when done.
   */
  buildTiled(): void;

  /** Async version of {@link RecastModule.buildTiled}. Resolves with the navmesh type string. */
  buildTiledAsync(): Promise<string>;

  // ── Pathfinding ────────────────────────────────────────────────────────────

  /**
   * Async pathfinding between two points.
   * Resolves with an array of waypoints along the path.
   * @param max - Maximum number of waypoints (default: 100).
   */
  findPathAsync(start: Vec3, end: Vec3, max?: number): Promise<Vec3[]>;

  /**
   * Find the navmesh point nearest to the given position within an extent box.
   * Resolves with the nearest point.
   */
  findNearestPointAsync(position: Vec3, extent: Vec3): Promise<Vec3>;

  /**
   * Find the polygon nearest to the given position within an extent box.
   * Resolves with the polygon object, or `null` if none is found.
   */
  findNearestPolyAsync(position: Vec3, extent: Vec3): Promise<NavPoly | null>;

  /**
   * Get a random walkable point on the navmesh.
   * Resolves with the point.
   */
  getRandomPointAsync(): Promise<Vec3>;

  /**
   * Find all polygons within a bounding-box centred at the given position.
   * Resolves with an array of polygon objects, or `null` on failure.
   * @param maxPolys - Maximum number of polygons to return (default: 1000).
   */
  queryPolygonsAsync(posX: number, posY: number, posZ: number, extX: number, extY: number, extZ: number, maxPolys?: number): Promise<NavPoly[] | null>;

  // ── Polygon flags ──────────────────────────────────────────────────────────

  /**
   * Set traversal flags on all polygons within an bounding-box.
   * Use the `FLAG_*` constants for the `flags` bitmask.
   */
  setPolyFlags(posX: number, posY: number, posZ: number, extX: number, extY: number, extZ: number, flags: number): void;

  /**
   * Set traversal flags on a specific polygon by its reference.
   */
  setPolyFlagsByRef(ref: number, flags: number): void;

  // ── Persistence ────────────────────────────────────────────────────────────

  /** Save the current tiled navmesh to `path` on disk (Node.js / Emscripten FS). */
  saveTileMeshAsync(path: string): Promise<void>;

  /** Load a previously saved navmesh from `path`. */
  loadTileMeshAsync(path: string): Promise<void>;

  /** Save the tile cache (dynamic obstacles + navmesh) to `path`. */
  saveTileCacheAsync(path: string): Promise<void>;

  /** Load a previously saved tile cache from `path`. */
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
   * Add a crowd agent.
   * Returns the agent index used to identify it in subsequent calls.
   * @example
   * const id = recast.addAgent({
   *  position: { x: 0, y: 0, z: 0 },
   *  radius: 0.5, height: 1.8,
   *  maxAcceleration: 8,
   *  maxSpeed: 3.5
   * });
   */
  addAgent(options: AgentOptions): number;

  /**
   * Update behavioral parameters of an existing crowd agent.
   * All fields are required — unspecified fields are reset to `0`.
   * Use {@link RecastModule.mergeCrowdAgentParameters} to perform a partial update.
   * Position is managed by the crowd simulation and cannot be set directly.
   */
  updateCrowdAgentParameters(agentId: number, options: Omit<AgentOptions, 'position'>): void;

  /**
   * Read the current behavioral parameters of a crowd agent as a JSON string.
   * Returns `"null"` if the agent index is invalid or the agent is inactive.
   */
  getCrowdAgentParameters(agentId: number): string;

  /**
   * Update a subset of crowd agent parameters.
   * Reads the agent's current parameters first and merges with the provided options,
   * so unspecified fields retain their current values.
   */
  mergeCrowdAgentParameters(agentId: number, options: Partial<Omit<AgentOptions, 'position'>>): void;

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
  crowdRequestMoveVelocity(agentId: number, x: number, y: number, z: number): void;

  /**
   * Advance the crowd simulation by `dt` seconds.
   * Triggers an `update` event on {@link RecastModule.events} with the active agent list.
   */
  crowdUpdate(dt: number): void;

  /**
   * Retrieve all currently active agents via callback.
   */
  crowdGetActiveAgents(callback_id?: number): void;

  // ── Temporary obstacles ────────────────────────────────────────────────────

  /**
   * Add a cylindrical obstacle to the tile cache.
   * Returns the obstacle reference ID, or `-1` on failure.
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

  /** Async: resolves with all active obstacles. */
  getAllTempObstaclesAsync(): Promise<TempObstacle[]>;

  // ── Off-mesh connections ───────────────────────────────────────────────────

  /**
   * Add a custom off-mesh connection (e.g. a jump or ladder link).
   * @param bidirectional - `true` for bidirectional, `false` for one-way.
   */
  addOffMeshConnection(startX: number, startY: number, startZ: number, endX: number, endY: number, endZ: number, radius: number, bidirectional: boolean): void;

  // ── Zones ──────────────────────────────────────────────────────────────────

  /**
   * Named zones keyed by zone name.
   * Populated by {@link RecastModule.setZones}.
   */
  zones: Record<string, Zone>;

  /**
   * Create named zones with initial polygon references and flags.
   * @example
   * recast.setZones({
   *  water: { 
   *    efs: [polyRef1, polyRef2],
   *    initialFlags: [recast.FLAG_SWIM]
   *  }
   * });
   */
  setZones(zones: Record<string, ZoneData>): void;

  /** The Zone constructor — use to create zones manually. */
  Zone: typeof Zone;

  // ── Events ────────────────────────────────────────────────────────────────

  /**
   * The underlying event emitter.
   * Emits:
   * - `built` (type: string) — after `buildSolo()` or `buildTiled()` completes.
   * - `update` (agents: CrowdAgent[]) — after each `crowdUpdate()` call.
   */
  events: RecastEventEmitter;

  /** Shorthand for `recast.events.on(type, listener)`. */
  on(type: string, listener: (...args: any[]) => void): void;
  /** Shorthand for `recast.events.off(type, listener)`. */
  off(type: string, listener: (...args: any[]) => void): void;
  /** Shorthand for `recast.events.emit(type, ...args)`. */
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

export default Recast;
