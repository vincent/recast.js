/* global recast */

/**
 * @categoryDescription Formation
 * Directed geometric formation: each agent holds a numbered slot whose offset
 * rotates with the direction of travel (centroid → destination in XZ).
 * Supported types: 'circle', 'line', 'square', 'arc'.
 * @showCategories
 * @module
 */

/**
 * @private
 * Compute the local (pre-rotation) XZ offset for a slot in a formation.
 * @param {'circle'|'line'|'square'|'arc'} type
 * @param {number} slotIndex
 * @param {number} totalSlots
 * @param {number} spacing
 * @returns {{ x: number, z: number }}
 */
function _formationOffset(type, slotIndex, totalSlots, spacing) {
  var i = slotIndex, n = totalSlots, s = spacing;
  switch (type) {
    case 'circle': {
      var angle = (i / n) * 2 * Math.PI;
      var r = s * n * 0.5;
      return {
        x: Math.cos(angle) * r,
        z: Math.sin(angle) * r
      };
    }
    case 'line': {
      return {
        x: s * Math.floor(i / 2),
        z: (i % 2 === 0) ? 0 : -s
      };
    }
    case 'square': {
      var cols = Math.ceil(Math.sqrt(n));
      var rows = Math.ceil(n / cols);
      return {
        x: s * (i % cols) - (cols - 1) * s / 2,
        z: s * Math.floor(i / cols) - (rows - 1) * s / 2
      };
    }
    case 'arc': {
      var angle = (i / Math.max(n - 1, 1)) * Math.PI;
      var r = s * n * 0.5;
      return {
        x: Math.cos(angle) * r,
        z: Math.sin(angle) * r
      };
    }
    default:
      return { x: 0, z: 0 };
  }
}

/**
 * Directed geometric formation where each agent holds a numbered slot.
 * The formation rotates to face the direction of travel (centroid → destination).
 * 
 * * Install via `recast.withPlugin(Formation)`, then use `recast.createFormation(options)`.
 * 
 * @category Formation
 * @param {object} recastInstance - The recast module instance.
 * @param {FormationOptions} options
 */
function Formation(recastInstance, options) {
  this._recast = recastInstance;
  this._agents = (options.agentIds || []).slice(); // ordered array; index = slot number
  this.type    = options.type    !== undefined ? options.type    : 'circle';
  this.spacing = options.spacing !== undefined ? options.spacing : 2.0;
  this._destination  = null;
  this._lastForward  = { x: 1, z: 0 }; // used when centroid ≈ destination
  this._lastTargets  = {}; // agentIdx → { x, z } — last dispatched target
  this._bound = this._onUpdate.bind(this);
  recastInstance.events.on('update', this._bound);
}

/** Set the target destination for the whole formation.
 * @category Formation
 * @param {number} x @param {number} y @param {number} z */
Formation.prototype.requestMoveTarget = function(x, y, z) {
  this._destination = { x: x, y: y, z: z };
  this._lastTargets = {};
};

/** Add an agent to the next available slot.
 * @category Formation
 * @param {number} id
 */
Formation.prototype.addAgent = function(id) {
  if (this._agents.indexOf(id) === -1) this._agents.push(id);
};

/**
 * Remove an agent by ID, freeing its slot.
 * @category Formation
 * @param {number} id
 */
Formation.prototype.removeAgent = function(id) {
  var idx = this._agents.indexOf(id);
  if (idx !== -1) this._agents.splice(idx, 1);
};

/**
 * Stop updating and clean up event listeners.
 * @category Formation
 */
Formation.prototype.destroy = function() {
  this._recast.events.off('update', this._bound);
  this._agents = [];
  this._destination = null;
};

/**
 * @private
 * Called on each crowd `update` event. Computes per-slot world-space targets.
 * @param {CrowdAgent[]} agents - Active agent snapshot from crowdUpdate.
 */
Formation.prototype._onUpdate = function(agents) {
  if (!this._destination || this._agents.length === 0) return;

  // Build a lookup from agent idx → snapshot
  var agentMap = {};
  for (var i = 0; i < agents.length; i++) agentMap[agents[i].idx] = agents[i];

  // Collect active members in slot order
  var members = [];
  for (var i = 0; i < this._agents.length; i++) {
    var a = agentMap[this._agents[i]];
    if (a && a.active) members.push({ slot: i, agent: a });
  }
  if (members.length === 0) return;

  // Centroid (XZ only for direction; Y averaged for height)
  var cx = 0, cz = 0;
  for (var i = 0; i < members.length; i++) {
    cx += members[i].agent.position.x;
    cz += members[i].agent.position.z;
  }
  cx /= members.length;
  cz /= members.length;

  // Forward direction: centroid → destination in XZ.
  // Fall back to last known forward when nearly arrived to avoid flipping.
  var dst = this._destination;
  var fdx = dst.x - cx;
  var fdz = dst.z - cz;
  var fdLen = Math.sqrt(fdx * fdx + fdz * fdz);
  if (fdLen > this.spacing) {
    this._lastForward = { x: fdx / fdLen, z: fdz / fdLen };
  }
  var fwd   = this._lastForward;
  var right = { x: fwd.z, z: -fwd.x }; // 90° clockwise rotation in XZ

  var EPS = 0.1;
  var n = this._agents.length; // total slots (including inactive) for stable geometry
  for (var i = 0; i < members.length; i++) {
    var slot = members[i].slot;
    var a    = members[i].agent;
    var off  = _formationOffset(this.type, slot, n, this.spacing);
    // Rotate local offset into world space
    var wx = right.x * off.x + fwd.x * off.z;
    var wz = right.z * off.x + fwd.z * off.z;
    var tx = dst.x + wx;
    var tz = dst.z + wz;
    var prev = this._lastTargets[a.idx];
    if (!prev || Math.abs(tx - prev.x) > EPS || Math.abs(tz - prev.z) > EPS) {
      this._lastTargets[a.idx] = { x: tx, z: tz };
      this._recast.crowdRequestMoveTarget(a.idx, tx, dst.y, tz);
    }
  }
};

/**
 * Register this plugin on a recast instance. Called internally by `withPlugin`.
 * Adds `recast.createFormation(options)`.
 * @category Formation
 * @param {object} r - The recast module instance.
 */
Formation.install = function(r) {
  r.createFormation = function(options) {
    return new Formation(r, options || {});
  };
};

recast.Formation = Formation;
