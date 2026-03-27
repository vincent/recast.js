/* global recast */

/**
 * @categoryDescription Formation
 * Directed geometric formation: each agent holds a numbered slot whose offset
 * rotates with the direction of travel (centroid → destination in XZ).
 * Supported types: 'circle', 'line', 'square', 'arc', 'v', 'wedge',
 * 'hex', 'rings', 'spiral', 'wave'.
 * @showCategories
 * @module
 */

/**
 * @private
 * Compute the local (pre-rotation) XZ offset for a slot in a formation.
 * @param {'circle'|'line'|'square'|'arc'|'v'|'wedge'|'hex'|'rings'|'spiral'|'wave'} type
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
      return {
        x: s * (i % cols),
        z: s * Math.floor(i / cols)
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
    case 'v': {
      if (i === 0) return { x: 0, z: 0 };
      var rank = Math.ceil(i / 2);
      var side = (i % 2 === 1) ? -1 : 1;
      return {
        x: side * rank * s,
        z: -rank * s
      };
    }
    case 'wedge': {
      var remaining = i;
      var row = 0;
      while (remaining >= (row * 2 + 1)) {
        remaining -= (row * 2 + 1);
        row++;
      }
      return {
        x: (remaining - row) * s,
        z: -row * s
      };
    }
    case 'hex': {
      var hexRowHeight = Math.sqrt(3) * 0.5 * s;
      var cols = Math.max(1, Math.ceil(Math.sqrt(n)));
      var row = Math.floor(i / cols);
      var col = i % cols;
      return {
        x: (col + 0.5 * (row % 2)) * s,
        z: row * hexRowHeight
      };
    }
    case 'rings': {
      if (i === 0) return { x: 0, z: 0 };
      var ring = 1;
      var start = 1;
      while (start + ring * 6 <= i) {
        start += ring * 6;
        ring++;
      }
      var ringIndex = i - start;
      var ringSlots = ring * 6;
      var ringAngle = (ringIndex / ringSlots) * 2 * Math.PI;
      return {
        x: Math.cos(ringAngle) * ring * s,
        z: Math.sin(ringAngle) * ring * s
      };
    }
    case 'spiral': {
      var goldenAngle = Math.PI * (3 - Math.sqrt(5));
      var theta = i * goldenAngle;
      var r = s * Math.sqrt(i);
      return {
        x: Math.cos(theta) * r,
        z: Math.sin(theta) * r
      };
    }
    case 'wave': {
      var waveX = (i - (n - 1) * 0.5) * s;
      return {
        x: waveX,
        z: Math.sin(i * 0.75) * s
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
  this._arrived      = false; // true once all agents have reached their slots
  this._bound = this._onUpdate.bind(this);
  recastInstance.events.on('update', this._bound);
}

/** Set the target destination for the whole formation.
 * @category Formation
 * @param {number} x @param {number} y @param {number} z */
Formation.prototype.requestMoveTarget = function(x, y, z) {
  this._destination = { x: x, y: y, z: z };
  this._lastTargets = {};
  this._arrived = false;
};

/** Add an agent to the next available slot.
 * @category Formation
 * @param {number} id
 */
Formation.prototype.addAgent = function(id) {
  if (this._agents.indexOf(id) === -1) {
    this._agents.push(id);
    this._arrived = false;
  }
};

/**
 * Remove an agent by ID, freeing its slot.
 * @category Formation
 * @param {number} id
 */
Formation.prototype.removeAgent = function(id) {
  var idx = this._agents.indexOf(id);
  if (idx !== -1) {
    this._agents.splice(idx, 1);
    this._arrived = false;
  }
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
 * 
 * Non-centered formations: the stability mechanism locks _lastForward only when fdLen ≤ spacing (centroid near destination),
 * which requires the formation's slot centroid to be at (0,0). Only circle satisfied this. 
 * => Compute the actual centroid of all n slot offsets dynamically in _onUpdate and subtract it from each slot before rotation.
 *    Works correctly for all types and any n (including partial grids like n=5 where a formula-based approach fails).
 * 
 * Continuous target overwriting (the real jitter cause): the formation issued new crowdRequestMoveTarget calls every frame,
 * even as the forward direction oscillated during the approach phase.
 * This kept slot positions changing, so agents could never converge.
 * => The "destroy trick" worked by stopping all target issuance.
 * 
 * @param {CrowdAgent[]} agents - Active agent snapshot from crowdUpdate.
 */
Formation.prototype._onUpdate = function(agents) {
  if (!this._destination || this._agents.length === 0) return;
  if (this._arrived) return;

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

  // Compute the centroid of ALL n slot offsets in local space so we can center
  // the formation around the destination regardless of formation type or n.
  var n = this._agents.length; // total slots (including inactive) for stable geometry
  var localCX = 0, localCZ = 0;
  for (var i = 0; i < n; i++) {
    var lo = _formationOffset(this.type, i, n, this.spacing);
    localCX += lo.x;
    localCZ += lo.z;
  }
  localCX /= n;
  localCZ /= n;

  // How close an agent must be to its assigned target before we consider it
  // "arrived" and allow re-evaluation of the slot position.
  var ARRIVE_EPS = this.spacing * 0.4;
  var allArrived = true;

  for (var i = 0; i < members.length; i++) {
    var slot = members[i].slot;
    var a    = members[i].agent;
    var off  = _formationOffset(this.type, slot, n, this.spacing);
    // Center offset and rotate into world space
    var ox = off.x - localCX;
    var oz = off.z - localCZ;
    var wx = right.x * ox + fwd.x * oz;
    var wz = right.z * ox + fwd.z * oz;
    var tx = dst.x + wx;
    var tz = dst.z + wz;

    var prev = this._lastTargets[a.idx];
    if (!prev) {
      // First target for this agent — issue immediately.
      this._lastTargets[a.idx] = { x: tx, z: tz };
      this._recast.crowdRequestMoveTarget(a.idx, tx, dst.y, tz);
      allArrived = false;
    } else {
      var atPrev = Math.abs(a.position.x - prev.x) <= ARRIVE_EPS &&
                   Math.abs(a.position.z - prev.z) <= ARRIVE_EPS;
      if (atPrev) {
        // Agent reached its last target.  Re-issue only if the ideal slot has
        // drifted far enough that a correction is worthwhile.
        if (Math.abs(tx - prev.x) > 0.1 || Math.abs(tz - prev.z) > 0.1) {
          this._lastTargets[a.idx] = { x: tx, z: tz };
          this._recast.crowdRequestMoveTarget(a.idx, tx, dst.y, tz);
          allArrived = false;
        }
        // else: agent is at its slot and the slot hasn't moved — fully arrived.
      } else {
        // Still navigating to the previously issued target — don't interfere.
        allArrived = false;
      }
    }
  }
  if (allArrived) this._arrived = true;
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
