/* global recast */

// ─── Formation ────────────────────────────────────────────────────────────────
// Directed geometric formation: each agent holds a numbered slot whose offset
// rotates with the direction of travel (centroid → destination in XZ).
// Supported types: 'circle', 'line', 'square', 'arc'.

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
    default:
      return { x: 0, z: 0 };
  }
}

function Formation(recastInstance, options) {
  this._recast = recastInstance;
  this._agents = (options.agentIds || []).slice(); // ordered array; index = slot number
  this.type    = options.type    !== undefined ? options.type    : 'circle';
  this.spacing = options.spacing !== undefined ? options.spacing : 2.0;
  this._destination  = null;
  this._lastForward  = { x: 1, z: 0 }; // used when centroid ≈ destination
  this._bound = this._onUpdate.bind(this);
  recastInstance.events.on('update', this._bound);
}

Formation.prototype.requestMoveTarget = function(x, y, z) {
  this._destination = { x: x, y: y, z: z };
};

Formation.prototype.addAgent = function(id) {
  if (this._agents.indexOf(id) === -1) this._agents.push(id);
};

Formation.prototype.removeAgent = function(id) {
  var idx = this._agents.indexOf(id);
  if (idx !== -1) this._agents.splice(idx, 1);
};

Formation.prototype.destroy = function() {
  this._recast.events.off('update', this._bound);
  this._agents = [];
  this._destination = null;
};

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
  if (fdLen > 0.1) {
    this._lastForward = { x: fdx / fdLen, z: fdz / fdLen };
  }
  var fwd   = this._lastForward;
  var right = { x: fwd.z, z: -fwd.x }; // 90° clockwise rotation in XZ

  var n = this._agents.length; // total slots (including inactive) for stable geometry
  for (var i = 0; i < members.length; i++) {
    var slot = members[i].slot;
    var a    = members[i].agent;
    var off  = _formationOffset(this.type, slot, n, this.spacing);
    // Rotate local offset into world space
    var wx = right.x * off.x + fwd.x * off.z;
    var wz = right.z * off.x + fwd.z * off.z;
    this._recast.crowdRequestMoveTarget(a.idx, dst.x + wx, dst.y, dst.z + wz);
  }
};

/**
 * Registers Formation on a recast instance.
 * Called via recast.withPlugin(Formation).
 * @param {object} recast - recast instance
 */
Formation.install = function(recast) {
  recast.createFormation = function(options) {
    return new Formation(recast, options || {});
  };
};

recast.Formation = Formation;
