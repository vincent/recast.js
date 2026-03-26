/* global recast */

// ─── FlockGroup ───────────────────────────────────────────────────────────────
// Loose emergent group behavior: cohesion (pull toward centroid) + optional
// alignment (match average heading). Separation is already handled natively by
// Detour via the DT_CROWD_SEPARATION flag — no duplication needed here.
//
// All steering is applied through crowdRequestMoveTarget (navmesh-aware),
// not crowdRequestMoveVelocity, so agents stay on valid polygons.

/**
 * Loose emergent group behavior: cohesion + optional alignment.
 * Separation is handled natively by Detour's DT_CROWD_SEPARATION flag.
 * Install via `recast.withPlugin(FlockGroup)`, then use `recast.createFlockGroup(options)`.
 * 
 * @category Flock
 * @param {object} recastInstance - The recast module instance.
 * @param {FlockGroupOptions} options
 */
function FlockGroup(recastInstance, options) {
  this._recast = recastInstance;
  this._agentIds = new Set(options.agentIds || []);
  this.cohesionWeight  = options.cohesionWeight  !== undefined ? options.cohesionWeight  : 0.5;
  this.alignmentWeight = options.alignmentWeight !== undefined ? options.alignmentWeight : 0.0;
  this.cohesionRadius  = options.cohesionRadius  !== undefined ? options.cohesionRadius  : Infinity;
  this._destination = null;
  this._bound = this._onUpdate.bind(this);
  recastInstance.events.on('update', this._bound);
}

/** Set the target destination for the whole flock.
 * @param {number} x @param {number} y @param {number} z */
FlockGroup.prototype.requestMoveTarget = function(x, y, z) {
  this._destination = { x: x, y: y, z: z };
};

/** Add an agent to the flock. @param {number} id */
FlockGroup.prototype.addAgent = function(id) {
  this._agentIds.add(id);
};

/** Remove an agent from the flock. @param {number} id */
FlockGroup.prototype.removeAgent = function(id) {
  this._agentIds.delete(id);
};

/** Stop updating and clean up event listeners. */
FlockGroup.prototype.destroy = function() {
  this._recast.events.off('update', this._bound);
  this._agentIds.clear();
  this._destination = null;
};

/**
 * @private
 * Called on each crowd `update` event. Applies cohesion and alignment forces.
 * @param {CrowdAgent[]} agents - Active agent snapshot from crowdUpdate.
 */
FlockGroup.prototype._onUpdate = function(agents) {
  if (!this._destination || this._agentIds.size === 0) return;

  // Collect active members
  var members = [];
  for (var i = 0; i < agents.length; i++) {
    var a = agents[i];
    if (this._agentIds.has(a.idx) && a.active) members.push(a);
  }
  if (members.length === 0) return;

  // Centroid
  var cx = 0, cy = 0, cz = 0;
  for (var i = 0; i < members.length; i++) {
    cx += members[i].position.x;
    cy += members[i].position.y;
    cz += members[i].position.z;
  }
  cx /= members.length;
  cy /= members.length;
  cz /= members.length;

  // Average velocity (for alignment nudge)
  var avx = 0, avy = 0, avz = 0;
  if (this.alignmentWeight > 0) {
    for (var i = 0; i < members.length; i++) {
      avx += members[i].velocity.x;
      avy += members[i].velocity.y;
      avz += members[i].velocity.z;
    }
    var avLen = Math.sqrt(avx * avx + avy * avy + avz * avz);
    if (avLen > 0.001) {
      avx = (avx / avLen) * this.alignmentWeight;
      avy = (avy / avLen) * this.alignmentWeight;
      avz = (avz / avLen) * this.alignmentWeight;
    } else {
      avx = avy = avz = 0;
    }
  }

  var dst = this._destination;
  for (var i = 0; i < members.length; i++) {
    var a = members[i];
    var dx = cx - a.position.x;
    var dy = cy - a.position.y;
    var dz = cz - a.position.z;
    var dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

    var cohX = 0, cohY = 0, cohZ = 0;
    if (dist <= this.cohesionRadius) {
      cohX = dx * this.cohesionWeight;
      cohY = dy * this.cohesionWeight;
      cohZ = dz * this.cohesionWeight;
    }

    this._recast.crowdRequestMoveTarget(
      a.idx,
      dst.x + cohX + avx,
      dst.y + cohY + avy,
      dst.z + cohZ + avz
    );
  }
};

/**
 * Register this plugin on a recast instance. Called internally by `withPlugin`.
 * Adds `recast.createFlockGroup(options)`.
 * @param {object} recast - The recast module instance.
 */
FlockGroup.install = function(recast) {
  recast.createFlockGroup = function(options) {
    return new FlockGroup(recast, options || {});
  };
};

recast.FlockGroup = FlockGroup;
