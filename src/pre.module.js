/*
 * recast.js — pre.module.js
 * Injected inside the Emscripten MODULARIZE=1 factory function.
 * Module is provided by the factory (moduleArg).
 */

var ENVIRONMENT_IS_NODE   = typeof process === 'object' && typeof require === 'function';
var ENVIRONMENT_IS_WEB    = typeof window === 'object';
var ENVIRONMENT_IS_WORKER = typeof importScripts === 'function';

/**
 * Tiny event emitter — supports multiple listeners per event type.
 */
function EventEmitter() {
  if (!(this instanceof EventEmitter)) return new EventEmitter();
  EventEmitter.init.call(this);
}

EventEmitter.init = function() {
  this._listeners = {};
};

EventEmitter.prototype._addListener = function(type, listener, once) {
  if (typeof listener !== 'function')
    throw TypeError('listener must be a function');
  if (!this._listeners[type]) this._listeners[type] = [];
  this._listeners[type].push({ once: once, fn: listener });
  return this;
};

EventEmitter.prototype.eventNames = function() {
  return Object.keys(this._listeners);
};

EventEmitter.prototype.on = function(type, listener) {
  return this._addListener(type, listener, false);
};

EventEmitter.prototype.once = function(type, listener) {
  return this._addListener(type, listener, true);
};

EventEmitter.prototype.off = function(type, listener) {
  if (!this._listeners[type]) return this;
  const list = this._listeners[type];
  for (let i = list.length - 1; i >= 0; i--) {
    if (list[i].fn === listener) { list.splice(i, 1); break; }
  }
  if (list.length === 0) delete this._listeners[type];
  return this;
};

EventEmitter.prototype.removeAllListeners = function(type) {
  if (type) {
    delete this._listeners[type];
    return this;
  }
  for (const e in this._listeners) delete this._listeners[e];
  return this;
};

EventEmitter.prototype.emit = function(type) {
  if (!this._listeners[type]) return this;
  const args = Array.prototype.slice.call(arguments, 1);
  const list = this._listeners[type].slice(); // copy so once-removals don't affect iteration
  for (let i = 0; i < list.length; i++) {
    list[i].fn.apply(this, args);
    if (list[i].once) {
      const idx = this._listeners[type] ? this._listeners[type].indexOf(list[i]) : -1;
      if (idx !== -1) this._listeners[type].splice(idx, 1);
    }
  }
  if (this._listeners[type] && this._listeners[type].length === 0) {
    delete this._listeners[type];
  }
  return this;
};

EventEmitter.prototype.deferEmit = function(type) {
  const self = this;
  if (!self._listeners[type]) return self;
  const args = Array.prototype.slice.call(arguments);
  queueMicrotask(function() { self.emit.apply(self, args); });
  return self;
};


/**
 * Bit operations
 */
function bit_test(num, bit)   { return (num & bit) !== 0; }
function bit_set(num, bit)    { return num | bit; }
function bit_clear(num, bit)  { return num & ~bit; }
function bit_toggle(num, bit) { return num ^ bit; }

/**
 * String <=> ArrayBuffer (UTF-8 correct)
 */
function ab2str(buf) {
  return new TextDecoder().decode(buf);
}

function str2ab(str) {
  return new TextEncoder().encode(str).buffer;
}

/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */

// With MODULARIZE=1, Module is provided by the factory; augment with defaults.
if (typeof Module === 'undefined') var Module = {};
if (!Module.canvas) Module.canvas = {};
Module.noInitialRun = true;

const recast = Module;

recast.__RECAST_CALLBACKS = {};
recast.__RECAST_CALLBACKS.size = 0;

recast.__RECAST_OBJECTS = {};

recast.events = new EventEmitter();
recast.on   = recast.events.on.bind(recast.events);
recast.off  = recast.events.off.bind(recast.events);
recast.emit = recast.events.emit.bind(recast.events);

// Log levels — must match RECAST_LOG_* constants in main.cpp and library_recast.js
var LOG_DEBUG = 0, LOG_INFO = 1, LOG_ERROR = 2;

/**
 * Internal log helper. Reads recast.logger at call time (late binding).
 */
function _log(level, msg) {
  var logger = recast.logger;
  if (!logger) return;
  if (level === LOG_ERROR && logger.error) logger.error(msg);
  else if (level === LOG_INFO && logger.info) logger.info(msg);
  else if (level === LOG_DEBUG && logger.debug) logger.debug(msg);
}

/**
 * Fetch helper — replaces the old XMLHttpRequest _ajax helper.
 * Returns a Promise that resolves with text or ArrayBuffer.
 */
function _fetch(url, type) {
  type = type || 'text';
  return fetch(url).then(function(r) {
    if (!r.ok) throw new Error('HTTP ' + r.status + ' fetching ' + url);
    return type === 'arraybuffer' ? r.arrayBuffer() : r.text();
  });
}

var _OBJDataLoader = function(contents, callback) {
  _log(LOG_INFO, 'loading obj geometry');
  recast.initWithFileContent(contents.toString());
  _log(LOG_INFO, 'obj geometry loaded');
  callback(recast);
};

//// Worker message handler ////

var workerMain = function(event) {
  if (!event.data) return;

  const message = event.data;

  switch (message.type) {

    case 'ping':
      postMessage({
        vent: true,
        type: message.type,
        callback: message.callback
      });
      break;

    case 'set_cellSize':
    case 'set_cellHeight':
    case 'set_agentHeight':
    case 'set_agentRadius':
    case 'set_agentMaxClimb':
    case 'set_agentMaxSlope':
      recast[message.type](message.data);
      break;

    case 'config':
    case 'settings':
      recast.set_cellSize(message.data.cellSize);
      recast.set_cellHeight(message.data.cellHeight);
      recast.set_agentHeight(message.data.agentHeight);
      recast.set_agentRadius(message.data.agentRadius);
      recast.set_agentMaxClimb(message.data.agentMaxClimb);
      recast.set_agentMaxSlope(message.data.agentMaxSlope);
      postMessage({
        vent: true,
        type: message.type,
        callback: message.callback
      });
      break;

    case 'build':
      recast.build();
      postMessage({
        vent: true,
        type: message.type,
        callback: message.callback
      });
      break;

    case 'buildSolo':
      recast.buildSolo();
      postMessage({
        vent: true,
        type: message.type,
        callback: message.callback
      });
      break;

    case 'buildTiled':
      recast.buildTiled();
      postMessage({
        vent: true,
        type: message.type,
        callback: message.callback
      });
      break;

    case 'initCrowd':
      recast.initCrowd(1000, 1.0);
      break;

    case 'getNavMeshVertices':
      recast.getNavMeshVertices(recast.cb(function(data) {
        postMessage({
          type: message.type,
          data: Array.prototype.slice.call(arguments),
          callback: message.callback
        });
      }));
      break;

    case 'OBJLoader':
      recast.OBJLoader(message.data, function() {
        postMessage({
          type: message.type,
          callback: message.callback
        });
      });
      break;

    case 'OBJDataLoader':
      _OBJDataLoader(message.data, function() {
        postMessage({
          type: message.type,
          callback: message.callback
        });
      });
      break;

    case 'saveTileMesh':
      recast.saveTileMesh(message.data, recast.cb(function() {
        postMessage({
          type: message.type,
          data: Array.prototype.slice.call(arguments),
          callback: message.callback
        });
      }));
      break;

    case 'loadTileMesh':
      recast.loadTileMesh(message.data, recast.cb(function() {
        postMessage({
          type: message.type,
          data: Array.prototype.slice.call(arguments),
          callback: message.callback
        });
      }));
      break;

    case 'findNearestPoint':
      recast.findNearestPoint(
        message.data.position.x,
        message.data.position.y,
        message.data.position.z,
        message.data.extend.x,
        message.data.extend.y,
        message.data.extend.z,
        recast.cb(function(px, py, pz) {
          postMessage({
            type: message.type,
            data: [px, py, pz],
            callback: message.callback
          });
        })
      );
      break;

    case 'findNearestPoly':
      recast.findNearestPoly(
        message.data.position.x,
        message.data.position.y,
        message.data.position.z,
        message.data.extend.x,
        message.data.extend.y,
        message.data.extend.z,
        recast.cb(function() {
          postMessage({
            type: message.type,
            data: Array.prototype.slice.call(arguments),
            callback: message.callback
          });
        })
      );
      break;

    case 'queryPolygons':
      recast.queryPolygons(
        message.data.position.x,
        message.data.position.y,
        message.data.position.z,
        message.data.extend.x,
        message.data.extend.y,
        message.data.extend.z,
        message.data.maxPolys,
        recast.cb(function() {
          postMessage({
            type: message.type,
            data: Array.prototype.slice.call(arguments),
            callback: message.callback
          });
        })
      );
      break;

    case 'findPath':
      recast.findPath(message.data.sx, message.data.sy, message.data.sz, message.data.dx, message.data.dy, message.data.dz, message.data.max, recast.cb(function(path) {
        postMessage({
          type: message.type,
          data: [path],
          callback: message.callback
        });
      }));
      break;

    case 'getRandomPoint':
      recast.getRandomPoint(recast.cb(function(px, py, pz) {
        postMessage({
          type: message.type,
          data: [px, py, pz],
          callback: message.callback
        });
      }));
      break;

    case 'setPolyFlags':
      recast.setPolyFlags(message.data.sx, message.data.sy, message.data.sz, message.data.dx, message.data.dy, message.data.dz, message.data.flags);
      postMessage({
        vent: true,
        type: message.type,
        data: message.data,
        callback: message.callback
      });
      break;

    case 'setPolyFlagsByRef':
      recast.setPolyFlagsByRef(message.data.ref, message.data.flags);
      postMessage({
        vent: true,
        type: message.type,
        data: message.data,
        callback: message.callback
      });
      break;

    case 'addCrowdAgent':
      var idx = recast.addCrowdAgent(
        message.data.position.x,
        message.data.position.y,
        message.data.position.z,
        message.data.radius,
        message.data.height,
        message.data.maxAcceleration,
        message.data.maxSpeed,
        message.data.updateFlags,
        message.data.separationWeight
      );
      postMessage({
        vent: true,
        type: message.type,
        data: [idx],
        callback: message.callback
      });
      break;

    case 'updateCrowdAgentParameters':
      recast.updateCrowdAgentParameters(message.data.agent, message.data.options);
      break;

    case 'crowdRequestMoveVelocity':
      recast.crowdRequestMoveVelocity(message.data.agent, message.data.velocity.x, message.data.velocity.y, message.data.velocity.z);
      break;

    case 'removeCrowdAgent':
      recast.removeCrowdAgent(message.data);
      postMessage({
        vent: true,
        type: message.type,
        data: [message.data],
        callback: message.callback
      });
      break;

    case 'crowdRequestMoveTarget':
      recast.crowdRequestMoveTarget(message.data.agent, message.data.x, message.data.y, message.data.z);
      break;

    case 'crowdUpdate':
      recast._crowdUpdate(message.data);
      recast._crowdGetActiveAgents(!message.callback ? -1 : recast.cb(function() {
        postMessage({
          type: message.type,
          data: [agentPoolBuffer],
          callback: message.callback
        });
      }));
      break;

    case 'crowdGetActiveAgents':
      recast._crowdGetActiveAgents(!message.callback ? -1 : recast.cb(function() {
        postMessage({
          vent: true,
          type: message.type,
          data: [agentPoolBuffer],
          callback: message.callback
        });
      }));
      break;

    case 'addTempObstacle':
      recast.addTempObstacle(message.data.posX, message.data.posY, message.data.posZ, message.data.radius);
      postMessage({
        vent: true,
        type: message.type,
        data: [message.data],
        callback: message.callback
      });
      break;

    case 'removeTempObstacle':
      recast.removeTempObstacle(message.data);
      postMessage({
        vent: true,
        type: message.type,
        data: [message.data],
        callback: message.callback
      });
      break;

    case 'removeAllTempObstacles':
      recast.removeAllTempObstacles();
      postMessage({
        vent: true,
        type: message.type,
        data: [message.data],
        callback: message.callback
      });
      break;

    case 'getAllTempObstacles':
      recast.getAllTempObstacles(message.data.callback_id);
      postMessage({
        vent: true,
        type: message.type,
        data: [message.data],
        callback: message.callback
      });
      break;

    case 'addOffMeshConnection':
      recast.addOffMeshConnection(message.data.startX, message.data.startY, message.data.startZ, message.data.endX, message.data.endY, message.data.endZ, message.data.radius, message.data.bidir);
      postMessage({
        vent: true,
        type: message.type,
        data: [message.data],
        callback: message.callback
      });
      break;

    case 'rebuildAllTiles':
      recast.rebuildAllTiles();
      postMessage({
        vent: true,
        type: message.type,
      });
      break;

    default:
      _log(LOG_ERROR, 'Unknown worker message type: ' + message.type);
      throw new Error(message.type + ' is not a known Recast method');
  }
};

if (ENVIRONMENT_IS_WEB || ENVIRONMENT_IS_WORKER) {
  onmessage = workerMain;
} else if (ENVIRONMENT_IS_NODE) {
  process.on('message', function(message) {
    workerMain(message);
  });

  postMessage = function(message) {
    if (process.send) {
      process.send({ data: message });
    }
  };
}

//// exported recast module functions ////

// UpdateFlags
recast.CROWD_ANTICIPATE_TURNS   = 1;
recast.CROWD_OBSTACLE_AVOIDANCE = 2;
recast.CROWD_SEPARATION         = 4;
recast.CROWD_OPTIMIZE_VIS       = 8;
recast.CROWD_OPTIMIZE_TOPO      = 16;

// Polyflags
recast.FLAG_WALK     = 0x01;
recast.FLAG_SWIM     = 0x02;
recast.FLAG_DOOR     = 0x04;
recast.FLAG_JUMP     = 0x08;
recast.FLAG_DISABLED = 0x10;
recast.FLAG_ALL      = 0xffff;

recast.setGLContext = function(gl_context) {
  recast.glContext = gl_context;
};

/**
 * Callback registration — auto-increment ID, auto-delete on first call.
 * Replaces the old ring-of-10 approach.
 */
recast._cbId = 0;
recast.cb = function(func) {
  const id = ++recast._cbId;
  recast.__RECAST_CALLBACKS[id] = function() {
    delete recast.__RECAST_CALLBACKS[id];
    func.apply(this, arguments);
  };
  return id;
};

recast.drawObject = function(objectName) {
  const object = recast.__RECAST_OBJECTS[objectName];

  if (!object) {
    throw new Error(objectName + ' is not a valid object, or has not been created');
  }

  for (let i = 0; i < object.buffers.length; i++) {
    recast.glContext.bindBuffer(recast.glContext.ARRAY_BUFFER, object.buffers[i]);
    recast.glContext.bufferData(recast.glContext.ARRAY_BUFFER, object.datas[i], recast.glContext.STATIC_DRAW);
    recast.glContext.drawArrays(recast.glContext.TRIANGLES, 0, object.buffers[i].numItems);
  }
};

recast.settings = function(options) {
  recast.set_cellSize(options.cellSize);
  recast.set_cellHeight(options.cellHeight);
  recast.set_agentHeight(options.agentHeight);
  recast.set_agentRadius(options.agentRadius);
  recast.set_agentMaxClimb(options.agentMaxClimb);
  recast.set_agentMaxSlope(options.agentMaxSlope);
};

recast.OBJDataLoader = function(data, callback) {
  _OBJDataLoader(data, callback);
};

recast.OBJLoader = function(path, callback) {
  _log(LOG_INFO, 'obj loader: loading ' + path);
  if (ENVIRONMENT_IS_NODE) {
    const fs = require('fs');
    fs.readFile(path, function(err, data) {
      if (err) throw new Error(err);
      _OBJDataLoader(data, callback);
    });
  } else {
    _fetch(path).then(function(data) {
      _OBJDataLoader(data, callback);
    });
  }
};

recast.addAgent = function(options) {
  return recast.addCrowdAgent(
    options.position.x,
    options.position.y,
    options.position.z,
    options.radius,
    options.height,
    options.maxAcceleration,
    options.maxSpeed,
    options.updateFlags,
    options.separationWeight
  );
};

recast.crowdRequestMoveVelocity = function(agentId, x, y, z) {
  return recast.requestMoveVelocity(agentId, x, y, z);
};

// Install wrappers over C++ bindings after WASM/embind init.
var _prevOnRuntimeInitialized = Module.onRuntimeInitialized;
Module.onRuntimeInitialized = function() {
  _log(LOG_INFO, 'Recast WASM module initialized');
  if (_prevOnRuntimeInitialized) _prevOnRuntimeInitialized();

  var _rawUpdateParams = recast.updateCrowdAgentParameters;
  recast.updateCrowdAgentParameters = function(agentId, options) {
    _rawUpdateParams(agentId,
      0, 0, 0,
      options.radius           !== undefined ? options.radius           : 0,
      options.height           !== undefined ? options.height           : 0,
      options.maxAcceleration  !== undefined ? options.maxAcceleration  : 0,
      options.maxSpeed         !== undefined ? options.maxSpeed         : 0,
      options.updateFlags      !== undefined ? options.updateFlags      : 0,
      options.separationWeight !== undefined ? options.separationWeight : 0
    );
  };

  recast.mergeCrowdAgentParameters = function(agentId, options) {
    var current = JSON.parse(recast.getCrowdAgentParameters(agentId)) || {};
    recast.updateCrowdAgentParameters(agentId, {
      radius:           options.radius           !== undefined ? options.radius           : current.radius           || 0,
      height:           options.height           !== undefined ? options.height           : current.height           || 0,
      maxAcceleration:  options.maxAcceleration  !== undefined ? options.maxAcceleration  : current.maxAcceleration  || 0,
      maxSpeed:         options.maxSpeed         !== undefined ? options.maxSpeed         : current.maxSpeed         || 0,
      updateFlags:      options.updateFlags      !== undefined ? options.updateFlags      : current.updateFlags      || 0,
      separationWeight: options.separationWeight !== undefined ? options.separationWeight : current.separationWeight || 0,
    });
  };
};

recast.findNearest = function(position, callback_id) {
  return recast.findNearestPoint(
    position.x,
    position.y,
    position.z,
    3, 3, 3,
    callback_id
  );
};

recast.crowdGetActiveAgents = function(callback_id) {
  return recast._crowdGetActiveAgents(callback_id || -1);
};

recast.crowdUpdate = function (dt) {
  recast._crowdUpdate(dt);
  recast._crowdGetActiveAgents(-1);
};

recast.queryPolygons = function(posX, posY, posZ, extX, extY, extZ, maxPolys, callback_id) {
  if (typeof callback_id === 'undefined' && typeof maxPolys === 'number') {
    callback_id = maxPolys;
    maxPolys = 1000;
  }
  return recast._queryPolygons(posX, posY, posZ, extX, extY, extZ, maxPolys, callback_id);
};

recast.saveTileMesh = function(path, callback_id) {
  _log(LOG_INFO, 'saveTileMesh: ' + path);
  recast._saveTileMesh(path, callback_id);
};

recast.loadTileMesh = function(path, callback_id) {
  _log(LOG_INFO, 'loadTileMesh: ' + path);
  const fsPath = '/tmp/' + path.split('/').pop();
  if (ENVIRONMENT_IS_NODE) {
    const fs = require('fs');
    fs.readFile(path, function(err, data) {
      if (err) throw new Error(err);
      FS.writeFile(fsPath, data);
      recast._loadTileMesh(fsPath, callback_id);
    });
  } else {
    _fetch(path, 'arraybuffer').then(function(data) {
      FS.writeFile(fsPath, new Int8Array(data));
      recast._loadTileMesh(fsPath, callback_id);
    });
  }
};

recast.saveTileCache = function(path, callback_id) {
  _log(LOG_INFO, 'saveTileCache: ' + path);
  recast._saveTileCache(path, callback_id);
};

recast.loadTileCache = function(path, callback_id) {
  _log(LOG_INFO, 'loadTileCache: ' + path);
  const fsPath = '/tmp/' + path.split('/').pop();
  if (ENVIRONMENT_IS_NODE) {
    const fs = require('fs');
    fs.readFile(path, function(err, data) {
      if (err) throw new Error(err);
      FS.writeFile(fsPath, data);
      recast._loadTileCache(fsPath, callback_id);
    });
  } else {
    _fetch(path, 'arraybuffer').then(function(data) {
      FS.writeFile(fsPath, new Int8Array(data));
      recast._loadTileCache(fsPath, callback_id);
    });
  }
};

//// Async wrappers ////

recast.OBJLoaderAsync = function(path) {
  return new Promise(function(resolve) {
    recast.OBJLoader(path, function() { resolve(); });
  });
};

recast.OBJDataLoaderAsync = function(data) {
  return new Promise(function(resolve) {
    recast.OBJDataLoader(data, function() { resolve(); });
  });
};

recast.buildSoloAsync = function() {
  return new Promise(function(resolve) {
    _log(LOG_INFO, 'buildSolo: starting');
    recast.events.once('built', function(type) {
      _log(LOG_INFO, 'buildSolo: complete');
      resolve(type);
    });
    recast.buildSolo();
  });
};

recast.buildTiledAsync = function() {
  return new Promise(function(resolve) {
    _log(LOG_INFO, 'buildTiled: starting');
    recast.events.once('built', function(type) {
      _log(LOG_INFO, 'buildTiled: complete');
      resolve(type);
    });
    recast.buildTiled();
  });
};

recast.findPathAsync = function(start, end, max) {
  return new Promise(function(resolve) {
    recast.findPath(start.x, start.y, start.z, end.x, end.y, end.z, max || 100, recast.cb(function(path) {
      _log(LOG_DEBUG, 'findPath: ' + (path ? path.length : 0) + ' waypoints');
      resolve(path);
    }));
  });
};

recast.getRandomPointAsync = function() {
  return new Promise(function(resolve) {
    recast.getRandomPoint(recast.cb(function(x, y, z) {
      resolve({ x: x, y: y, z: z });
    }));
  });
};

recast.findNearestPointAsync = function(position, extent) {
  return new Promise(function(resolve) {
    recast.findNearestPoint(position.x, position.y, position.z, extent.x, extent.y, extent.z, recast.cb(function(x, y, z) {
      resolve({ x: x, y: y, z: z });
    }));
  });
};

recast.findNearestPolyAsync = function(position, extent) {
  return new Promise(function(resolve) {
    recast.findNearestPoly(position.x, position.y, position.z, extent.x, extent.y, extent.z, recast.cb(resolve));
  });
};

recast.queryPolygonsAsync = function(posX, posY, posZ, extX, extY, extZ, maxPolys) {
  return new Promise(function(resolve) {
    recast.queryPolygons(posX, posY, posZ, extX, extY, extZ, maxPolys || 1000, recast.cb(resolve));
  });
};

recast.getAllTempObstaclesAsync = function() {
  return new Promise(function(resolve) {
    recast.getAllTempObstacles(recast.cb(resolve));
  });
};

recast.saveTileMeshAsync = function(path) {
  return new Promise(function(resolve) {
    recast.saveTileMesh(path, recast.cb(function() {
      resolve(Array.prototype.slice.call(arguments));
    }));
  });
};

recast.loadTileMeshAsync = function(path) {
  return new Promise(function(resolve) {
    recast.loadTileMesh(path, recast.cb(resolve));
  });
};

recast.saveTileCacheAsync = function(path) {
  return new Promise(function(resolve) {
    recast.saveTileCache(path, recast.cb(function() {
      resolve(Array.prototype.slice.call(arguments));
    }));
  });
};

recast.loadTileCacheAsync = function(path) {
  return new Promise(function(resolve) {
    recast.loadTileCache(path, recast.cb(resolve));
  });
};

//////////////////////////////////////////

recast.zones = {};

recast.setZones = function(zones) {
  const zonesKeys = Object.keys(zones);
  for (let i = zonesKeys.length - 1; i >= 0; i--) {
    const zone = new Zone(zonesKeys[i], zones[zonesKeys[i]]);
    recast.zones[zone.name] = zone;
  }
};

recast.Zone = Zone;

function Zone(name, data) {
  this.name  = name;
  this.refs  = data.refs;
  this.flags = 0;

  const initialFlags = data.initialFlags || data.flags || [];
  for (let i = 0; i < initialFlags.length; i++) {
    this.setFlags(initialFlags[i]);
  }

  this.syncFlags();
}

Zone.prototype.syncFlags = function() {
  for (let i = 0; i < this.refs.length; i++) {
    recast.setPolyFlagsByRef(this.refs[i], this.flags);
  }
  return this;
};

Zone.prototype.isWalkable = function() {
  return this.is(recast.FLAG_WALK);
};

Zone.prototype.is = function(flags) {
  return bit_test(this.flags, flags);
};

Zone.prototype.setFlags = function(flags) {
  this.flags = Math.min(65535, bit_set(this.flags, flags));
  return this.syncFlags();
};

Zone.prototype.clearFlags = function(flags) {
  this.flags = bit_clear(this.flags, flags);
  return this.syncFlags();
};

Zone.prototype.toggleFlags = function(flags) {
  this.flags = Math.min(65535, bit_toggle(this.flags, flags));
  return this.syncFlags();
};

//////////////////////////////////////////

function AgentPool(n) {
  this.__pools = [];
  let i = 0;
  while (i < n) {
    this.__pools[i] = { position: {}, velocity: {} };
    i++;
  }
}

AgentPool.prototype.get = function(idx, position_x, position_y, position_z,
                                   velocity_x, velocity_y, velocity_z,
                                   radius, active, state, neighbors,
                                   partial, desiredSpeed)
{
  if (this.__pools.length > 0) {
    const ag = this.__pools.pop();
    ag.idx          = idx;
    ag.position.x   = position_x;
    ag.position.y   = position_y;
    ag.position.z   = position_z;
    ag.velocity.x   = velocity_x;
    ag.velocity.y   = velocity_y;
    ag.velocity.z   = velocity_z;
    ag.radius       = radius;
    ag.active       = !!active;
    ag.state        = state;
    ag.neighbors    = neighbors;
    ag.partial      = !!partial;
    ag.desiredSpeed = desiredSpeed;
    return ag;
  }
  return null;
};

AgentPool.prototype.add = function(v) {
  this.__pools.push(v);
};

var agentPool = new AgentPool(1000);
var agentPoolBuffer = [];
agentPool.ready = true;

//////////////////////////////////////////

function VectorPool(n) {
  this.__pools = [];
  let i = 0;
  while (i < n) {
    this.__pools[i] = { x: 0, y: 0, z: 0 };
    i++;
  }
}

VectorPool.prototype.get = function(x, y, z) {
  if (this.__pools.length > 0) {
    const v = this.__pools.pop();
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
  }
  return null;
};

VectorPool.prototype.add = function(v) {
  this.__pools.push(v);
};

var vectorPool = new VectorPool(10000);
vectorPool.ready = true;

//////////////////////////////////////////
