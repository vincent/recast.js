/*jshint onevar: false, indent:4, strict: false */
/*global setImmediate: false, setTimeout: false, console: false, module: true, process: true, define: true, onmessage: true, postMessage: true, require: true */

var ENVIRONMENT_IS_NODE = typeof process === 'object' && typeof require === 'function';
var ENVIRONMENT_IS_WEB = typeof window === 'object';
var ENVIRONMENT_IS_WORKER = typeof importScripts === 'function';

/**
 * Tiny event emitter for Node.js and the browser
 * from https://github.com/joaquimserafim/tiny-eventemitter
 */
function EventEmitter () {
  if (!(this instanceof EventEmitter)) return new EventEmitter();
  EventEmitter.init.call(this);
}

EventEmitter.init = function () {
  this._listeners = {};
};

EventEmitter.prototype._addListenner = function (type, listener, once) {
  if (typeof listener !== 'function')
    throw TypeError('listener must be a function');

  if (!this._listeners[type])
    this._listeners[type] = {
      once: once,
      fn: function () {
        return listener.apply(this, arguments);
      }
    };

  return this;
};

EventEmitter.prototype.listeners = function () {
  return Object.keys(this._listeners);
};

EventEmitter.prototype.on = function (type, listener) {
  return this._addListenner(type, listener, 0);
};

EventEmitter.prototype.once = function (type, listener) {
  return this._addListenner(type, listener, 1);
};

EventEmitter.prototype.remove = function (type) {
  if (type) {
    delete this._listeners[type];
    return this;
  }

  for (var e in this._listeners) delete this._listeners[e];

  return this;
};

EventEmitter.prototype.emit = function (type) {
  if (!this._listeners[type])
    return;

  var args = Array.prototype.slice.call(arguments, 1);

  // exec event
  this._listeners[type].fn.apply(this, args);

  // remove events that run only once
  if (this._listeners[type].once) this.remove(type);

  return this;
};

EventEmitter.prototype.deferEmit = function (type) {
  var self = this;

  if (!self._listeners[type])
    return;

  var args = Array.prototype.slice.call(arguments, 1);

  process.nextTick(function () {
    // exec event
    self._listeners[type].fn.apply(self, args);

    // remove events that run only once
    if (self._listeners[type].once) self.remove(type);
  });

  return self;
};


/**
 * Bit operations, from http://stackoverflow.com/a/8436459/1689894
 */

function bit_test(num, bit){
    return (num & bit) !== 0;
}

function bit_set(num, bit){
    return num |= bit;
}

function bit_clear(num, bit){
    return num &= ~bit;
}

function bit_toggle(num, bit){
    return num ^= bit;
}

/**
 * String <=> ArrayBuffer
 */
function ab2str(buf) {
  return String.fromCharCode.apply(null, new Uint16Array(buf));
}

function str2ab(str) {
  var buf = new ArrayBuffer(str.length*2); // 2 bytes for each char
  var bufView = new Uint8Array(buf);
  for (var i=0, strLen=str.length; i<strLen; i++) {
    bufView[i] = str.charCodeAt(i);
  }
  return buf;
}

/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
var Module = {
    canvas: {},
    noInitialRun: true,
    noFSInit: true
};
var recast = Module;


// global on the server, window in the browser
var root, previous_recast;

root = this;
if (root !== null) {
  previous_recast = root.recast;
}

recast.__RECAST_CALLBACKS = {};
recast.__RECAST_CALLBACKS.size = 0;

recast.__RECAST_OBJECTS = {};

recast.vent = new EventEmitter();
recast.on = recast.vent.on;
recast.emit = recast.vent.emit;
recast.deferEmit = recast.vent.deferEmit;

var _ajax = function(url, data, callback, type, responseType) {
  var data_array, data_string, idx, req, value;
  if (! data) {
    data = {};
  }
  if (! callback) {
    callback = function() {};
  }
  if (! type) {
    //default to a GET request
    type = 'GET';
  }
  data_array = [];
  for (idx in data) {
    value = data[idx];
    data_array.push('' + idx + '=' + value);
  }
  data_string = data_array.join('&');
  req = new XMLHttpRequest();
  req.open(type, url, true);
  req.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  if (responseType) {
    req.responseType = 'arraybuffer';responseType
  }
  req.onreadystatechange = function() {
    if (req.readyState == 4 && req.status == 200) {
      return callback(responseType === 'arraybuffer' ? req.response : req.responseText);
    }
  };
  // debug('ajax request', data_string);
  req.send(data_string);
  return req;
};

var _OBJDataLoader = function (contents, callback) {
  recast.initWithFileContent(contents.toString());
  callback(recast);
};

//// nextTick implementation with browser-compatible fallback ////
if (typeof process === 'undefined' || !(process.nextTick)) {
    if (typeof setImmediate === 'function') {
        recast.nextTick = function (fn) {
            // not a direct alias for IE10 compatibility
            setImmediate(fn);
        };
        recast.setImmediate = recast.nextTick;
    }
    else {
        recast.nextTick = function (fn) {
            setTimeout(fn, 0);
        };
        recast.setImmediate = recast.nextTick;
    }
}
else {
    recast.nextTick = process.nextTick;
    if (typeof setImmediate !== 'undefined') {
        recast.setImmediate = function (fn) {
          // not a direct alias for IE10 compatibility
          setImmediate(fn);
        };
    }
    else {
        recast.setImmediate = recast.nextTick;
    }
}

//// link worker recast module functions ////

var workerMain = function(event) {
  if (! event.data) {
    return;
  }

  var message = event.data;

  switch(message.type) {

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
        recast.cb(function(px, py, pz){
          postMessage({
            type: message.type,
            data: [ px, py, pz ],
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
        recast.cb(function(){
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
        recast.cb(function(){
          postMessage({
            type: message.type,
            data: Array.prototype.slice.call(arguments),
            callback: message.callback
          });
        })
      );
      break;

    case 'findPath':
      recast.findPath(message.data.sx, message.data.sy, message.data.sz, message.data.dx, message.data.dy, message.data.dz, message.data.max, recast.cb(function(path){
        postMessage({
          type: message.type,
          data: [ path ],
          callback: message.callback
        });
      }));
      break;

    case 'getRandomPoint':
      recast.getRandomPoint(recast.cb(function(px, py, pz){
        postMessage({
          type: message.type,
          data: [ px, py, pz ],
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
        data: [ idx ],
        callback: message.callback
      });
      break;

    case 'updateCrowdAgentParameters':
      recast.updateCrowdAgentParameters(message.data.agent,
        message.data.options.position.x,
        message.data.options.position.y,
        message.data.options.position.z,
        message.data.options.radius,
        message.data.options.height,
        message.data.options.maxAcceleration,
        message.data.options.maxSpeed,
        message.data.options.updateFlags,
        message.data.options.separationWeight
      );
      break;

    case 'requestMoveVelocity':
      recast.requestMoveVelocity(message.data.agent, message.data.velocity.x, message.data.velocity.y, message.data.velocity.z);
      break;

    case 'removeCrowdAgent':
      recast.removeCrowdAgent(message.data);
      postMessage({
        vent: true,
        type: message.type,
        data: [ message.data ],
        callback: message.callback
      });
      break;

    case 'crowdRequestMoveTarget':
      recast.crowdRequestMoveTarget(message.data.agent, message.data.x, message.data.y, message.data.z);
      break;

    case 'crowdUpdate':
      recast.crowdUpdate(message.data);
      recast._crowdGetActiveAgents(! message.callback ? -1 : recast.cb(function(){
        postMessage({
          type: message.type,
          data: [ agentPoolBuffer ],
          callback: message.callback
        });
      }));
      break;

    case 'crowdGetActiveAgents':
      recast._crowdGetActiveAgents(! message.callback ? -1 : recast.cb(function(){
        postMessage({
          vent: true,
          type: message.type,
          data: [ agentPoolBuffer ],
          callback: message.callback
        });
      }));
      break;

    case 'addTempObstacle':
      recast.addTempObstacle(message.data.posX, message.data.posY, message.data.posZ, message.data.radius);
      postMessage({
        vent: true,
        type: message.type,
        ddata: [ message.data ],
        callback: message.callback
      });
      break;

    case 'removeTempObstacle':
      recast.removeTempObstacle(message.data);
      postMessage({
        vent: true,
        type: message.type,
        ddata: [ message.data ],
        callback: message.callback
      });
      break;

    case 'removeAllTempObstacles':
      recast.removeAllTempObstacles();
      postMessage({
        vent: true,
        type: message.type,
        ddata: [ message.data ],
        callback: message.callback
      });
      break;

    case 'getAllTempObstacles':
      recast.getAllTempObstacles(message.data.callback_id);
      postMessage({
        vent: true,
        type: message.type,
        ddata: [ message.data ],
        callback: message.callback
      });
      break;

    case 'addOffMeshConnection':
      recast.addOffMeshConnection(message.data.startX, message.data.startY, message.data.startZ, message.data.endX, message.data.endY, message.data.endZ, message.data.radius, message.data.bidir);
      postMessage({
        vent: true,
        type: message.type,
        ddata: [ message.data ],
        callback: message.callback
      });
      break;

    default:
      throw new Error(message.type + ' is not a known Recast method');

  }
};

if (ENVIRONMENT_IS_WEB || ENVIRONMENT_IS_WORKER) {
  onmessage = workerMain;
  // postMessage = postMessage;

} else if (ENVIRONMENT_IS_NODE) {
  process.on('message', function(message) {
    workerMain(message);
  });

  postMessage = function (message) {
    if (process.send) { // not in the main process
      process.send({ data: message });
    }
  };

}

//// exported recast module functions ////

// UpdateFlags
recast.CROWD_ANTICIPATE_TURNS   = 1;
recast.CROWD_OBSTACLE_AVOIDANCE = 2;
recast.CROWD_SEPARATION         = 4;
recast.CROWD_OPTIMIZE_VIS       = 8;          ///< Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
recast.CROWD_OPTIMIZE_TOPO      = 16;        ///< Use dtPathCorridor::optimizePathTopology() to optimize the agent path.

// Polyflags
recast.FLAG_WALK       = 0x01;
recast.FLAG_SWIM       = 0x02;
recast.FLAG_DOOR       = 0x04;
recast.FLAG_JUMP       = 0x08;
recast.FLAG_DISABLED   = 0x10;
recast.FLAG_ALL        = 0xffff;

recast.setGLContext = function (gl_context) {
  recast.glContext = gl_context;
};


recast.cb = function (func) {
  recast.__RECAST_CALLBACKS.size = recast.__RECAST_CALLBACKS.size % 10;
  var last = (++recast.__RECAST_CALLBACKS.size) - 1;
  recast.__RECAST_CALLBACKS[last] = func;
  // recast.__RECAST_CALLBACKS[last].__debug = 'callback_id#' + last;
  return last;
};

recast.drawObject = function (objectName) {
  var object = recast.__RECAST_OBJECTS[objectName];

  if (! object) {
    throw new Error(objectName + ' is not a valid object, or has not ben created');
  }

  // recast.glContext.clear(recast.glContext.COLOR_BUFFER_BIT | recast.glContext.DEPTH_BUFFER_BIT);

  for (var i = 0; i < object.buffers.length; i++) {
    recast.glContext.bindBuffer(recast.glContext.ARRAY_BUFFER, object.buffers[i]);
    recast.glContext.bufferData(recast.glContext.ARRAY_BUFFER, object.datas[i], recast.glContext.STATIC_DRAW);
    recast.glContext.drawArrays(recast.glContext.TRIANGLES, 0, object.buffers[i].numItems);
  }
};

recast.settings = function (options) {
  recast.set_cellSize(options.cellSize);
  recast.set_cellHeight(options.cellHeight);
  recast.set_agentHeight(options.agentHeight);
  recast.set_agentRadius(options.agentRadius);
  recast.set_agentMaxClimb(options.agentMaxClimb);
  recast.set_agentMaxSlope(options.agentMaxSlope);
};

recast.OBJDataLoader = function (data, callback) {
  _OBJDataLoader(data, callback);
};

recast.OBJLoader = function (path, callback) {
  // with node FS api
  if (ENVIRONMENT_IS_NODE) {
    var fs = require('fs');
    fs.readFile(path, function(err, data) {
      if (err) throw new Error(err);
      _OBJDataLoader(data, callback);
    });

  // with ajax
  } else {
    _ajax(path, {}, function(data) {
      _OBJDataLoader(data, callback);
    });
  }
};

recast.addAgent = function (options) {
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

recast.findNearest = function (position, callback_id) {
  return recast.findNearestPoint(
    position.x,
    position.y,
    position.z,
    3,
    3,
    3,
    callback_id
  );
};

recast.crowdGetActiveAgents = function (callback_id) {
  return recast._crowdGetActiveAgents(callback_id || -1);
};

recast.queryPolygons = function (posX, posY, posZ, extX, extY, extZ, maxPolys, callback_id) {
  if (typeof callback_id === 'undefined' && typeof maxPolys === 'number') {
      callback_id = maxPolys;
      maxPolys = 1000;
  }
  return recast._queryPolygons(posX, posY, posZ, extX, extY, extZ, maxPolys, callback_id);
};

recast.saveTileMesh = function (path, callback_id) {
  recast._saveTileMesh(path, callback_id);
};

recast.loadTileMesh = function (path, callback_id) {
  // with node FS api
  if (ENVIRONMENT_IS_NODE) {
    var fs = require('fs');
    fs.readFile(path, function(err, data) {
      if (err) throw new Error(err);
      // FIXME
      FS.writeFile(path, data, { encoding: 'binary' });
      recast._loadTileMesh(path, callback_id);
    });

  // with ajax
  } else {
    _ajax(path, {}, function(data) {
      // FIXME
      FS.writeFile(path, new Int8Array(data), { encoding: 'binary' });
      recast._loadTileMesh(path, callback_id);
    }, null, 'arraybuffer');
  }
};

recast.saveTileCache = function (path, callback_id) {
  recast._saveTileCache(path, callback_id);
};

recast.loadTileCache = function (path, callback_id) {
  // with node FS api
  if (ENVIRONMENT_IS_NODE) {
    var fs = require('fs');
    fs.readFile(path, function(err, data) {
      if (err) throw new Error(err);
      // FIXME
      FS.writeFile(path, data, { encoding: 'binary' });
      recast._loadTileCache(path, callback_id);
    });

  // with ajax
  } else {
    _ajax(path, {}, function(data) {
      // FIXME
      FS.writeFile(path, new Int8Array(data), { encoding: 'binary' });
      recast._loadTileCache(path, callback_id);
    }, null, 'arraybuffer');
  }
};


recast.zones = { };

recast.setZones = function (zones) {
  var zonesKeys = Object.keys(zones);
  for (var i = zonesKeys.length - 1; i >= 0; i--) {
    var zone = new Zone(zonesKeys[i], zones[zonesKeys[i]]);
    recast.zones[ zone.name ] = zone;
  }
};

//////////////////////////////////////////

recast.Zone = Zone;

function Zone (name, data) {
  this.name  = name;
  this.refs  = data.refs;
  this.flags = 0;

  for (var i = 0; i < data.flags.length; i++) {
    this.setFlags(data.flags[i]);
  }

  this.syncFlags();
}

Zone.prototype.syncFlags = function() {
  for (var i = 0; i < this.refs.length; i++) {
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

//////////////////////////////////////////

function AgentPool (n) {
  this.__pools = [];
  var i = 0;
  while (i < n) {
    this.__pools[i] = { position:{}, velocity:{} };
    i++;
  }
  // debug('__pools is %o length', this.__pools.length);
}

// Get a new array
AgentPool.prototype.get = function(idx,position_x,position_y,position_z,
                                   velocity_x,velocity_y,velocity_z,
                                   radius,active,state,neighbors,
                                   partial, desiredSpeed)
{
  if ( this.__pools.length > 0 ) {
    var ag = this.__pools.pop();
    ag.idx          = idx;
    ag.position.x   = position_x;
    ag.position.y   = position_y;
    ag.position.z   = position_z;
    ag.velocity.x   = velocity_x;
    ag.velocity.y   = velocity_y;
    ag.velocity.z   = velocity_z;
    ag.radius       = radius;
    ag.active       = active;
    ag.state        = state;
    ag.neighbors    = neighbors;
    ag.partial      = partial;
    ag.desiredSpeed = desiredSpeed;
    return ag;
  }

  // console.log( "pool ran out!" )
  return null;
};

// Release an array back into the pool
AgentPool.prototype.add = function( v ) {
  this.__pools.push( v );
};

var agentPool = new AgentPool(1000);
var agentPoolBuffer = [];
agentPool.ready = true; // just to make jshint happy

//////////////////////////////////////////

//////////////////////////////////////////

function VectorPool (n) {
  this.__pools = [];
  var i = 0;
  while (i < n) {
    this.__pools[i] = { x:0, y:0, z:0 };
    i++;
  }
  // debug('__pools is %o length', this.__pools.length);
}

// Get a new array
VectorPool.prototype.get = function(x, y, z) {
  if ( this.__pools.length > 0 ) {
    var v = this.__pools.pop();
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
  }

  // console.log( "pool ran out!" )
  return null;
};

// Release an array back into the pool
VectorPool.prototype.add = function( v ) {
  this.__pools.push( v );
};

var vectorPool = new VectorPool(10000);
vectorPool.ready = true; // just to make jshint happy

//////////////////////////////////////////
