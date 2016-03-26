/*jshint onevar: false, indent:4, strict:false */
/*global setImmediate: false, setTimeout: false, console: false, module: true, process: true, define: true, require: true */

var ENVIRONMENT_IS_NODE = typeof process === 'object' && typeof require === 'function';
var ENVIRONMENT_IS_WEB = typeof window === 'object';

var ENVIRONMENT = 'unkonwn';
if (ENVIRONMENT_IS_WEB) {
    ENVIRONMENT = 'web';
}
if (ENVIRONMENT_IS_NODE) {
    ENVIRONMENT = 'node';
}

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
  // console.log('in context', ENVIRONMENT, 'add a listener on', type, listener);
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
  // console.log('in context', ENVIRONMENT, 'emit a event on', type, this._listeners[type] ? this._listeners[type].length : 0, 'listeners');
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
    throw new Error('Event "' + type + '" don\'t exists');

  var args = Array.prototype.slice.call(arguments, 1);

  process.nextTick(function () {
    // exec event
    self._listeners[type].fn.apply(self, args);

    // remove events that run only once
    if (self._listeners[type].once) self.remove(type);
  });

  return self;
};



/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
var root = this;

var Recast = function (worker_url, onWorkerReady) {

    var recast = this;

    this.vent = new EventEmitter();

    this.__RECAST_CALLBACKS = {};
    this.__RECAST_CALLBACKS.size = 0;

    if (ENVIRONMENT_IS_WEB && typeof Worker !== 'undefined') {

        // Create a new worker and run onReady when it is correctly initialized.

        var spawnWorker = function (workerurl, onReady) {
            var worker = new Worker(workerurl);
            worker.onmessage = function() {
                worker.onmessage = null;
                onReady(worker);
            };
            worker.postMessage({ type:'ping' });
            return worker;
        };

        this.worker = spawnWorker(worker_url, function() {

            recast.worker.addEventListener('message', function(event){
                var message = event.data;

                if (typeof message.callback === 'number') {
                    recast.__RECAST_CALLBACKS[message.callback].apply(event, message.data);
                }

                if (message.vent) {
                    recast.vent.emit(message.type, message.data);
                }
            });

            if (typeof onWorkerReady === 'function') {
                onWorkerReady(recast);
            }
        });

    } else if (ENVIRONMENT_IS_NODE) {
        var child_process = require('child_process');
        this.worker = child_process.fork(worker_url);

        this.worker.postMessage = function(eventData) {
            recast.worker.send({
                type: 'message',
                data: eventData
            });
        };

        this.worker.on('message', function(event){
            var message = event.data;

            if (typeof message.callback === 'number') {
                recast.__RECAST_CALLBACKS[message.callback].apply(event, message.data);
            }
            if (message.vent) {
                recast.vent.emit(message.type, message.data);
            }
        });

        if (typeof onWorkerReady === 'function') {
            onWorkerReady(recast);
        }
    }
};

Recast.prototype.set_cellSize = function (data) {
    this.worker.postMessage({type: 'set_cellSize', data: data });
};

Recast.prototype.set_cellHeight = function (data) {
    this.worker.postMessage({type: 'set_cellHeight', data: data });
};

Recast.prototype.set_agentHeight = function (data) {
    this.worker.postMessage({type: 'set_agentHeight', data: data });
};

Recast.prototype.set_agentRadius = function (data) {
    this.worker.postMessage({type: 'set_agentRadius', data: data });
};

Recast.prototype.set_agentMaxClimb = function (data) {
    this.worker.postMessage({type: 'set_agentMaxClimb', data: data });
};

Recast.prototype.set_agentMaxSlope = function (data) {
    this.worker.postMessage({type: 'set_agentMaxSlope', data: data });
};

Recast.prototype.settings = function (data, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'settings', data: data, callback: callback_id });
};

Recast.prototype.build = function (data, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'build', data: data, callback: callback_id });
};

Recast.prototype.buildSolo = function (data, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'buildSolo', data: data, callback: callback_id });
};

Recast.prototype.buildTiled = function (data, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'buildTiled', data: data, callback: callback_id });
};

Recast.prototype.initCrowd = function (data, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'initCrowd', data: data, callback: callback_id });
};

Recast.prototype.initWithFileContent = function (data, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'initWithFileContent', data: data, callback: callback_id });
};

Recast.prototype.saveTileMesh = function (data, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'saveTileMesh', data: data, callback: callback_id });
};

Recast.prototype.loadTileMesh = function (data, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'loadTileMesh', data: data, callback: callback_id });
};

Recast.prototype.findNearest = function (position, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'findNearestPoint', data: { position: position, extend:{x:3,y:3,z:3} }, callback: callback_id });
};

Recast.prototype.findNearestPoint = function (sx, sy, sz, dx, dy, dz, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'findNearestPoint', data: { position: {x:sx,y:sy,z:sz}, extend:{x:dx,y:dy,z:dz} }, callback: callback_id });
};

Recast.prototype.findNearestPoly = function (sx, sy, sz, dx, dy, dz, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'findNearestPoly', data: { position: {x:sx,y:sy,z:sz}, extend:{x:dx,y:dy,z:dz} }, callback: callback_id });
};

Recast.prototype.queryPolygons = function (sx, sy, sz, dx, dy, dz, maxPolys, callback_id) {
    if (typeof callback_id === 'undefined') {
        callback_id = maxPolys;
        maxPolys    = 1000;
    }
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'queryPolygons', data: { position: {x:sx,y:sy,z:sz}, extend:{x:dx,y:dy,z:dz}, maxPolys:maxPolys }, callback: callback_id });
};

Recast.prototype.setPolyFlags = function (position, radius, flags, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    if (typeof radius === 'number') radius = { x:radius, y:radius, z:radius };
    this.worker.postMessage({type: 'setPolyFlags', data: { sx:position.x, sy:position.x, sz:position.x, dx:radius.x, dy:radius.y, dz:radius.z, flags:flags}, callback: callback_id });
};
Recast.prototype.setPolyFlagsByRef = function (ref, flags, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'setPolyFlagsByRef', data: { ref:ref, flags:flags}, callback: callback_id });
};

Recast.prototype.findPath = function (sx, sy, sz, dx, dy, dz, max, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'findPath', data: { sx:sx, sy:sy, sz:sz, dx:dx, dy:dy, dz:dz, max:max }, callback: callback_id });
};

Recast.prototype.getRandomPoint = function (callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'getRandomPoint', data: null, callback: callback_id });
};

Recast.prototype.addCrowdAgent = function (data, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'addCrowdAgent', data: data, callback: callback_id });
};

Recast.prototype.updateCrowdAgentParameters = function (agent, options, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'updateCrowdAgentParameters', data: { agent: agent, options: options }, callback: callback_id });
};

Recast.prototype.requestMoveVelocity = function (agent, velocity, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'requestMoveVelocity', data: { agent: agent, velocity: velocity }, callback: callback_id });
};

Recast.prototype.addAgent = function (data, callback_id) {
    this.worker.postMessage({ type: 'addCrowdAgent', data: data, callback: callback_id });
};

Recast.prototype.removeCrowdAgent = function (agent, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'removeCrowdAgent', data: agent, callback: callback_id });
};

Recast.prototype.crowdRequestMoveTarget = function (agent, position, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'crowdRequestMoveTarget', data: { agent:agent, x:position.x, y:position.y, z:position.z }, callback: callback_id });
};

Recast.prototype.crowdUpdate = function (data, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'crowdUpdate', data: data, callback: callback_id });
};

Recast.prototype.crowdGetActiveAgents = function (callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({type: 'crowdGetActiveAgents', data: null, callback: callback_id });
};

Recast.prototype.addTempObstacle = function (posX, posY, posZ, radius, callback_id) {
    callback_id = typeof callback_id === 'number' ? callback_id : this.cb(callback_id);
    this.worker.postMessage({ type: 'addTempObstacle', data: { posX:posX, posY:posY, posZ:posZ, radius:radius }, callback: callback_id });
};

Recast.prototype.removeTempObstacle = function (ref, callback_id) {
    this.worker.postMessage({ type: 'removeTempObstacle', data: ref, callback: callback_id });
};

Recast.prototype.removeAllTempObstacles = function (callback_id) {
    this.worker.postMessage({ type: 'removeAllTempObstacles', data: null, callback: callback_id });
};

Recast.prototype.getAllTempObstacles = function (callback_id) {
    this.worker.postMessage({ type: 'getAllTempObstacles', data: null, callback: callback_id });
};

Recast.prototype.addOffMeshConnection = function (startX, startY, startZ,endX, endY, endZ, radius, bidir, callback_id) {
    this.worker.postMessage({ type: 'addOffMeshConnection', data: { startX:startX, startY:startY, startZ:startZ, endX:endX, endY:endY, endZ:endZ, radius:radius, bidir:bidir }, callback: callback_id });
};

Recast.prototype.cb = function (func) {
    if (!func) return null;
    this.__RECAST_CALLBACKS.size = this.__RECAST_CALLBACKS.size % 1000;
    var last = (++this.__RECAST_CALLBACKS.size) - 1;
    this.__RECAST_CALLBACKS[last] = func;
    return last;
};

Recast.prototype.OBJLoader = function(path, callback_id) {
    this.worker.postMessage({type: 'OBJLoader', data: path, callback: callback_id });
};

Recast.prototype.OBJDataLoader = function(objdata, callback_id) {
    this.worker.postMessage({type: 'OBJDataLoader', data: objdata, callback: callback_id });
};

Recast.prototype.getNavMeshVertices = function(callback_id) {
	this.worker.postMessage({type: 'getNavMeshVertices', data: null, callback: callback_id });
};

// Node.js
if (typeof module !== 'undefined' && module.exports) {
    module.exports = Recast;
}
// AMD / RequireJS
else if (typeof define !== 'undefined' && define.amd) {
    define([], function () {
        return Recast;
    });
}
// included directly via <script> tag
else {
    root.Recast = Recast;
}
