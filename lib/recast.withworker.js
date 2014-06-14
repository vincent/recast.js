/*jshint onevar: false, indent:4, strict:false */
/*global setImmediate: false, setTimeout: false, console: false, module: true, process: true, define: true */

var ENVIRONMENT_IS_NODE = typeof process === 'object' && typeof require === 'function';
var ENVIRONMENT_IS_WEB = typeof window === 'object';
var ENVIRONMENT_IS_WORKER = typeof importScripts === 'function';
var ENVIRONMENT_IS_SHELL = !ENVIRONMENT_IS_WEB && !ENVIRONMENT_IS_NODE && !ENVIRONMENT_IS_WORKER;

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
    throw new Error('Event "' + type + '" don\'t exists');

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

var Worker = (ENVIRONMENT_IS_NODE && typeof Worker === 'undefined') ?
  require('webworker-threads').Worker : Worker;

/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
(function () {

    var root = this;
    var worker_url = '../lib/recast.js';

    var Recast = function (worker_url) {

        var recast = this;

        this.vent = new EventEmitter();

        this.__RECAST_CALLBACKS = {};
        this.__RECAST_CALLBACKS.size = 0;

        this.worker = new Worker(worker_url);

        this.worker.addEventListener('message', function(event){
            var message = event.data;
            if (typeof message.callback === 'number') {
                recast.__RECAST_CALLBACKS[message.callback].apply(event, message.data);
            
            }
            if (message.vent) {
                recast.vent.emit(message.type, message.data);
            } 
        });
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

    Recast.prototype.build = function (data, callback_id) {
        this.worker.postMessage({type: 'build', data: data, callback: callback_id });
    };

    Recast.prototype.initCrowd = function (data, callback_id) {
        this.worker.postMessage({type: 'initCrowd', data: data, callback: callback_id });
    };

    Recast.prototype.initWithFileContent = function (data, callback_id) {
        this.worker.postMessage({type: 'initWithFileContent', data: data, callback: callback_id });
    };

    Recast.prototype.findNearestPoint = function (sx, sy, sz, dx, dy, dz, callback_id) {
        this.worker.postMessage({type: 'findNearestPoint', data: { position: {x:sx,y:sy,z:sz}, extend:{x:dx,y:dy,z:dz} }, callback: callback_id });
    };

    Recast.prototype.findNearestPoly = function (sx, sy, sz, dx, dy, dz, callback_id) {
        this.worker.postMessage({type: 'findNearestPoly', data: { position: {x:sx,y:sy,z:sz}, extend:{x:dx,y:dy,z:dz} }, callback: callback_id });
    };    

    Recast.prototype.findPath = function (sx, sy, sz, dx, dy, dz, max, callback_id) {
        this.worker.postMessage({type: 'findPath', data: { sx:sx, sy:sy, sz:sz, dx:dx, dy:dy, dz:dz, max:max }, callback: callback_id });
    };

    Recast.prototype.getRandomPoint = function (callback_id) {
        this.worker.postMessage({type: 'getRandomPoint', data: null, callback: callback_id });
    };

    Recast.prototype.addCrowdAgent = function (data, callback_id) {
        this.worker.postMessage({type: 'addCrowdAgent', data: data, callback: callback_id });
    };

    Recast.prototype.updateCrowdAgentParameters = function (agent, options, callback_id) {
        this.worker.postMessage({type: 'updateCrowdAgentParameters', data: { agent: agent, options: options }, callback: callback_id });
    };

    Recast.prototype.requestMoveVelocity = function (agent, velocity, callback_id) {
        this.worker.postMessage({type: 'requestMoveVelocity', data: { agent: agent, velocity: velocity }, callback: callback_id });
    };

    Recast.prototype.removeCrowdAgent = function (agent, callback_id) {
        this.worker.postMessage({type: 'removeCrowdAgent', data: agent, callback: callback_id });
    };

    Recast.prototype.crowdRequestMoveTarget = function (agent, position, callback_id) {
        this.worker.postMessage({type: 'crowdRequestMoveTarget', data: { agent:agent, x:position.x, y:position.y, z:position.z }, callback: callback_id });
    };

    Recast.prototype.crowdUpdate = function (data, callback_id) {
        this.worker.postMessage({type: 'crowdUpdate', data: data, callback: callback_id });
    };

    Recast.prototype.crowdGetActiveAgents = function (callback_id) {
        this.worker.postMessage({type: 'crowdGetActiveAgents', data: null, callback: callback_id });
    };

    Recast.prototype.addAgent = function (data, callback_id) {
      return this.worker.postMessage({ type: 'addCrowdAgent', data: data, callback: callback_id });
    };

    Recast.prototype.cb = function (callback) {
      var last = (++ this.__RECAST_CALLBACKS.size) - 1;
      this.__RECAST_CALLBACKS[last] = callback;
      return last;
    };

    Recast.prototype.OBJLoader = function(path, callback_id) {
        this.worker.postMessage({type: 'OBJLoader', data: path, callback: callback_id });
    };

    // Node.js
    if (typeof module !== 'undefined' && module.exports) {
        module.exports = new Recast(worker_url);
    }
    // AMD / RequireJS
    else if (typeof define !== 'undefined' && define.amd) {
        define([], function () {
            return new Recast(worker_url);
        });
    }
    // included directly via <script> tag
    else {
        root.recast = new Recast(worker_url);
    }

})();