/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
/*jshint onevar: false, indent:4 */
/*global mergeInto: true, LibraryManager: true, postMessage: true, Module: true, agentPool: true, agentPoolBuffer: true, ENVIRONMENT_IS_WORKER: true */
(function () {
  'use strict';

  mergeInto(LibraryManager.library, {

    flush_active_agents_callback: function() {
      Module.vent.emit('update', agentPoolBuffer);
      if (typeof window === 'undefined' || typeof importScripts === 'function') {
        postMessage({
          type: 'update',
          vent: true,
          data: agentPoolBuffer
        });
      }
    },

    invoke_vector_callback: function (callback_id, x, y, z) {
      Module.__RECAST_CALLBACKS[callback_id](x, y, z);
    },

    invoke_file_callback: function (callback_id, filename) {
      Module.__RECAST_CALLBACKS[callback_id](null, FS.readFile(Module.Pointer_stringify(filename)));
    },

    invoke_update_callback: function (callback_id) {
      Module.__RECAST_CALLBACKS[callback_id](agentPoolBuffer);
    },

    invoke_generic_callback_string: function (callback_id, data) {
      Module.__RECAST_CALLBACKS[callback_id](JSON.parse(Module.Pointer_stringify(data)));
    },

    gl_create_object: function (objectName) {
      objectName = Module.Pointer_stringify(objectName);
      Module.__RECAST_GLOBAL_CURRENT = objectName;

      Module.__RECAST_OBJECTS[objectName] = {
        buffers: [],
        datas: []
      };
    },

    gl_add_to_object: function () {
      var buffer = Module.glContext.createBuffer();
      var data = new Float32Array(Module.__RECAST_GLOBAL_DATA);
      buffer.itemSize = 3;
      buffer.numItems = data.length / buffer.itemSize;

      Module.__RECAST_OBJECTS[Module.__RECAST_GLOBAL_CURRENT].buffers.push(buffer);
      Module.__RECAST_OBJECTS[Module.__RECAST_GLOBAL_CURRENT].datas.push(data);
    },

    agentPool_clear: function () {
      agentPoolBuffer.length = 0;
    },

    agentPool_add: function (idx) {
      agentPool.add(agentPoolBuffer[idx]);
    },

    agentPool_get: function (idx, position_x, position_y, position_z, velocity_x, velocity_y, velocity_z, radius, active, state, neighbors, partial, desiredSpeed) {
      agentPoolBuffer.push(agentPool.get(idx, position_x, position_y, position_z, velocity_x, velocity_y, velocity_z, radius, active, state, neighbors, partial, desiredSpeed));
    }

  });
})();
