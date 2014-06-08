/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
/*jshint onevar: false, indent:4, strict: false */
/*global setImmediate: false, setTimeout: false, console: false, module: true, process: true, define: true */

// Custom Module definition from recast.js library
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

recast.noConflict = function () {
    root.recast = previous_recast;
    return recast;
};

function only_once(fn) {
    var called = false;
    return function() {
        if (called) { throw new Error('Callback was already called.'); }
        called = true;
        fn.apply(root, arguments);
    };
}

//// cross-browser compatiblity functions ////

var _toString = Object.prototype.toString;

var _isArray = Array.isArray || function (obj) {
    return _toString.call(obj) === '[object Array]';
};

var _each = function (arr, iterator) {
    if (arr.forEach) {
        return arr.forEach(iterator);
    }
    for (var i = 0; i < arr.length; i += 1) {
        iterator(arr[i], i, arr);
    }
};

var _map = function (arr, iterator) {
    if (arr.map) {
        return arr.map(iterator);
    }
    var results = [];
    _each(arr, function (x, i, a) {
        results.push(iterator(x, i, a));
    });
    return results;
};

var _reduce = function (arr, iterator, memo) {
    if (arr.reduce) {
        return arr.reduce(iterator, memo);
    }
    _each(arr, function (x, i, a) {
        memo = iterator(memo, x, i, a);
    });
    return memo;
};

var _keys = function (obj) {
    if (Object.keys) {
        return Object.keys(obj);
    }
    var keys = [];
    for (var k in obj) {
        if (obj.hasOwnProperty(k)) {
            keys.push(k);
        }
    }
    return keys;
};

var _ajax = function(url, data, callback, type) {
  var data_array, data_string, idx, req, value;
  if (data == null) {
    data = {};
  }
  if (callback == null) {
    callback = function() {};
  }
  if (type == null) {
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
  req.open(type, url, false);
  req.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  req.onreadystatechange = function() {
    if (req.readyState === 4 && req.status === 200) {
      return callback(req.responseText);
    }
  };
  // debug('ajax request', data_string);
  req.send(data_string);
  return req;
};

Module.OBJUrlLoader = function (url, callback) {
  _ajax(url, {}, function(data) {
    console.log('ajax success: ' + data.length);
    Module.initWithFileContent(data);
    console.log('init success');
    Module.build();
    console.log('build success');
    Module.initCrowd(100, 1.0);
    console.log('crowd success');
    callback(Module);
  });
};

Module.cb = function (func) {
  var last = (++Module.__RECAST_CALLBACKS.size) - 1;
  Module.__RECAST_CALLBACKS[last] = func;
  return last;
};



//// exported recast module functions ////

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




