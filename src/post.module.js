
var ENVIRONMENT_IS_NODE = typeof process === 'object' && typeof require === 'function';
var ENVIRONMENT_IS_WEB = typeof window === 'object';
var ENVIRONMENT_IS_WORKER = typeof importScripts === 'function';
var ENVIRONMENT_IS_SHELL = !ENVIRONMENT_IS_WEB && !ENVIRONMENT_IS_NODE && !ENVIRONMENT_IS_WORKER;

// Node.js
if (typeof module !== 'undefined' && module.exports) {
    module.exports = Module;
}
// AMD / RequireJS
else if (typeof define !== 'undefined' && define.amd) {
    define([], function () {
        return Module;
    });
}
// included directly via <script> tag
else {
    root.Recast = Module;
}

if (ENVIRONMENT_IS_WORKER) {
  postMessage({ type: 'ready' }); 
}